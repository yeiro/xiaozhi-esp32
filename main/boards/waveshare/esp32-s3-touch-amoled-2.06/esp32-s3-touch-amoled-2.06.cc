#include "wifi_board.h"
#include "display/lcd_display.h"
#include "esp_lcd_sh8601.h"

#include "codecs/box_audio_codec.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "mcp_server.h"
#include "config.h"
#include "power_save_timer.h"
#include "axp2101.h"
#include "i2c_device.h"

#include <esp_log.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include "settings.h"

#include <esp_lcd_touch_ft5x06.h>
#include <esp_lvgl_port.h>
#include <lvgl.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <atomic>
#include <array>
#include <algorithm>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "WaveshareEsp32s3TouchAMOLED2inch06"

class Pmic : public Axp2101 {
public:
    Pmic(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : Axp2101(i2c_bus, addr) {
        WriteReg(0x22, 0b110); // PWRON > OFFLEVEL as POWEROFF Source enable
        WriteReg(0x27, 0x10);  // hold 4s to power off

        // Disable All DCs but DC1
        WriteReg(0x80, 0x01);
        // Disable All LDOs
        WriteReg(0x90, 0x00);
        WriteReg(0x91, 0x00);

        // Set DC1 to 3.3V
        WriteReg(0x82, (3300 - 1500) / 100);

        // Set ALDO1 to 3.3V
        WriteReg(0x92, (3300 - 500) / 100);
        WriteReg(0x93, (3300 - 500) / 100);

        // Enable ALDO1(MIC)
        WriteReg(0x90, 0x03);

        WriteReg(0x64, 0x02); // CV charger voltage setting to 4.1V

        WriteReg(0x61, 0x02); // set Main battery precharge current to 50mA
        WriteReg(0x62, 0x0A); // set Main battery charger current to 400mA ( 0x08-200mA, 0x09-300mA, 0x0A-400mA )
        WriteReg(0x63, 0x01); // set Main battery term charge current to 25mA
    }
};

#define LCD_OPCODE_WRITE_CMD (0x02ULL)
#define LCD_OPCODE_READ_CMD (0x03ULL)
#define LCD_OPCODE_WRITE_COLOR (0x32ULL)

static const sh8601_lcd_init_cmd_t vendor_specific_init[] = {
    // set display to qspi mode
    {0x11, (uint8_t []){0x00}, 0, 120},
    {0xC4, (uint8_t []){0x80}, 1, 0},
    {0x44, (uint8_t []){0x01, 0xD1}, 2, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x53, (uint8_t []){0x20}, 1, 10},
    {0x63, (uint8_t []){0xFF}, 1, 10},
    {0x51, (uint8_t []){0x00}, 1, 10},
    {0x2A, (uint8_t []){0x00,0x16,0x01,0xAF}, 4, 0},
    {0x2B, (uint8_t []){0x00,0x00,0x01,0xF5}, 4, 0},
    {0x29, (uint8_t []){0x00}, 0, 10},
    {0x51, (uint8_t []){0xFF}, 1, 0},
};

// Custom AMOLED display with 3D glowing particle & voice-wave orb
class CustomLcdDisplay : public SpiLcdDisplay {
private:
    static constexpr int kNumParticles = 96;
    struct Vec3 {
        float x, y, z;
    };
    std::array<Vec3, kNumParticles> base_points_;
    lv_obj_t* orb_obj_ = nullptr;
    lv_obj_t* watch_header_ = nullptr;
    lv_obj_t* watch_time_label_ = nullptr;
    lv_obj_t* watch_info_label_ = nullptr;
    lv_timer_t* orb_timer_ = nullptr;
    float phase_ = 0.0f;
    float rot_y_ = 0.0f;
    float energy_ = 0.08f;
    float tap_pulse_ = 0.0f;
    float speak_blend_ = 0.0f;
    float listen_blend_ = 0.0f;
    int64_t last_tap_us_ = 0;
    int64_t last_activity_us_ = 0;
    bool screen_sleeping_ = false;
    int last_watch_sec_ = -1;
    std::atomic<int> caracas_temp_c_{-999};

    void WakeScreen() {
        last_activity_us_ = esp_timer_get_time();
        if (screen_sleeping_) {
            screen_sleeping_ = false;
            Board::GetInstance().GetBacklight()->RestoreBrightness();
            if (orb_obj_) {
                lv_obj_invalidate(orb_obj_);
            }
        }
    }

    void SleepScreen() {
        if (!screen_sleeping_) {
            screen_sleeping_ = true;
            Board::GetInstance().GetBacklight()->SetBrightness(0);
        }
    }

    static void weather_fetch_task(void* arg) {
        auto* self = static_cast<CustomLcdDisplay*>(arg);
        // Wait for Wi-Fi connection before first fetch
        vTaskDelay(pdMS_TO_TICKS(8000));
        while (true) {
            auto state = Application::GetInstance().GetDeviceState();
            if (state != kDeviceStateStarting && state != kDeviceStateWifiConfiguring) {
                auto network = Board::GetInstance().GetNetwork();
                if (network) {
                    auto http = network->CreateHttp(0);
                    if (http) {
                        const char* url = "http://api.open-meteo.com/v1/forecast?latitude=10.4806&longitude=-66.9036&current=temperature_2m";
                        if (auto opened = http->Open("GET", url); opened) {
                            std::string body = http->ReadAll();
                            http->Close();
                            auto pos = body.rfind("\"temperature_2m\":");
                            if (pos != std::string::npos) {
                                float val = std::strtof(body.c_str() + pos + 17, nullptr);
                                if (val > -50.0f && val < 65.0f) {
                                    self->caracas_temp_c_.store(static_cast<int>(std::lround(val)));
                                    ESP_LOGI(TAG, "Caracas temp updated: %d C", self->caracas_temp_c_.load());
                                }
                            }
                        }
                    }
                }
            }
            // Refresh every 15 minutes (or retry in 30s if not fetched yet)
            vTaskDelay(pdMS_TO_TICKS(self->caracas_temp_c_.load() == -999 ? 30000 : 900000));
        }
    }

    void UpdateWatchHeader() {
        if (!watch_time_label_ || !watch_info_label_) return;
        time_t now = time(nullptr);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        if (tm_now.tm_year + 1900 < 2024) {
            // Time not synced yet
            lv_label_set_text(watch_time_label_, "--:--");
            int temp = caracas_temp_c_.load();
            if (temp != -999) {
                char buf[48];
                snprintf(buf, sizeof(buf), "Caracas %dC", temp);
                lv_label_set_text(watch_info_label_, buf);
            } else {
                lv_label_set_text(watch_info_label_, "Caracas, VE");
            }
            return;
        }

        if (tm_now.tm_sec == last_watch_sec_) return;
        last_watch_sec_ = tm_now.tm_sec;

        // 12-hour clean smartwatch format (e.g. 10:05)
        char time_buf[16];
        int h12 = tm_now.tm_hour % 12;
        if (h12 == 0) h12 = 12;
        snprintf(time_buf, sizeof(time_buf), "%02d:%02d", h12, tm_now.tm_min);
        lv_label_set_text(watch_time_label_, time_buf);

        static const char* kDaysEs[] = {"Dom", "Lun", "Mar", "Mie", "Jue", "Vie", "Sab"};
        static const char* kMonthsEs[] = {"Ene", "Feb", "Mar", "Abr", "May", "Jun",
                                          "Jul", "Ago", "Sep", "Oct", "Nov", "Dic"};
        const char* ampm = (tm_now.tm_hour >= 12) ? "PM" : "AM";
        int temp = caracas_temp_c_.load();
        char info_buf[64];
        if (temp != -999) {
            snprintf(info_buf, sizeof(info_buf), "%s %d %s %s | Caracas %dC",
                     kDaysEs[tm_now.tm_wday % 7], tm_now.tm_mday,
                     kMonthsEs[tm_now.tm_mon % 12], ampm, temp);
        } else {
            snprintf(info_buf, sizeof(info_buf), "%s %d %s %s | Caracas",
                     kDaysEs[tm_now.tm_wday % 7], tm_now.tm_mday,
                     kMonthsEs[tm_now.tm_mon % 12], ampm);
        }
        lv_label_set_text(watch_info_label_, info_buf);
    }

    void InitFibonacciSphere() {
        const float golden_angle = 2.39996323f; // pi * (3 - sqrt(5))
        for (int i = 0; i < kNumParticles; ++i) {
            float y = 1.0f - (static_cast<float>(i) / static_cast<float>(kNumParticles - 1)) * 2.0f;
            float r = std::sqrt(std::max(0.0f, 1.0f - y * y));
            float theta = golden_angle * static_cast<float>(i);
            base_points_[i] = {std::cos(theta) * r, y, std::sin(theta) * r};
        }
    }

    static uint32_t LerpHex(uint32_t hex_a, uint32_t hex_b, float t) {
        t = std::clamp(t, 0.0f, 1.0f);
        uint8_t ra = (hex_a >> 16) & 0xFF, ga = (hex_a >> 8) & 0xFF, ba = hex_a & 0xFF;
        uint8_t rb = (hex_b >> 16) & 0xFF, gb = (hex_b >> 8) & 0xFF, bb = hex_b & 0xFF;
        uint8_t r = static_cast<uint8_t>(ra + (rb - ra) * t);
        uint8_t g = static_cast<uint8_t>(ga + (gb - ga) * t);
        uint8_t b = static_cast<uint8_t>(ba + (bb - ba) * t);
        return (static_cast<uint32_t>(r) << 16) | (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
    }

    static lv_color_t LerpColor(uint32_t hex_a, uint32_t hex_b, float t) {
        return lv_color_hex(LerpHex(hex_a, hex_b, t));
    }

    static void DrawCircle(lv_layer_t* layer, int32_t cx, int32_t cy, int32_t radius,
                           lv_color_t color, lv_opa_t opa) {
        if (radius <= 0 || opa == LV_OPA_TRANSP) return;
        lv_draw_rect_dsc_t dsc;
        lv_draw_rect_dsc_init(&dsc);
        dsc.radius = LV_RADIUS_CIRCLE;
        dsc.bg_color = color;
        dsc.bg_opa = opa;
        dsc.border_width = 0;
        lv_area_t area = {cx - radius, cy - radius, cx + radius, cy + radius};
        lv_draw_rect(layer, &dsc, &area);
    }

    static void DrawRing(lv_layer_t* layer, int32_t cx, int32_t cy, int32_t radius,
                         int32_t width, lv_color_t color, lv_opa_t opa) {
        if (radius <= 0 || width <= 0 || opa == LV_OPA_TRANSP) return;
        lv_draw_rect_dsc_t dsc;
        lv_draw_rect_dsc_init(&dsc);
        dsc.radius = LV_RADIUS_CIRCLE;
        dsc.bg_opa = LV_OPA_TRANSP;
        dsc.border_width = width;
        dsc.border_color = color;
        dsc.border_opa = opa;
        lv_area_t area = {cx - radius, cy - radius, cx + radius, cy + radius};
        lv_draw_rect(layer, &dsc, &area);
    }

    void ApplyAmoledBlackTheme() {
        lv_color_t black = lv_color_hex(0x000000);
        lv_color_t text_light = lv_color_hex(0xF1F5F9);
        lv_color_t icon_accent = lv_color_hex(0x93C5FD);

        lv_obj_t* screen = lv_screen_active();
        if (screen) {
            lv_obj_set_style_bg_color(screen, black, 0);
            lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
        }
        if (container_) {
            lv_obj_set_style_bg_color(container_, black, 0);
            lv_obj_set_style_bg_opa(container_, LV_OPA_COVER, 0);
        }
        if (top_bar_) {
            lv_obj_set_style_bg_color(top_bar_, black, 0);
            lv_obj_set_style_bg_opa(top_bar_, LV_OPA_COVER, 0);
            lv_obj_set_style_pad_left(top_bar_, LV_HOR_RES * 0.08, 0);
            lv_obj_set_style_pad_right(top_bar_, LV_HOR_RES * 0.08, 0);
        }
        if (status_bar_) {
            // MUST remain transparent so top_bar_ (Wi-Fi & Battery icons) stays visible!
            lv_obj_set_style_bg_opa(status_bar_, LV_OPA_TRANSP, 0);
            lv_obj_set_style_text_color(status_bar_, text_light, 0);
        }
        if (content_) {
            lv_obj_set_style_bg_color(content_, black, 0);
            lv_obj_set_style_bg_opa(content_, LV_OPA_COVER, 0);
        }
        if (emoji_box_) {
            lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        }
        if (bottom_bar_) {
            lv_obj_set_style_bg_color(bottom_bar_, black, 0);
            lv_obj_set_style_bg_opa(bottom_bar_, LV_OPA_TRANSP, 0);
        }
        if (status_label_) lv_obj_set_style_text_color(status_label_, text_light, 0);
        if (network_label_) lv_obj_set_style_text_color(network_label_, icon_accent, 0);
        if (battery_label_) lv_obj_set_style_text_color(battery_label_, lv_color_hex(0x34D399), 0);
        if (chat_message_label_) lv_obj_set_style_text_color(chat_message_label_, text_light, 0);
        if (emoji_label_) lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
        if (emoji_image_) lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
    }

    static void touch_indev_cb(lv_event_t* e) {
        auto* self = static_cast<CustomLcdDisplay*>(lv_event_get_user_data(e));
        if (!self) return;
        int64_t now_us = esp_timer_get_time();

        // Debounce rapid hardware touch bounce (< 80 ms)
        if (self->last_tap_us_ > 0 && (now_us - self->last_tap_us_) < 80000) {
            return;
        }

        // 1-tap wakes up the AMOLED screen immediately from sleep
        if (self->screen_sleeping_) {
            ESP_LOGI(TAG, "Touch 1-tap -> Wake AMOLED screen");
            self->WakeScreen();
            self->last_tap_us_ = now_us;
            return;
        }

        self->last_activity_us_ = now_us;
        // Double-tap (80 ms .. 480 ms) triggers the same action as the BOOT button
        if (self->last_tap_us_ > 0 && (now_us - self->last_tap_us_) < 480000) {
            ESP_LOGI(TAG, "Touch Double-tap -> ToggleChatState");
            self->last_tap_us_ = 0;
            self->tap_pulse_ = 1.0f;
            Application::GetInstance().ToggleChatState();
        } else {
            self->last_tap_us_ = now_us;
            self->tap_pulse_ = 0.45f; // Subtle visual feedback on single tap
        }
    }

    static void orb_timer_cb(lv_timer_t* timer) {
        auto* self = static_cast<CustomLcdDisplay*>(lv_timer_get_user_data(timer));
        if (!self || !self->orb_obj_) return;

        int64_t now_us = esp_timer_get_time();
        auto state = Application::GetInstance().GetDeviceState();

        // Any active state wakes up the AMOLED screen automatically (e.g. "Hi, ESP")
        if (state != kDeviceStateIdle) {
            self->WakeScreen();
        } else {
            // Auto screen-off after 25 seconds in idle to save battery on AMOLED
            if (!self->screen_sleeping_ && self->last_activity_us_ > 0 &&
                (now_us - self->last_activity_us_) > 25000000LL) {
                self->SleepScreen();
            }
            if (self->screen_sleeping_) {
                return; // Skip redrawing while AMOLED is off -> ultra-low power!
            }
        }
        float target_energy = 0.08f;
        float target_speak = 0.0f;
        float target_listen = 0.0f;
        float rot_speed = 0.016f;

        if (state == kDeviceStateSpeaking) {
            target_energy = 1.0f;
            target_speak = 1.0f;
            rot_speed = 0.042f;
        } else if (state == kDeviceStateListening) {
            target_energy = 0.55f;
            target_listen = 1.0f;
            rot_speed = 0.028f;
        } else if (state == kDeviceStateConnecting || state == kDeviceStateStarting || state == kDeviceStateActivating) {
            target_energy = 0.35f;
            rot_speed = 0.050f;
        }

        // Smooth interpolation for refined, fluid transitions
        self->energy_ += (target_energy - self->energy_) * 0.14f;
        self->speak_blend_ += (target_speak - self->speak_blend_) * 0.12f;
        self->listen_blend_ += (target_listen - self->listen_blend_) * 0.12f;
        self->phase_ += 0.055f + self->energy_ * 0.045f;
        self->rot_y_ += rot_speed;
        if (self->tap_pulse_ > 0.01f) {
            self->tap_pulse_ *= 0.86f;
        } else {
            self->tap_pulse_ = 0.0f;
        }

        self->UpdateWatchHeader();
        lv_obj_invalidate(self->orb_obj_);
    }

    static void orb_draw_cb(lv_event_t* e) {
        auto* self = static_cast<CustomLcdDisplay*>(lv_event_get_user_data(e));
        lv_obj_t* obj = static_cast<lv_obj_t*>(lv_event_get_target(e));
        lv_layer_t* layer = lv_event_get_layer(e);
        if (!self || !obj || !layer) return;

        lv_area_t coords;
        lv_obj_get_coords(obj, &coords);
        int32_t cx = (coords.x1 + coords.x2) / 2;
        int32_t cy = (coords.y1 + coords.y2) / 2;
        int32_t half_size = std::min(coords.x2 - coords.x1, coords.y2 - coords.y1) / 2;

        float t = self->phase_;
        float e_val = self->energy_;

        // Multi-harmonic organic voice wave modulation
        float voice_wave = 0.0f;
        if (self->speak_blend_ > 0.02f) {
            float w = 0.50f * std::sin(t * 2.8f) +
                      0.30f * std::sin(t * 5.3f + 1.1f) +
                      0.20f * std::cos(t * 8.1f + 2.4f);
            voice_wave = (0.5f + 0.5f * w) * self->speak_blend_;
        }
        float idle_breath = std::sin(t * 0.9f);
        float base_r = static_cast<float>(half_size) * (0.60f + 0.025f * idle_breath + 0.09f * voice_wave + 0.03f * self->listen_blend_);
        int32_t r_core = static_cast<int32_t>(base_r);

        // Luxury color palette blending:
        // Idle: Midnight Sapphire (0x0A1336 -> 0x1E3A8A -> 0x3B82F6)
        // Listening: Electric Cyan / Deep Teal (0x062038 -> 0x0E7490 -> 0x22D3EE)
        // Speaking: Royal Violet + Electric Blue + Cyan Aura (0x1B1045 -> 0x4F46E5 -> 0x38BDF8)
        lv_color_t outer_glow_col = lv_color_hex(LerpHex(LerpHex(0x1E3A8A, 0x06B6D4, self->listen_blend_), 0x6366F1, self->speak_blend_));
        lv_color_t core_dark      = lv_color_hex(LerpHex(LerpHex(0x070E28, 0x062336, self->listen_blend_), 0x130D36, self->speak_blend_));
        lv_color_t core_mid       = lv_color_hex(LerpHex(LerpHex(0x162E7A, 0x0C5B78, self->listen_blend_), 0x3730A3, self->speak_blend_));
        lv_color_t core_bright    = lv_color_hex(LerpHex(LerpHex(0x3B6EF5, 0x22D3EE, self->listen_blend_), 0x7C3AED, self->speak_blend_));
        lv_color_t wave_col       = lv_color_hex(LerpHex(LerpHex(0x60A5FA, 0x38BDF8, self->listen_blend_), 0xA78BFA, self->speak_blend_ * 0.6f));

        // 1. Soft ambient outer halo rings (AMOLED glow)
        DrawCircle(layer, cx, cy, r_core + 34 + static_cast<int32_t>(voice_wave * 12), outer_glow_col, 22);
        DrawCircle(layer, cx, cy, r_core + 20 + static_cast<int32_t>(voice_wave * 8), outer_glow_col, 42);
        DrawCircle(layer, cx, cy, r_core + 8 + static_cast<int32_t>(voice_wave * 4), outer_glow_col, 68);

        // 2. Rhythmic expanding sound-wave rings when speaking or listening
        if (self->speak_blend_ > 0.05f || self->listen_blend_ > 0.05f) {
            float activity = std::max(self->speak_blend_, self->listen_blend_ * 0.7f);
            for (int w_idx = 0; w_idx < 3; ++w_idx) {
                float p = std::fmod(t * (0.28f + 0.14f * self->speak_blend_) + w_idx * 0.333f, 1.0f);
                int32_t wr = r_core + static_cast<int32_t>(p * (half_size - r_core + 6));
                lv_opa_t wopa = static_cast<lv_opa_t>((1.0f - p) * (1.0f - p) * 180.0f * activity);
                int32_t wthick = (w_idx == 0) ? 3 : 2;
                DrawRing(layer, cx, cy, wr, wthick, wave_col, wopa);
            }
        }

        // 3. Tap feedback ripple
        if (self->tap_pulse_ > 0.02f) {
            float inv = 1.0f - self->tap_pulse_;
            int32_t tr = r_core + static_cast<int32_t>(inv * 46.0f);
            lv_opa_t topa = static_cast<lv_opa_t>(self->tap_pulse_ * 220.0f);
            DrawRing(layer, cx, cy, tr, 3, lv_color_hex(0xE0F2FE), topa);
        }

        // 4. Layered 3D luminous sphere core
        DrawCircle(layer, cx, cy, r_core, core_dark, LV_OPA_COVER);
        DrawCircle(layer, cx - r_core / 10, cy - r_core / 10, (r_core * 82) / 100, core_mid, 175);
        DrawCircle(layer, cx - r_core / 6, cy - r_core / 6, (r_core * 52) / 100, core_bright, 145);
        DrawCircle(layer, cx - r_core / 5, cy - r_core / 5, (r_core * 26) / 100, lv_color_hex(0x93C5FD), 70);

        // Subtle rim light around the orb edge
        DrawRing(layer, cx, cy, r_core, 2, wave_col, static_cast<lv_opa_t>(90 + e_val * 90));

        // 5. 3D Rotating Fibonacci Particle Field (Back hemisphere first, then Front)
        float cos_y = std::cos(self->rot_y_);
        float sin_y = std::sin(self->rot_y_);
        float tilt_x = 0.28f + 0.10f * std::sin(t * 0.45f);
        float cos_x = std::cos(tilt_x);
        float sin_x = std::sin(tilt_x);

        float dot_orbit_r = base_r * 0.90f;

        for (int pass = 0; pass < 2; ++pass) {
            for (int i = 0; i < kNumParticles; ++i) {
                const auto& pt = self->base_points_[i];

                // Rotate around Y
                float x1 = pt.x * cos_y + pt.z * sin_y;
                float z1 = -pt.x * sin_y + pt.z * cos_y;
                // Tilt around X
                float y2 = pt.y * cos_x - z1 * sin_x;
                float z2 = pt.y * sin_x + z1 * cos_x;

                bool is_front = (z2 >= 0.0f);
                if ((pass == 0 && is_front) || (pass == 1 && !is_front)) {
                    continue;
                }

                // Individual particle rhythmic wave displacement when speaking
                float ripple = 1.0f;
                if (self->speak_blend_ > 0.02f) {
                    float band = std::sin(pt.y * 5.0f - t * 4.5f + pt.x * 3.0f);
                    ripple += 0.14f * band * voice_wave;
                } else if (self->listen_blend_ > 0.02f) {
                    float band = std::sin(pt.y * 4.0f - t * 2.5f);
                    ripple += 0.05f * band * self->listen_blend_;
                }

                float depth = (z2 + 1.0f) * 0.5f; // 0 (back) .. 1 (front)
                float persp = 0.86f + 0.22f * depth;

                int32_t px = cx + static_cast<int32_t>(x1 * dot_orbit_r * ripple * persp);
                int32_t py = cy + static_cast<int32_t>(y2 * dot_orbit_r * ripple * persp);

                int32_t dot_r = is_front
                    ? static_cast<int32_t>(2.8f + depth * 3.0f + voice_wave * 1.4f)
                    : static_cast<int32_t>(1.6f + depth * 1.8f);

                lv_opa_t dot_opa = is_front
                    ? static_cast<lv_opa_t>(135 + depth * 120)
                    : static_cast<lv_opa_t>(40 + depth * 80);

                lv_color_t dot_col = is_front
                    ? LerpColor(0xBAE6FD, 0xFFFFFF, depth)
                    : LerpColor(0x3B82F6, 0x93C5FD, depth);

                if (is_front && depth > 0.65f) {
                    // Subtle soft halo behind brightest front particles
                    DrawCircle(layer, px, py, dot_r + 2, wave_col, static_cast<lv_opa_t>(dot_opa / 4));
                }
                DrawCircle(layer, px, py, dot_r, dot_col, dot_opa);
            }
        }
    }

public:
    static void rounder_event_cb(lv_event_t* e) {
        lv_area_t* area = (lv_area_t* )lv_event_get_param(e);
        uint16_t x1 = area->x1;
        uint16_t x2 = area->x2;

        uint16_t y1 = area->y1;
        uint16_t y2 = area->y2;

        // round the start of coordinate down to the nearest 2M number
        area->x1 = (x1 >> 1) << 1;
        area->y1 = (y1 >> 1) << 1;
        // round the end of coordinate up to the nearest 2N+1 number
        area->x2 = ((x2 >> 1) << 1) + 1;
        area->y2 = ((y2 >> 1) << 1) + 1;
    }

    CustomLcdDisplay(esp_lcd_panel_io_handle_t io_handle,
                     esp_lcd_panel_handle_t panel_handle,
                     int width,
                     int height,
                     int offset_x,
                     int offset_y,
                     bool mirror_x,
                     bool mirror_y,
                     bool swap_xy)
        : SpiLcdDisplay(io_handle, panel_handle,
                        width, height, offset_x, offset_y, mirror_x, mirror_y, swap_xy) {
        InitFibonacciSphere();
    }

    ~CustomLcdDisplay() {
        if (orb_timer_) {
            lv_timer_delete(orb_timer_);
            orb_timer_ = nullptr;
        }
    }

    virtual void SetupUI() override {
        SpiLcdDisplay::SetupUI();

        DisplayLockGuard lock(this);
        lv_obj_set_style_pad_left(status_bar_, LV_HOR_RES * 0.1, 0);
        lv_obj_set_style_pad_right(status_bar_, LV_HOR_RES * 0.1, 0);
        lv_display_add_event_cb(display_, rounder_event_cb, LV_EVENT_INVALIDATE_AREA, NULL);

        ApplyAmoledBlackTheme();

        lv_obj_t* screen = lv_screen_active();

        // Smartwatch Header: Big crisp clock + Date + Live Caracas Temperature (explicit full width!)
        watch_header_ = lv_obj_create(screen);
        lv_obj_remove_style_all(watch_header_);
        lv_obj_set_size(watch_header_, LV_HOR_RES, LV_SIZE_CONTENT);
        lv_obj_align(watch_header_, LV_ALIGN_TOP_MID, 0, 48);
        lv_obj_set_flex_flow(watch_header_, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(watch_header_, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
        lv_obj_set_style_pad_row(watch_header_, 4, 0);

        watch_time_label_ = lv_label_create(watch_header_);
        lv_obj_set_style_text_font(watch_time_label_, &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(watch_time_label_, lv_color_hex(0xF8FAFC), 0);
        lv_label_set_text(watch_time_label_, "--:--");

        watch_info_label_ = lv_label_create(watch_header_);
        lv_obj_set_style_text_font(watch_info_label_, &lv_font_montserrat_20, 0);
        lv_obj_set_style_text_color(watch_info_label_, lv_color_hex(0x7DD3FC), 0);
        lv_label_set_text(watch_info_label_, "Caracas, VE");

        // Create interactive 3D Particle & Voice-Wave Orb centered below the watch header
        orb_obj_ = lv_obj_create(screen);
        lv_obj_remove_style_all(orb_obj_);
        lv_obj_set_size(orb_obj_, 250, 250);
        lv_obj_align(orb_obj_, LV_ALIGN_CENTER, 0, 52);
        lv_obj_add_event_cb(orb_obj_, orb_draw_cb, LV_EVENT_DRAW_MAIN, this);

        // Keep top_bar_, status_bar_, bottom_bar_ above background
        if (top_bar_) lv_obj_move_foreground(top_bar_);
        if (status_bar_) lv_obj_move_foreground(status_bar_);
        if (bottom_bar_) lv_obj_move_foreground(bottom_bar_);

        last_activity_us_ = esp_timer_get_time();
        orb_timer_ = lv_timer_create(orb_timer_cb, 33, this);
        xTaskCreate(weather_fetch_task, "weather_ccs", 4096, this, 1, nullptr);
    }

    void AttachTouchIndev(lv_indev_t* indev) {
        if (!indev) return;
        DisplayLockGuard lock(this);
        lv_indev_add_event_cb(indev, touch_indev_cb, LV_EVENT_PRESSED, this);
        ESP_LOGI(TAG, "Global touch indev listener attached (1-tap wake / double-tap talk)");
    }

    virtual bool SetEmotion(const char* emotion) override {
        // Keep our 3D glowing orb visible instead of default emoji faces/GIFs
        DisplayLockGuard lock(this);
        if (emoji_box_) lv_obj_add_flag(emoji_box_, LV_OBJ_FLAG_HIDDEN);
        if (emoji_label_) lv_obj_add_flag(emoji_label_, LV_OBJ_FLAG_HIDDEN);
        if (emoji_image_) lv_obj_add_flag(emoji_image_, LV_OBJ_FLAG_HIDDEN);
        return true;
    }

    virtual void SetTheme(Theme* theme) override {
        SpiLcdDisplay::SetTheme(theme);
        DisplayLockGuard lock(this);
        ApplyAmoledBlackTheme();
    }
};

class CustomBacklight : public Backlight {
public:
    CustomBacklight(esp_lcd_panel_io_handle_t panel_io) : Backlight(), panel_io_(panel_io) {}

protected:
    esp_lcd_panel_io_handle_t panel_io_;

    virtual void SetBrightnessImpl(uint8_t brightness) override {
        auto display = Board::GetInstance().GetDisplay();
        DisplayLockGuard lock(display);
        uint8_t data[1] = {((uint8_t)((255*  brightness) / 100))};
        int lcd_cmd = 0x51;
        lcd_cmd &= 0xff;
        lcd_cmd <<= 8;
        lcd_cmd |= LCD_OPCODE_WRITE_CMD << 24;
        esp_lcd_panel_io_tx_param(panel_io_, lcd_cmd, &data, sizeof(data));
    }
};

class WaveshareEsp32s3TouchAMOLED2inch06 : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Pmic* pmic_ = nullptr;
    Button boot_button_;
    CustomLcdDisplay* display_;
    CustomBacklight* backlight_;
    PowerSaveTimer* power_save_timer_;

    void InitializePowerSaveTimer() {
        power_save_timer_ = new PowerSaveTimer(-1, 60, 300);
        power_save_timer_->OnEnterSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(true);
            GetBacklight()->SetBrightness(20); });
        power_save_timer_->OnExitSleepMode([this]() {
            GetDisplay()->SetPowerSaveMode(false);
            GetBacklight()->RestoreBrightness(); });
        power_save_timer_->OnShutdownRequest([this](){ 
            pmic_->PowerOff(); });
        power_save_timer_->SetEnabled(true);
    }

    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeAxp2101() {
        ESP_LOGI(TAG, "Init AXP2101");
        pmic_ = new Pmic(i2c_bus_, 0x34);
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.sclk_io_num = EXAMPLE_PIN_NUM_LCD_PCLK;
        buscfg.data0_io_num = EXAMPLE_PIN_NUM_LCD_DATA0;
        buscfg.data1_io_num = EXAMPLE_PIN_NUM_LCD_DATA1;
        buscfg.data2_io_num = EXAMPLE_PIN_NUM_LCD_DATA2;
        buscfg.data3_io_num = EXAMPLE_PIN_NUM_LCD_DATA3;
        buscfg.max_transfer_sz = DISPLAY_WIDTH*  DISPLAY_HEIGHT*  sizeof(uint16_t);
        buscfg.flags = SPICOMMON_BUSFLAG_QUAD;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });

#if CONFIG_USE_DEVICE_AEC
        boot_button_.OnDoubleClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateIdle) {
                app.SetAecMode(app.GetAecMode() == kAecOff ? kAecOnDeviceSide : kAecOff);
            }
        });
#endif
    }

    void InitializeSH8601Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS;
        io_config.dc_gpio_num = GPIO_NUM_NC;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 32;
        io_config.lcd_param_bits = 8;
        io_config.flags.quad_mode = true;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");
        const sh8601_vendor_config_t vendor_config = {
            .init_cmds = &vendor_specific_init[0],
            .init_cmds_size = sizeof(vendor_specific_init) / sizeof(sh8601_lcd_init_cmd_t),
            .flags = {
                .use_qspi_interface = 1,
            }};

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        panel_config.vendor_config = (void* )&vendor_config;
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh8601(panel_io, &panel_config, &panel));
        esp_lcd_panel_set_gap(panel, 0x16, 0);
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, false);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        esp_lcd_panel_disp_on_off(panel, true);
        display_ = new CustomLcdDisplay(panel_io, panel,
                                        DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        backlight_ = new CustomBacklight(panel_io);
        backlight_->RestoreBrightness();
    }

    void InitializeTouch() {
        esp_lcd_touch_handle_t tp;
        esp_lcd_touch_config_t tp_cfg = {
            .x_max = DISPLAY_WIDTH - 1,
            .y_max = DISPLAY_HEIGHT - 1,
            .rst_gpio_num = GPIO_NUM_9,
            .int_gpio_num = GPIO_NUM_38,
            .levels = {
                .reset = 0,
                .interrupt = 0,
            },
            .flags = {
                .swap_xy = 0,
                .mirror_x = 0,
                .mirror_y = 0,
            },
        };
        esp_lcd_panel_io_handle_t tp_io_handle = NULL;
        esp_lcd_panel_io_i2c_config_t tp_io_config = {
            .dev_addr = ESP_LCD_TOUCH_IO_I2C_FT5x06_ADDRESS,
            .control_phase_bytes = 1,
            .dc_bit_offset = 0,
            .lcd_cmd_bits = 8,
            .flags =
            {
                .disable_control_phase = 1,
            }
        };
        tp_io_config.scl_speed_hz = 400*  1000;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_, &tp_io_config, &tp_io_handle));
        ESP_LOGI(TAG, "Initialize touch controller");
        ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &tp));
        const lvgl_port_touch_cfg_t touch_cfg = {
            .disp = lv_display_get_default(),
            .handle = tp,
        };
        lv_indev_t* touch_indev = lvgl_port_add_touch(&touch_cfg);
        if (display_ && touch_indev) {
            display_->AttachTouchIndev(touch_indev);
        }
        ESP_LOGI(TAG, "Touch panel initialized successfully");
    }

    // 初始化工具
    void InitializeTools() {
        auto &mcp_server = McpServer::GetInstance();
        mcp_server.AddTool("self.system.reconfigure_wifi",
            "End this conversation and enter WiFi configuration mode.\n"
            "**CAUTION** You must ask the user to confirm this action.",
            PropertyList(), [this](const PropertyList& properties) {
                EnterWifiConfigMode();
                return true;
            });
    }

public:
    WaveshareEsp32s3TouchAMOLED2inch06() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializePowerSaveTimer();
        InitializeCodecI2c();
        InitializeAxp2101();
        InitializeSpi();
        InitializeSH8601Display();
        InitializeTouch();
        InitializeButtons();
        InitializeTools();
    }

    virtual AudioCodec* GetAudioCodec() override {
        static BoxAudioCodec audio_codec(
            i2c_bus_, 
            AUDIO_INPUT_SAMPLE_RATE, 
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, 
            AUDIO_I2S_GPIO_BCLK, 
            AUDIO_I2S_GPIO_WS, 
            AUDIO_I2S_GPIO_DOUT, 
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, 
            AUDIO_CODEC_ES8311_ADDR, 
            AUDIO_CODEC_ES7210_ADDR, 
            AUDIO_INPUT_REFERENCE);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        return backlight_;
    }

    virtual bool GetBatteryLevel(int &level, bool &charging, bool &discharging) override {
        static bool last_discharging = false;
        charging = pmic_->IsCharging();
        discharging = pmic_->IsDischarging();
        if (discharging != last_discharging)
        {
            power_save_timer_->SetEnabled(discharging);
            last_discharging = discharging;
        }

        level = pmic_->GetBatteryLevel();
        return true;
    }

    virtual void SetPowerSaveLevel(PowerSaveLevel level) override {
        if (level != PowerSaveLevel::LOW_POWER) {
            power_save_timer_->WakeUp();
        }
        WifiBoard::SetPowerSaveLevel(level);
    }
};

DECLARE_BOARD(WaveshareEsp32s3TouchAMOLED2inch06);
