#include "quiet_cool.h"
#include "esphome/core/log.h"
#include "quietcool.h"

namespace esphome {
    namespace quiet_cool {
        
        static const char *TAG = "quiet_cool.fan";

        void QuietCoolFan::setup() {
            ESP_LOGI(TAG, "=== QuietCoolFan::setup() called ===");
            ESP_LOGI(TAG, "pins_set_ = %s", this->pins_set_ ? "true" : "false");
            ESP_LOGI(TAG, "csn_pin_ = %d, gdo0_pin_ = %d, gdo2_pin_ = %d", this->csn_pin_, this->gdo0_pin_, this->gdo2_pin_);

            if (!this->pins_set_) {
                ESP_LOGE(TAG, "QuietCool pins not configured via YAML; radio not initialised");
                return;
            }

            if (this->qc_ == nullptr) {
                ESP_LOGI(TAG, "Creating QuietCool object...");
                this->qc_.reset(new QuietCool(this->csn_pin_, this->gdo0_pin_, this->gdo2_pin_, 18, 19, 23, remote_id_.data(), center_freq_mhz, deviation_khz));
                ESP_LOGI(TAG, "QuietCool object created");
            }

            ESP_LOGI(TAG, "Calling qc_->begin()...");
            this->qc_->begin();
            ESP_LOGI(TAG, "qc_->begin() completed");
            ESP_LOGD(TAG, "QuietCool initialized");
        }

        void QuietCoolFan::reinit_radio() {
            ESP_LOGI(TAG, "=== MANUAL RE-INITIALIZATION TRIGGERED ===");
            if (this->qc_) {
                ESP_LOGI(TAG, "Calling qc_->begin() again...");
                this->qc_->begin();
                ESP_LOGI(TAG, "Re-initialization complete");
            } else {
                ESP_LOGE(TAG, "Cannot reinit - qc_ is null!");
            }
        }

        void QuietCoolFan::scan_frequencies() {
            ESP_LOGI(TAG, "=== FREQUENCY SCAN STARTED ===");
            if (!this->qc_) {
                ESP_LOGE(TAG, "Cannot scan - qc_ is null!");
                return;
            }

            // Scan from 433.85 to 433.95 MHz in 5 kHz steps
            float original_freq = this->center_freq_mhz;
            ESP_LOGI(TAG, "Original frequency: %.3f MHz", original_freq);

            for (float freq = 433.850; freq <= 433.950; freq += 0.005) {
                ESP_LOGI(TAG, "Testing frequency: %.3f MHz", freq);

                // Update frequency in QuietCool object and reinit
                this->qc_->set_frequency(freq);
                this->qc_->begin();

                // Send LOW speed, ON command
                this->qc_->send(QUIETCOOL_SPEED_LOW, QUIETCOOL_DURATION_ON);

                // Wait between transmissions
                delay(2000);
            }

            // Restore original frequency
            this->qc_->set_frequency(original_freq);
            this->qc_->begin();
            ESP_LOGI(TAG, "=== FREQUENCY SCAN COMPLETE ===");
            ESP_LOGI(TAG, "Restored original frequency: %.3f MHz", original_freq);
        }

        fan::FanTraits QuietCoolFan::get_traits() {
            return fan::FanTraits(false, true, false, this->speed_count_);
        }

        void QuietCoolFan::control(const fan::FanCall &call) {
            ESP_LOGD(TAG, "Control called: state=%s, speed=%s",
                     call.get_state().has_value() ? (*call.get_state() ? "ON" : "OFF") : "<unchanged>",
                     call.get_speed().has_value() ? (std::to_string(*call.get_speed())).c_str() : "<unchanged>");

            // Store old state for logging
            bool old_state = this->state;
            int old_speed = this->speed;

            // Handle state changes
            if (call.get_state().has_value()) {
                bool new_state = *call.get_state();

                if (new_state) {
                    // Turning ON
                    if (call.get_speed().has_value()) {
                        // Speed explicitly specified, use it
                        this->speed = *call.get_speed();
                    } else if (this->speed == 0 || !old_state) {
                        // No speed specified and either:
                        // - Current speed is 0, or
                        // - We were previously OFF
                        // Default to speed 1 (LOW)
                        this->speed = 1;
                        ESP_LOGD(TAG, "No speed specified, defaulting to speed 1");
                    }
                    // else: keep existing speed from previous ON state
                    this->state = true;
                } else {
                    // Turning OFF
                    this->state = false;
                    this->speed = 0;
                }
            } else if (call.get_speed().has_value()) {
                // Only speed changed, not state
                int new_speed = *call.get_speed();
                this->speed = new_speed;

                if (new_speed == 0) {
                    // Speed 0 means turn OFF
                    this->state = false;
                } else {
                    // Non-zero speed means turn ON
                    this->state = true;
                }
            }

            // Now map internal state to hardware commands
            QuietCoolSpeed qcspd = QUIETCOOL_SPEED_LOW;
            QuietCoolDuration qcdur = QUIETCOOL_DURATION_ON;

            int current_speed = this->speed;

            if (!this->state || current_speed == 0) {
                // Fan is OFF
                qcdur = QUIETCOOL_DURATION_OFF;
                qcspd = QUIETCOOL_SPEED_LOW;  // Doesn't matter, but be explicit
            } else {
                // Fan is ON, map speed to hardware
                qcdur = QUIETCOOL_DURATION_ON;

                if (this->speed_count_ == 2) {
                    // 2-speed mode: speed 1 = LOW, speed 2 = HIGH
                    if (current_speed == 1) {
                        qcspd = QUIETCOOL_SPEED_LOW;
                    } else {
                        qcspd = QUIETCOOL_SPEED_HIGH;
                    }
                } else {
                    // 3-speed mode: speed 1 = LOW, speed 2 = MEDIUM, speed 3 = HIGH
                    if (current_speed == 1) {
                        qcspd = QUIETCOOL_SPEED_LOW;
                    } else if (current_speed == 2) {
                        qcspd = QUIETCOOL_SPEED_MEDIUM;
                    } else {
                        qcspd = QUIETCOOL_SPEED_HIGH;
                    }
                }
            }

            // Send command to hardware
            if (this->qc_) {
                ESP_LOGI(TAG, "Sending to hardware: speed=0x%02X, duration=0x%02X", qcspd, qcdur);
                this->qc_->send(qcspd, qcdur);
            }

            ESP_LOGI(TAG, "State updated: state=%s, speed=%d (was: state=%s, speed=%d)",
                     this->state ? "ON" : "OFF", current_speed,
                     old_state ? "ON" : "OFF", old_speed);

            // Publish state to Home Assistant
            // This will update both the on/off state and the percentage
            this->publish_state();
        }

        void QuietCoolFan::write_state_() {
            ESP_LOGVV(TAG, "write_state_: driving pins: state=%s ", 
                      (this->state ? "ON" : "OFF"));
            ESP_LOGVV(TAG, "write_state_: output calls completed");
        }

        void QuietCoolFan::dump_config() {
            LOG_FAN("", "QuietCool fan", this);
            ESP_LOGCONFIG(TAG, "  Setup called: %s", this->qc_ ? "YES" : "NO");
            ESP_LOGCONFIG(TAG, "  Pins set: %s", this->pins_set_ ? "YES" : "NO");
            ESP_LOGCONFIG(TAG, "  CS Pin: %d", this->csn_pin_);
            ESP_LOGCONFIG(TAG, "  GDO0 Pin: %d", this->gdo0_pin_);
            ESP_LOGCONFIG(TAG, "  GDO2 Pin: %d", this->gdo2_pin_);
            ESP_LOGCONFIG(TAG, "  Speed Count: %d", this->speed_count_);
        }
    }  // namespace quiet_cool
}  // namespace esphome
