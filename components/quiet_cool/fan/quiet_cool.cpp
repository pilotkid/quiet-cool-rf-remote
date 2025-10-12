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
            float inc_speed = call.get_speed().value_or(-1.0f);
            ESP_LOGD(TAG, "Control called: state=%s, speed=%s", 
                     call.get_state().has_value() ? (*call.get_state() ? "ON" : "OFF") : "<unchanged>",
                     call.get_speed().has_value() ? (std::to_string(inc_speed)).c_str() : "<unchanged>");
            bool old_state = this->state;
            if (call.get_state().has_value())
                this->state = *call.get_state();

            QuietCoolSpeed qcspd = QUIETCOOL_SPEED_LOW;
            QuietCoolDuration qcdur = QUIETCOOL_DURATION_ON;
            if (call.get_speed().has_value()) {
                this->speed_ = *call.get_speed();
                if (this->speed_ < 0.5) {
                    qcdur = QUIETCOOL_DURATION_OFF;
                } else if (this->speed_count_ == 2) {
                    // 2-speed mode: speed 1 = LOW, speed 2 = HIGH
                    if (this->speed_ < 1.5) qcspd = QUIETCOOL_SPEED_LOW;
                    else qcspd = QUIETCOOL_SPEED_HIGH;
                } else {
                    // 3-speed mode: speed 1 = LOW, speed 2 = MEDIUM, speed 3 = HIGH
                    if (this->speed_ < 1.5) qcspd = QUIETCOOL_SPEED_LOW;
                    else if (this->speed_ < 2.5) qcspd = QUIETCOOL_SPEED_MEDIUM;
                    else qcspd = QUIETCOOL_SPEED_HIGH;
                }
            } else {
		qcdur = QUIETCOOL_DURATION_OFF;
	    }
            if (this->qc_) this->qc_->send(qcspd, qcdur);


            ESP_LOGV(TAG, "Post-update internal state: state=%s speed=%s", 
                     (this->state ? "ON" : "OFF"),
                     (std::to_string(this->speed_)).c_str());

            this->write_state_();
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
