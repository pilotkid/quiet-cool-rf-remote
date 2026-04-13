#include "quiet_cool.h"
#include "esphome/core/log.h"
#include "quietcool.h"
#include <string>
#include <cstring>                    // for memmove, memcpy
#include <Arduino.h>                  // for digitalRead
#include "ELECHOUSE_CC1101_SRC_DRV.h" // for register readback

namespace esphome
{
    namespace quiet_cool
    {

        static const char *TAG = "quiet_cool.fan";

        void QuietCoolFan::setup()
        {
            ESP_LOGD(TAG, "setup: pins_set=%s csn=%d gdo0=%d gdo2=%d",
                     this->pins_set_ ? "true" : "false", this->csn_pin_, this->gdo0_pin_, this->gdo2_pin_);

            if (!this->pins_set_)
            {
                ESP_LOGE(TAG, "QuietCool pins not configured via YAML; radio not initialised");
                return;
            }

            if (this->qc_ == nullptr)
            {
                this->qc_.reset(new QuietCool(this->csn_pin_, this->gdo0_pin_, this->gdo2_pin_, 18, 19, 23, remote_id_.data(), center_freq_mhz, deviation_khz));
            }

            this->qc_->begin();
            ESP_LOGI(TAG, "QuietCool initialized");
        }

        void QuietCoolFan::reinit_radio()
        {
            if (this->qc_)
            {
                ESP_LOGI(TAG, "Re-initializing radio");
                this->qc_->begin();
            }
        }

        void QuietCoolFan::scan_frequencies()
        {
            ESP_LOGI(TAG, "=== FREQUENCY SCAN STARTED ===");
            if (!this->qc_)
            {
                ESP_LOGE(TAG, "Cannot scan - qc_ is null!");
                return;
            }

            // Scan from 433.85 to 433.95 MHz in 5 kHz steps
            float original_freq = this->center_freq_mhz;
            ESP_LOGI(TAG, "Original frequency: %.3f MHz", original_freq);

            for (float freq = 433.850; freq <= 433.950; freq += 0.005)
            {
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

        fan::FanTraits QuietCoolFan::get_traits()
        {
            return fan::FanTraits(false, true, false, this->speed_count_);
        }

        void QuietCoolFan::loop()
        {
            uint32_t now = millis();

            if (!this->qc_)
            {
                return;
            }

            // Periodic calibration for long-term stability (every 5 minutes)
            static uint32_t last_cal = 0;
            if (now - last_cal > 300000)
            {
                this->qc_->calibrate();
                last_cal = now;
            }

            // Periodic MARCSTATE monitoring (every 10 seconds) — safety net
            static uint32_t last_log = 0;
            static bool regs_logged = false;
            if (now - last_log > 10000)
            {
                uint8_t marcstate = this->qc_->getMarcState();
                ESP_LOGD(TAG, "CC1101 MARCSTATE: 0x%02X (0x0D = RX)", marcstate);
                ESP_LOGD(TAG, "RX stats: pkts=%lu, overflows=%lu, gdo0_blocked=%lu",
                         rx_packet_count_, overflow_count_, gdo0_blocked_count_);

                // One-time register readback (setup logs lost before API connects)
                if (!regs_logged)
                {
                    regs_logged = true;
                    uint8_t mdmcfg2 = ELECHOUSE_cc1101.SpiReadReg(0x12);
                    uint8_t pktctrl1 = ELECHOUSE_cc1101.SpiReadReg(0x07);
                    uint8_t foccfg = ELECHOUSE_cc1101.SpiReadReg(0x19);
                    uint8_t bscfg = ELECHOUSE_cc1101.SpiReadReg(0x1A);
                    ESP_LOGI(TAG, "Regs: MDMCFG2=0x%02X(SYNC_MODE=%d) PKTCTRL1=0x%02X(PQT=%d) FOCCFG=0x%02X BSCFG=0x%02X",
                             mdmcfg2, mdmcfg2 & 0x07, pktctrl1, (pktctrl1 >> 5) & 0x07, foccfg, bscfg);
                }

                if (marcstate == 0x11 || marcstate == 0x16)
                {
                    ESP_LOGW(TAG, "CC1101 FIFO error (state 0x%02X) - recovering", marcstate);
                    this->qc_->recoverFromFifoError();
                }
                else if (marcstate != 0x0D && marcstate != 0x1F)
                {
                    ESP_LOGW(TAG, "CC1101 not in RX (state 0x%02X) - forcing RX", marcstate);
                    this->qc_->forceRxMode();
                }

                last_log = now;
            }

            // Periodic WAKE poll (every 30s) — query fan state
            static uint32_t last_wake = 0;
            if (now - last_wake > 30000)
            {
                ESP_LOGD(TAG, "Periodic WAKE poll");
                this->qc_->sendWake();
                last_wake = millis(); // Use millis() after sendWake() completes (~500ms)
            }

            // --- Drain all complete packets from FIFO ---
            // Dual-condition read: satisfies TI FIFO errata (SWRZ020) while allowing
            // aggressive draining during burst reception (3 packets in ~250ms).
            uint8_t packets_this_loop = 0;
            while (true)
            {
                // Double-read RXBYTES per TI recommendation (use lower value)
                uint8_t rb1 = this->qc_->getRxBytes();
                uint8_t rb2 = this->qc_->getRxBytes();
                uint8_t rxbytes_raw = (rb1 < rb2) ? rb1 : rb2;
                bool overflow = rxbytes_raw & 0x80;
                uint8_t rxbytes = rxbytes_raw & 0x7F;

                if (overflow)
                {
                    ESP_LOGW(TAG, "RX FIFO overflow - recovering");
                    this->qc_->recoverFromFifoError();
                    overflow_count_++;
                    break;
                }

                // Dual-condition: safe to read when either:
                // (a) 2+ packets buffered — reading 22 leaves ≥22, so SPI read pointer
                //     never catches radio write pointer (satisfies TI errata workaround)
                // (b) 1 packet complete and no active reception (GDO0 LOW)
                bool can_read = (rxbytes >= 44) ||
                                (rxbytes >= 22 && digitalRead(this->gdo0_pin_) == LOW);

                if (!can_read)
                {
                    if (rxbytes >= 22)
                        gdo0_blocked_count_++; // Diagnostic
                    break;
                }

                uint8_t buffer[22];
                this->qc_->readRxBurst(buffer, 22);

                // Publish RX packet to optional text sensor as three columns: ID  CMD  DATA
                if (this->rx_packet_text_sensor_ != nullptr)
                {
                    // Column 1: 7-byte ID as contiguous hex (no spaces)
                    char id_cont[7 * 2 + 1];
                    id_cont[0] = '\0';
                    for (int i = 0; i < 7; ++i)
                    {
                        snprintf(id_cont + i * 2, 3, "%02X", buffer[7 + i]);
                    }

                    // Column 2: command bytes (14,15)
                    uint8_t cmd1 = (20 >= 15) ? buffer[14] : 0;
                    uint8_t cmd2 = (20 >= 16) ? buffer[15] : 0;
                    char cmd_col[12];
                    snprintf(cmd_col, sizeof(cmd_col), "0x%02X,0x%02X", cmd1, cmd2);

                    // Column 3: data bytes (16-19) as contiguous hex
                    char data_col[4 * 2 + 1];
                    data_col[0] = '\0';
                    for (int i = 0; i < 4; ++i)
                    {
                        int idx = 16 + i;
                        uint8_t v = (idx < 20) ? buffer[idx] : 0;
                        snprintf(data_col + i * 2, 3, "%02X", v);
                    }

                    char out[64];
                    snprintf(out, sizeof(out), "%s  %s  %s", id_cont, cmd_col, data_col);
                    this->rx_packet_text_sensor_->publish_state(std::string(out));
                }

                this->qc_->processPacket(buffer, 20, now);

                // RSSI/LQI from APPEND_STATUS bytes
                int8_t rssi_raw = (int8_t)buffer[20];
                int rssi_dbm = (rssi_raw >= 128) ? (rssi_raw - 256) / 2 - 74 : rssi_raw / 2 - 74;
                uint8_t lqi = buffer[21] & 0x7F;

                // Log with source indicator
                const char *source = (rssi_dbm > -70) ? "FAN" : "REMOTE";
                ESP_LOGI(TAG, "RX [%s] RSSI=%d LQI=%d", source, rssi_dbm, lqi);

                // Consume decoded command and sync HA state
                auto rx_cmd = this->qc_->consumeRxCommand();
                if (rx_cmd.valid && !rx_cmd.is_wake)
                {
                    // Deduplication: ignore same command within 1 second
                    static uint8_t last_rx_speed = 0;
                    static uint8_t last_rx_duration = 0;
                    static bool last_rx_was_off = false;
                    static uint32_t last_rx_time = 0;

                    bool is_duplicate = (now - last_rx_time < 1000) &&
                                        (rx_cmd.is_off == last_rx_was_off) &&
                                        (rx_cmd.speed == last_rx_speed) &&
                                        (rx_cmd.duration == last_rx_duration);

                    last_rx_speed = rx_cmd.speed;
                    last_rx_duration = rx_cmd.duration;
                    last_rx_was_off = rx_cmd.is_off;
                    last_rx_time = now;

                    if (!is_duplicate)
                    {
                        if (rx_cmd.is_off)
                        {
                            this->state = false;
                            this->speed = 0;
                            ESP_LOGI(TAG, "RX sync: OFF");
                        }
                        else
                        {
                            this->state = true;
                            if (this->speed_count_ == 2)
                            {
                                if (rx_cmd.speed == 0x90)
                                    this->speed = 1;
                                else
                                    this->speed = 2;
                            }
                            else
                            {
                                if (rx_cmd.speed == 0x90)
                                    this->speed = 1;
                                else if (rx_cmd.speed == 0xA0)
                                    this->speed = 2;
                                else
                                    this->speed = 3;
                            }
                            ESP_LOGI(TAG, "RX sync: ON speed=%d", this->speed);
                        }
                        this->publish_state();
                    }
                }

                packets_this_loop++;
                rx_packet_count_++;
                if (packets_this_loop >= 5)
                    break; // Safety limit per loop iteration
            }

            // FIFO alignment recovery: residual bytes indicate misalignment
            if (packets_this_loop > 0)
            {
                uint8_t residual = this->qc_->getRxBytes() & 0x7F;
                if (residual > 0 && residual < 22)
                {
                    ESP_LOGW(TAG, "FIFO misaligned (%d residual bytes) - flushing", residual);
                    this->qc_->recoverFromFifoError();
                }
            }
        }

        void QuietCoolFan::send_wake()
        {
            if (this->qc_)
            {
                this->qc_->sendWake();
            }
        }

        void QuietCoolFan::control(const fan::FanCall &call)
        {
            ESP_LOGD(TAG, "Control called: state=%s, speed=%s",
                     call.get_state().has_value() ? (*call.get_state() ? "ON" : "OFF") : "<unchanged>",
                     call.get_speed().has_value() ? (std::to_string(*call.get_speed())).c_str() : "<unchanged>");

            // Store old state for logging
            bool old_state = this->state;
            int old_speed = this->speed;

            // Handle state changes
            if (call.get_state().has_value())
            {
                bool new_state = *call.get_state();

                if (new_state)
                {
                    // Turning ON
                    if (call.get_speed().has_value())
                    {
                        // Speed explicitly specified, use it
                        this->speed = *call.get_speed();
                    }
                    else if (this->speed == 0 || !old_state)
                    {
                        // No speed specified and either:
                        // - Current speed is 0, or
                        // - We were previously OFF
                        // Default to speed 1 (LOW)
                        this->speed = 1;
                        ESP_LOGD(TAG, "No speed specified, defaulting to speed 1");
                    }
                    // else: keep existing speed from previous ON state
                    this->state = true;
                }
                else
                {
                    // Turning OFF
                    this->state = false;
                    this->speed = 0;
                }
            }
            else if (call.get_speed().has_value())
            {
                // Only speed changed, not state
                int new_speed = *call.get_speed();
                this->speed = new_speed;

                if (new_speed == 0)
                {
                    // Speed 0 means turn OFF
                    this->state = false;
                }
                else
                {
                    // Non-zero speed means turn ON
                    this->state = true;
                }
            }

            // Now map internal state to hardware commands
            QuietCoolSpeed qcspd = QUIETCOOL_SPEED_LOW;
            QuietCoolDuration qcdur = QUIETCOOL_DURATION_ON;

            int current_speed = this->speed;

            if (!this->state || current_speed == 0)
            {
                // Fan is OFF
                qcdur = QUIETCOOL_DURATION_OFF;
                qcspd = QUIETCOOL_SPEED_LOW; // Doesn't matter, but be explicit
            }
            else
            {
                // Fan is ON, map speed to hardware
                qcdur = QUIETCOOL_DURATION_ON;

                if (this->speed_count_ == 2)
                {
                    // 2-speed mode: speed 1 = LOW, speed 2 = HIGH
                    if (current_speed == 1)
                    {
                        qcspd = QUIETCOOL_SPEED_LOW;
                    }
                    else
                    {
                        qcspd = QUIETCOOL_SPEED_HIGH;
                    }
                }
                else
                {
                    // 3-speed mode: speed 1 = LOW, speed 2 = MEDIUM, speed 3 = HIGH
                    if (current_speed == 1)
                    {
                        qcspd = QUIETCOOL_SPEED_LOW;
                    }
                    else if (current_speed == 2)
                    {
                        qcspd = QUIETCOOL_SPEED_MEDIUM;
                    }
                    else
                    {
                        qcspd = QUIETCOOL_SPEED_HIGH;
                    }
                }
            }

            // Send command to hardware
            if (this->qc_)
            {
                ESP_LOGI(TAG, "Sending to hardware: speed=0x%02X, duration=0x%02X", qcspd, qcdur);
                this->qc_->send(qcspd, qcdur);
                ESP_LOGI(TAG, "TX complete");
            }

            ESP_LOGI(TAG, "State updated: state=%s, speed=%d (was: state=%s, speed=%d)",
                     this->state ? "ON" : "OFF", current_speed,
                     old_state ? "ON" : "OFF", old_speed);

            // Publish state to Home Assistant
            // This will update both the on/off state and the percentage
            this->publish_state();
        }

        void QuietCoolFan::write_state_()
        {
            ESP_LOGVV(TAG, "write_state_: driving pins: state=%s ",
                      (this->state ? "ON" : "OFF"));
            ESP_LOGVV(TAG, "write_state_: output calls completed");
        }

        void QuietCoolFan::dump_config()
        {
            LOG_FAN("", "QuietCool fan", this);
            ESP_LOGCONFIG(TAG, "  Setup called: %s", this->qc_ ? "YES" : "NO");
            ESP_LOGCONFIG(TAG, "  Pins set: %s", this->pins_set_ ? "YES" : "NO");
            ESP_LOGCONFIG(TAG, "  CS Pin: %d", this->csn_pin_);
            ESP_LOGCONFIG(TAG, "  GDO0 Pin: %d", this->gdo0_pin_);
            ESP_LOGCONFIG(TAG, "  GDO2 Pin: %d", this->gdo2_pin_);
            ESP_LOGCONFIG(TAG, "  Speed Count: %d", this->speed_count_);
        }
    } // namespace quiet_cool
} // namespace esphome
