#pragma once
#include <stdint.h>
#include <stddef.h>

/*
  CC1101 Pin Connections:
  hdr   cc1101
  1      19  GND
  2      18  VCC
  3      6   GDO0
  4      7   CSn
  5      1   SCK
  6      20  MOSI
  7      2   MISO
  8      3   GDO2
*/

namespace esphome
{
  namespace quiet_cool
  {

    struct RxCommand
    {
      bool valid = false;
      uint8_t speed = 0;    // 0x90=LOW, 0xA0=MED, 0xB0=HIGH
      uint8_t duration = 0; // 0x0F=ON, 0x00=OFF, etc.
      bool is_wake = false;
      bool is_off = false;
    };

    enum QuietCoolSpeed
    {
      QUIETCOOL_SPEED_HIGH = 0xB0,
      QUIETCOOL_SPEED_MEDIUM = 0xA0,
      QUIETCOOL_SPEED_LOW = 0x90,
      QUIETCOOL_SPEED_LAST
    };

    enum QuietCoolDuration
    {
      QUIETCOOL_DURATION_1H = 0x01,
      QUIETCOOL_DURATION_2H = 0x02,
      QUIETCOOL_DURATION_4H = 0x04,
      QUIETCOOL_DURATION_8H = 0x08,
      QUIETCOOL_DURATION_12H = 0x0C,
      QUIETCOOL_DURATION_ON = 0x0F,
      QUIETCOOL_DURATION_OFF = 0x00,
      QUIETCOOL_DURATION_LAST
    };

    class QuietCool
    {
    private:
      static constexpr uint8_t TO_BIT(char c) { return (c == '1') ? 1 : 0; }

      uint8_t csn_pin;
      uint8_t gdo0_pin;
      uint8_t gdo2_pin; // allow -1 for invalid
      uint8_t sck_pin;
      uint8_t miso_pin;
      uint8_t mosi_pin;
      uint8_t remote_id[7];
      float center_freq_mhz;
      float deviation_khz;

      bool initCC1101();
      uint8_t readChipVersion();
      void processBitsFromBytes(const uint8_t *bytes, size_t byte_len, bool send_to_pin);
      void sendRawData(const uint8_t *data, size_t len);
      void sendPacket(const uint8_t cmd_code);
      const uint8_t getCommand(QuietCoolSpeed speed, QuietCoolDuration duration);
      void logBits(const uint8_t *data, size_t len);
      RxCommand last_rx_cmd_;

    public:
      QuietCool(uint8_t csn, uint8_t gdo0, uint8_t gdo2, uint8_t sck, uint8_t miso, uint8_t mosi, const uint8_t *remote_id_in, float freq_mhz, float deviation_khz);
      void begin();
      void send(QuietCoolSpeed speed, QuietCoolDuration duration);
      void sendWake();                    // Send WAKE (0x66) only — for state query testing
      void set_frequency(float freq_mhz); // Update center frequency

      // Helper methods for loop() monitoring
      uint8_t getMarcState();                                                 // Get current CC1101 state
      uint8_t getRxBytes();                                                   // Get number of bytes in RX FIFO
      void calibrate();                                                       // Perform frequency calibration
      void forceRxMode();                                                     // Force CC1101 back to RX mode
      bool readRxData(uint8_t *buffer, uint8_t max_len, uint8_t *bytes_read); // Read RX FIFO data

      // Packet processing methods (hardware-correct approach)
      uint8_t readRxByte();                                                                // Read single byte from RX FIFO
      void readRxBurst(uint8_t *buffer, uint8_t len);                                      // Burst read from RX FIFO
      void flushRxFifo();                                                                  // Flush RX FIFO (with proper IDLE transition)
      void quickResetRx();                                                                 // Fast SFRX+SRX after packet read (RXOFF_MODE=00)
      void enterRxMode();                                                                  // Enter RX mode (wrapper for SetRx)
      void recoverFromFifoError();                                                         // Recover from overflow/underflow error states
      void processPacket(const uint8_t *packet, size_t packet_len, uint32_t timestamp_ms); // Process complete packet
      RxCommand consumeRxCommand();                                                        // Returns last decoded RX command and clears it
    };

  } // namespace quiet_cool
} // namespace esphome
