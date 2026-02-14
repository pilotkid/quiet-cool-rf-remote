#include <stdint.h>
typedef uint8_t byte;
#include "quietcool.h"
#include "esphome/core/log.h"
#include "ELECHOUSE_CC1101_SRC_DRV.h"
#include <cstring>
#include <Arduino.h>  // for digitalRead in sendRawData

namespace esphome {
namespace quiet_cool {

static const char *TAG = "quietcool";

// CC1101 hardware prepends sync word (0x15 0xAA) automatically in TX mode.
// Data starts after the hardware sync — 7x AA preamble fill, then ID + CMD.
const uint8_t SYNC[] = {0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa};
#define SYNC_LEN (sizeof(SYNC))
#define REMOTE_ID_LEN (sizeof(REMOTE_ID))

#define CMD_CODE_LEN 2

// Commands are structured like this:
// LOW: 0x90
// MED: 0xA0    
// HIGH:0xB0
//
//  1 hour: 0x01
//  2 hour: 0x02
//  4 hour: 0x04
//  8 hour: 0x08
// 12 hour: 0x0C
//      on: 0x0F
//     off: 0x00
//
// the special command that's sent before anything else is 0x66.

// So, final packet format is:
//     <---- SYNC -------><--- ID -----><CD>
// 0x: 150aaaaaaaaaaaaaaaaTTUUVVWWXXYYZZGGGG
// sync: always the same
//   ID: unique identifier
//   CD: command, as described above.  Two bytes duplicated.

// Helper: Convert bytes to a bit string (MSB first)
static void bytesToBitString(const uint8_t* data, size_t len, char* bitstr, size_t maxlen) {
    size_t idx = 0;
    for (size_t i = 0; i < len && idx + 8 < maxlen; i++) {
        for (int b = 7; b >= 0; b--) {
            bitstr[idx++] = ((data[i] >> b) & 1) ? '1' : '0';
        }
    }
    bitstr[idx] = '\0';
}

// Helper: Log the bits in a byte array (MSB first)
void QuietCool::logBits(const uint8_t* data, size_t len) {
    char bitstr[8 * 32 + 1]; // up to 32 bytes (256 bits)
    bytesToBitString(data, len, bitstr, sizeof(bitstr));
    
    // Print bytes in hex format
    ESP_LOGD(TAG, "Bits sent: %s", bitstr);

    bitstr[0] = 0;
    for (size_t i = 0; i < len; i++) {
	char bb[3];
	snprintf(bb, 3, "%02X", data[i]);
        strcat(bitstr, bb);
    }
    ESP_LOGD(TAG, "Bytes sent: %s", bitstr);
}

void QuietCool::sendRawData(const uint8_t* data, size_t len) {
    if (len == 0) {
        ESP_LOGE(TAG, "No data to send");
        return;
    }
    ESP_LOGD(TAG, "Sending %zu bytes (%zu bits)", len, len * 8);
    logBits(data, len);

    // Write directly to TX FIFO — bypass ELECHOUSE SendData() which
    // prepends a length byte, corrupting fixed-length mode packets
    // by shifting all data 1 byte forward.
    ELECHOUSE_cc1101.SpiWriteBurstReg(0x3F, (byte*)data, (byte)len);
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    ELECHOUSE_cc1101.SpiStrobe(0x35);  // STX — start transmit
    while (!digitalRead(gdo0_pin));     // Wait for sync transmitted
    while (digitalRead(gdo0_pin));      // Wait for end of packet
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX — flush TX FIFO

    delay(10);
}

void QuietCool::sendPacket(const uint8_t cmd_code) {
    const uint8_t padding_len = 4;
    uint8_t full_cmd[SYNC_LEN + 7 + CMD_CODE_LEN+padding_len];
    memset(full_cmd, 0, sizeof(full_cmd));
    memcpy(full_cmd, SYNC, SYNC_LEN);
    memcpy(full_cmd + SYNC_LEN, remote_id, 7);
    memcpy(full_cmd + SYNC_LEN + 7, &cmd_code, 1);
    memcpy(full_cmd + SYNC_LEN + 8, &cmd_code, 1);
    size_t total_len = SYNC_LEN + 7 + CMD_CODE_LEN+padding_len;
    for (int i = 0; i < 3; i++) {
        sendRawData(full_cmd, total_len);
        delay(18);
    }
}

const uint8_t QuietCool::getCommand(QuietCoolSpeed speed, QuietCoolDuration duration) {
    ESP_LOGD(TAG, "getCommand got: speed=0x%02x, duration=0x%02x", speed, duration);

    // OFF is a special command (0x80), not speed|duration
    if (duration == QUIETCOOL_DURATION_OFF) {
        ESP_LOGD(TAG, "OFF command: 0x80");
        return 0x80;
    }

    const uint8_t off = 0x80;
    switch (speed) {
    case QUIETCOOL_SPEED_HIGH:
    case QUIETCOOL_SPEED_MEDIUM:
    case QUIETCOOL_SPEED_LOW:
    break;
    default:
	ESP_LOGD(TAG, "unknown speed: 0x%02x", speed);
	return off;
    };

    switch (duration) {
    case QUIETCOOL_DURATION_1H  :
    case QUIETCOOL_DURATION_2H  :
    case QUIETCOOL_DURATION_4H  :
    case QUIETCOOL_DURATION_8H  :
    case QUIETCOOL_DURATION_12H :
    case QUIETCOOL_DURATION_ON  :
    case QUIETCOOL_DURATION_OFF :
    break;
    default:
	ESP_LOGD(TAG, "unknown duration: 0x%02x", duration);
	return off;
    }
    uint8_t result = static_cast<uint8_t>(speed) | static_cast<uint8_t>(duration);
    ESP_LOGD(TAG, "Sending speed=0x%02x, duration=0x%02x: 0x%02x", speed, duration, result);
    return result;
}

    QuietCool::QuietCool(uint8_t csn, uint8_t gdo0, uint8_t gdo2, uint8_t sck, uint8_t miso, uint8_t mosi, const uint8_t* remote_id_in, float center_freq, float deviation_khz) : 
    csn_pin(csn),
    gdo0_pin(gdo0),
    gdo2_pin(gdo2),
    sck_pin(sck),
    miso_pin(miso),
    mosi_pin(mosi),
    center_freq_mhz(center_freq),
    deviation_khz(deviation_khz)
{
    for (int i = 0; i < 7; ++i) remote_id[i] = remote_id_in[i];
}

// --- Initialize CC1101 and verify communication ---
bool QuietCool::initCC1101() {
    ESP_LOGD(TAG, "sck:%d, miso:%d, mosi:%d, csn:%d, gdo0:%d\n", sck_pin, miso_pin, mosi_pin, csn_pin, gdo0_pin);
    ELECHOUSE_cc1101.setSpiPin(sck_pin, miso_pin, mosi_pin, csn_pin);

    int tries = 10;
    bool detected = false;
    while (tries--) {
        uint8_t version = readChipVersion();
        ESP_LOGI(TAG, "CC1101 VERSION READ: 0x%02X", version);
        if (version == 0x14 || version == 0x04) {
            ESP_LOGI(TAG, "CC1101 detected!");
            detected = true;
            break;
        }
    }

    if (!detected) {
        ESP_LOGE(TAG, "CC1101 not detected!");
        return false;
    }

    if (!ELECHOUSE_cc1101.getCC1101()) {
        ESP_LOGE(TAG, "CC1101 connection error");
        return false;
    }

    // ✅ Must set CC mode BEFORE Init so registers are configured correctly
    ELECHOUSE_cc1101.setCCMode(1);
    ELECHOUSE_cc1101.Init();

    ESP_LOGE(TAG, "Setting GDO0 pin to %d", gdo0_pin);
    ELECHOUSE_cc1101.setGDO0(gdo0_pin);

    // Basic configuration
    ESP_LOGE(TAG, "Setting center frequency to %f MHz", center_freq_mhz);
    ELECHOUSE_cc1101.setMHZ(center_freq_mhz);
    ELECHOUSE_cc1101.setPA(10);  // 10 dBm (10 mW) — match physical remote TX power

    // Packet-related configuration
    ELECHOUSE_cc1101.setModulation(0);       // FSK
    ESP_LOGI(TAG, "Setting deviaion to %f kHz.  That's a total spread of %f kHz", deviation_khz, 2*deviation_khz);
    ELECHOUSE_cc1101.setDeviation(deviation_khz);
    ELECHOUSE_cc1101.setDRate(2.398);

    // HYBRID APPROACH: Sync word filtering + sliding buffer for robustness
    // Enable sync word detection to filter out noise
    ELECHOUSE_cc1101.setSyncMode(1);  // 1 = 15/16 sync word bits (tolerates 1-bit error)
    ELECHOUSE_cc1101.setSyncWord(0x15, 0xAA);  // Match first 2 bytes of sync pattern
    ELECHOUSE_cc1101.setPQT(0);  // No preamble quality requirement (remote has no standard preamble)
    ESP_LOGI(TAG, "Sync word 0x15AA, 15/16 bit match, PQT=0 (optimized for burst reception)");

    ELECHOUSE_cc1101.setWhiteData(false);
    ELECHOUSE_cc1101.setManchester(false);
    ELECHOUSE_cc1101.setPktFormat(0);

    // Disable CRC - the remote doesn't use CRC (just duplicated command bytes for validation)
    ELECHOUSE_cc1101.setCrc(0);  // Disable CRC checking
    ESP_LOGI(TAG, "Disabled CRC (remote uses duplicated command bytes instead)");

    ELECHOUSE_cc1101.setLengthConfig(0);  // Fixed packet length
    ELECHOUSE_cc1101.setPacketLength(20);  // 20 bytes (matches TX format)
    ELECHOUSE_cc1101.setPRE(0);

    // RX-specific configuration for receiving physical remote commands
    ESP_LOGI(TAG, "Configuring RX parameters");

    // Set RX bandwidth to match signal characteristics (Carson's rule: ~25 kHz needed)
    ELECHOUSE_cc1101.setRxBW(58);  // ~58 kHz RX bandwidth (2-3x signal BW)
    ESP_LOGI(TAG, "Set RX bandwidth to 58 kHz");

    // Widen frequency offset compensation for better bit sync after inter-packet gaps
    // Default FOCCFG=0x16 has FOC_LIMIT=1 (narrow); 0x1D has FOC_LIMIT=3 (wide) + max post-sync gain
    ELECHOUSE_cc1101.SpiWriteReg(0x19, 0x1D);  // FOCCFG: wide AFC, aggressive post-sync correction
    ESP_LOGI(TAG, "Set FOCCFG=0x1D (wide AFC for burst reception)");

    // Fixed 20-byte packet length for clean boundaries
    ESP_LOGI(TAG, "Using fixed 20-byte packet length (filters noise, clean boundaries)");

    // Configure GDO0 for packet received (sync word detected)
    ELECHOUSE_cc1101.SpiWriteReg(0x02, 0x06);  // IOCFG0: GDO0 = sync word sent/received
    ESP_LOGI(TAG, "Configured GDO0 for packet RX complete (IOCFG0=0x06)");

    // Configure MCSM1: stay in RX after packet reception
    // MCSM1: bits 3-2 = RXOFF_MODE, 11 = stay in RX
    // Staying in RX allows continuous reception of burst packets (WAKE → CMD)
    // without losing sync between packets. FIFO is drained fast enough between
    // packets (~2ms read vs ~18ms inter-packet gap) to prevent overflow.
    uint8_t mcsm1 = ELECHOUSE_cc1101.SpiReadReg(0x17);  // Read current MCSM1
    mcsm1 = (mcsm1 & 0xF3) | 0x0C;  // Set RXOFF_MODE to 11 (stay in RX)
    ELECHOUSE_cc1101.SpiWriteReg(0x17, mcsm1);
    ESP_LOGI(TAG, "Configured MCSM1 to stay in RX after packet (MCSM1=0x%02X)", mcsm1);

    // Configure MCSM0 for automatic calibration on IDLE->RX/TX transitions
    // MCSM0: bits 5-4 = FS_AUTOCAL
    // 00 = Never, 01 = from IDLE to RX/TX, 10 = from RX/TX to IDLE, 11 = every 4th time
    uint8_t mcsm0 = ELECHOUSE_cc1101.SpiReadReg(0x18);  // Read current MCSM0
    mcsm0 = (mcsm0 & 0xCF) | 0x10;  // Set FS_AUTOCAL to 01 (calibrate on IDLE->RX/TX)
    ELECHOUSE_cc1101.SpiWriteReg(0x18, mcsm0);
    ESP_LOGI(TAG, "Configured MCSM0 for auto-calibration on IDLE->RX (MCSM0=0x%02X)", mcsm0);

    // Log key register values for validation
    uint8_t mdmcfg2 = ELECHOUSE_cc1101.SpiReadReg(0x12);
    uint8_t pktctrl1 = ELECHOUSE_cc1101.SpiReadReg(0x07);
    uint8_t sync1 = ELECHOUSE_cc1101.SpiReadReg(0x04);
    uint8_t sync0 = ELECHOUSE_cc1101.SpiReadReg(0x05);
    uint8_t bscfg = ELECHOUSE_cc1101.SpiReadReg(0x1A);
    uint8_t agcctrl2 = ELECHOUSE_cc1101.SpiReadReg(0x1B);
    uint8_t agcctrl1 = ELECHOUSE_cc1101.SpiReadReg(0x1C);
    uint8_t agcctrl0 = ELECHOUSE_cc1101.SpiReadReg(0x1D);
    ESP_LOGI(TAG, "Register readback: MDMCFG2=0x%02X (SYNC_MODE=%d), PKTCTRL1=0x%02X (PQT=%d)",
             mdmcfg2, mdmcfg2 & 0x07, pktctrl1, (pktctrl1 >> 5) & 0x07);
    ESP_LOGI(TAG, "  SYNC=0x%02X%02X, BSCFG=0x%02X, AGCCTRL=0x%02X/0x%02X/0x%02X",
             sync1, sync0, bscfg, agcctrl2, agcctrl1, agcctrl0);

    delay(500);
    return true;
}
uint8_t QuietCool::readChipVersion() {
    return ELECHOUSE_cc1101.SpiReadReg(0xF1);
}

void QuietCool::begin() {
    ESP_LOGI(TAG, "Starting CC1101 setup");
    if (!initCC1101()) {
        ESP_LOGE(TAG, "CC1101 not detected");
        return;
    }
    ESP_LOGI(TAG, "CC1101 initialized");

    // Ensure we're in IDLE after init
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delay(2);

    // Calibrate frequency synthesizer for stability
    ESP_LOGI(TAG, "Calibrating frequency synthesizer");
    ELECHOUSE_cc1101.SpiStrobe(0x33);  // SCAL
    delay(10);  // Calibration takes up to 712μs

    // Flush both FIFOs to start clean
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX
    delay(2);

    // Enter RX mode
    ESP_LOGI(TAG, "Entering RX mode");
    ELECHOUSE_cc1101.SetRx();
    delay(10);

    // Verify RX mode
    byte marcstate = ELECHOUSE_cc1101.SpiReadStatus(0xF5);
    ESP_LOGI(TAG, "Initial MARCSTATE = 0x%02X (0x0D = RX expected)", marcstate);

    if (marcstate == 0x0D) {
        ESP_LOGI(TAG, "CC1101 ready and listening");
    } else {
        ESP_LOGW(TAG, "CC1101 not in RX mode! State: 0x%02X", marcstate);
    }
}

void QuietCool::send(QuietCoolSpeed speed, QuietCoolDuration duration) {
    ESP_LOGI(TAG, "send(0x%02x, %0x%02x)", speed, duration);

    const uint8_t cmd_code = getCommand(speed, duration);
    ESP_LOGI(TAG, "Preparing to send cmd=0x%02x", cmd_code);

    // Proper sequence: Current state → SIDLE → TX → SIDLE → RX
    // Going to IDLE stops any RX activity, so we don't need to disable interrupts
    ESP_LOGD(TAG, "Entering IDLE mode");
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delay(2);  // Allow state transition

    // Flush TX FIFO while in IDLE
    ESP_LOGD(TAG, "Flushing TX FIFO");
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX
    delay(2);

    // Send packet (internally uses STX)
    sendPacket(cmd_code);

    // Wait for TX to complete
    delay(50);

    // Return to IDLE
    ESP_LOGD(TAG, "Returning to IDLE after TX");
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delay(2);

    // Flush RX FIFO before entering RX mode
    ESP_LOGD(TAG, "Flushing RX FIFO");
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX
    delay(2);

    // Enter RX mode
    ESP_LOGI(TAG, "Entering RX mode");
    ELECHOUSE_cc1101.SetRx();  // SRX
    delay(10);

    // Verify we're in RX mode
    byte marcstate = ELECHOUSE_cc1101.SpiReadStatus(0xF5);
    ESP_LOGI(TAG, "TX complete, MARCSTATE = 0x%02X (0x0D = RX expected)", marcstate);

    if (marcstate != 0x0D && marcstate != 0x1F) {
        ESP_LOGW(TAG, "Warning: Not in expected RX state after transmission!");
    }
}

void QuietCool::sendWake() {
    ESP_LOGI(TAG, "Sending WAKE (0x66) for state query");

    // Same TX sequence as send(): IDLE → flush → TX → IDLE → flush → RX
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delay(2);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX
    delay(2);

    sendPacket(0x66);
    delay(50);

    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delay(2);
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX
    delay(2);
    ELECHOUSE_cc1101.SetRx();
    delay(10);

    ESP_LOGI(TAG, "WAKE sent, now listening for fan response");
}

void QuietCool::set_frequency(float freq_mhz) {
    this->center_freq_mhz = freq_mhz;
    ESP_LOGI(TAG, "Frequency updated to %.3f MHz", freq_mhz);
}

// Helper methods for loop() monitoring
uint8_t QuietCool::getMarcState() {
    return ELECHOUSE_cc1101.SpiReadStatus(0xF5);  // MARCSTATE register
}

uint8_t QuietCool::getRxBytes() {
    return ELECHOUSE_cc1101.SpiReadStatus(0xFB);  // RXBYTES (bit 7 = overflow flag)
}

void QuietCool::calibrate() {
    ESP_LOGI(TAG, "Performing periodic frequency calibration");
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delay(2);
    ELECHOUSE_cc1101.SpiStrobe(0x33);  // SCAL - calibrate
    delay(10);
    ELECHOUSE_cc1101.SetRx();  // Back to RX
    delay(5);
    ESP_LOGI(TAG, "Calibration complete");
}

void QuietCool::forceRxMode() {
    ESP_LOGW(TAG, "Forcing CC1101 back to RX mode (flushing RX FIFO)");
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE - must go to IDLE first
    delayMicroseconds(100);
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX - flush RX FIFO (clears overflow)
    delayMicroseconds(100);
    ELECHOUSE_cc1101.SetRx();  // SRX - back to RX mode
    delayMicroseconds(100);
    ESP_LOGI(TAG, "RX FIFO flushed, back in RX mode");
}

// Read single byte from RX FIFO
uint8_t QuietCool::readRxByte() {
    return ELECHOUSE_cc1101.SpiReadReg(0x3F);  // RXFIFO single byte read
}

// Burst read from RX FIFO (single SPI transaction, more atomic than byte-by-byte)
void QuietCool::readRxBurst(uint8_t* buffer, uint8_t len) {
    ELECHOUSE_cc1101.SpiReadBurstReg(0x3F, buffer, len);
}

// Flush RX FIFO with proper IDLE transition (per CC1101 errata and TI E2E forums)
// CRITICAL: Must wait for IDLE state (0x01) before SFRX, otherwise chip gets stuck in SETTLING
void QuietCool::flushRxFifo() {
    // Step 1: Force IDLE state
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE

    // Step 2: Wait for IDLE (0x01) - CRITICAL to prevent SETTLING hang
    uint8_t marcstate;
    uint8_t retries = 0;
    do {
        delayMicroseconds(50);
        marcstate = ELECHOUSE_cc1101.SpiReadStatus(0xF5);  // MARCSTATE
        retries++;
    } while (marcstate != 0x01 && retries < 20);  // Max 1ms wait

    if (marcstate != 0x01) {
        ESP_LOGW(TAG, "SIDLE timeout (state 0x%02X)", marcstate);
    }

    // Step 3: Flush RX FIFO while in IDLE
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX

    // Step 4: Confirm still in IDLE after flush
    retries = 0;
    do {
        delayMicroseconds(50);
        marcstate = ELECHOUSE_cc1101.SpiReadStatus(0xF5);
        retries++;
    } while (marcstate != 0x01 && retries < 20);

    // Step 5: Re-enter RX mode
    ELECHOUSE_cc1101.SetRx();
    delayMicroseconds(100);
}

// Fast post-read recovery: flush FIFO and re-enter RX with minimal overhead.
// With RXOFF_MODE=00, CC1101 is already in IDLE after packet reception.
// Matches the ELECHOUSE library's ReceiveData() pattern: just SFRX + SRX.
void QuietCool::quickResetRx() {
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX — flush residual FIFO bytes
    ELECHOUSE_cc1101.SpiStrobe(0x34);  // SRX — enter RX mode
    // FS_AUTOCAL=01 handles calibration on IDLE→RX (~700μs, internal)
}

// Enter RX mode and wait until confirmed ready (per TI E2E recommendations)
void QuietCool::enterRxMode() {
    // Issue SRX strobe
    ELECHOUSE_cc1101.SetRx();

    // Wait for RX state (0x0D) to confirm ready
    uint8_t marcstate;
    uint8_t retries = 0;
    do {
        delayMicroseconds(50);
        marcstate = ELECHOUSE_cc1101.SpiReadStatus(0xF5);  // MARCSTATE
        retries++;
    } while (marcstate != 0x0D && retries < 30);  // Max 1.5ms wait

    if (marcstate != 0x0D) {
        ESP_LOGW(TAG, "SRX timeout (state 0x%02X after %d retries)", marcstate, retries);
    }
}

// Recover from FIFO overflow (0x11) or underflow (0x16) error states
void QuietCool::recoverFromFifoError() {
    ESP_LOGW(TAG, "Recovering from FIFO error state");
    // Full reset sequence: IDLE -> flush both FIFOs -> RX
    ELECHOUSE_cc1101.SpiStrobe(0x36);  // SIDLE
    delayMicroseconds(100);
    ELECHOUSE_cc1101.SpiStrobe(0x3B);  // SFTX - flush TX FIFO
    delayMicroseconds(100);
    ELECHOUSE_cc1101.SpiStrobe(0x3A);  // SFRX - flush RX FIFO
    delayMicroseconds(100);
    ELECHOUSE_cc1101.SetRx();
    delayMicroseconds(100);
    ESP_LOGI(TAG, "FIFO error recovery complete");
}

// Process complete 20-byte packet
void QuietCool::processPacket(const uint8_t* packet, size_t packet_len, uint32_t timestamp_ms) {
    if (!packet || packet_len < 20) {
        ESP_LOGE(TAG, "Invalid packet");
        return;
    }

    // Expected packet format after sync word (0x15 stripped by hardware):
    // Bytes 0-6:   AA AA AA AA AA AA AA  (7 sync bytes, 0x15 already stripped)
    // Bytes 7-13:  2D D4 06 CB 03 E6 45   (7-byte remote ID)
    // Bytes 14-15: XX XX                 (Command repeated)
    // Bytes 16-19: Status/padding bytes

    // Log full packet hex for ALL packets (diagnostic: find fan response packets)
    char hex[20*3 + 1];
    hex[0] = '\0';
    for (int i = 0; i < 20 && i < (int)packet_len; i++) {
        snprintf(hex + i*3, 4, "%02X ", packet[i]);
    }
    // Extract ID bytes (7-13) for easy visual comparison
    char id_hex[7*3 + 1];
    id_hex[0] = '\0';
    for (int i = 0; i < 7 && (7 + i) < (int)packet_len; i++) {
        snprintf(id_hex + i*3, 4, "%02X ", packet[7 + i]);
    }
    ESP_LOGD(TAG, "RX pkt: %s  ID=[%s] cmd=0x%02X,0x%02X", hex, id_hex,
             packet_len >= 15 ? packet[14] : 0, packet_len >= 16 ? packet[15] : 0);

    // Check for remote ID in expected position (bytes 7-13)
    bool match = true;
    for (uint8_t i = 0; i < 7; i++) {
        if (packet[7 + i] != remote_id[i]) {
            match = false;
            break;
        }
    }

    if (match) {
        // Extract command bytes
        uint8_t cmd1 = packet[14];
        uint8_t cmd2 = packet[15];

        // Decode known commands
        // Commands are: speed (upper nibble) | duration (lower nibble)
        uint8_t speed = cmd1 & 0xF0;
        uint8_t duration = cmd1 & 0x0F;

        // Reset last command
        last_rx_cmd_ = RxCommand{};

        if (cmd1 == 0x66 && cmd2 == 0x66) {
            ESP_LOGI(TAG, "RX: WAKE");
            last_rx_cmd_.valid = true;
            last_rx_cmd_.is_wake = true;
        } else if (cmd1 == 0x80 && cmd2 == 0x80) {
            ESP_LOGI(TAG, "RX: OFF");
            last_rx_cmd_.valid = true;
            last_rx_cmd_.is_off = true;
        } else if (cmd1 != cmd2) {
            // Command bytes don't match - likely corruption
            ESP_LOGW(TAG, "RX: CORRUPT (0x%02X != 0x%02X)", cmd1, cmd2);
        } else if (speed != 0x90 && speed != 0xA0 && speed != 0xB0) {
            // Invalid speed nibble — try bit-7 correction (known bit-sync corruption pattern:
            // MSB gets cleared, e.g. 0x9F→0x1F, 0xBF→0x3F). Setting bit 7 recovers the original.
            uint8_t corrected = cmd1 | 0x80;
            uint8_t corr_speed = corrected & 0xF0;
            uint8_t corr_duration = corrected & 0x0F;
            bool speed_ok = (corr_speed == 0x90 || corr_speed == 0xA0 || corr_speed == 0xB0);
            bool dur_ok = (corr_duration == 0x00 || corr_duration == 0x01 || corr_duration == 0x02 ||
                           corr_duration == 0x04 || corr_duration == 0x08 || corr_duration == 0x0C ||
                           corr_duration == 0x0F);
            if (speed_ok && dur_ok) {
                ESP_LOGW(TAG, "RX: BIT7 CORRECTED 0x%02X -> 0x%02X", cmd1, corrected);
                cmd1 = corrected;
                speed = corr_speed;
                duration = corr_duration;
                // Fall through to normal speed decoding below
            } else {
                ESP_LOGW(TAG, "RX: INVALID speed nibble 0x%02X (full cmd 0x%02X, uncorrectable)", speed, cmd1);
            }
        }

        // Duration validation (only reached if speed is valid or was corrected)
        if (speed == 0x90 || speed == 0xA0 || speed == 0xB0) {
        if (duration != 0x00 && duration != 0x01 && duration != 0x02 &&
            duration != 0x04 && duration != 0x08 && duration != 0x0C && duration != 0x0F) {
            // Invalid duration nibble
            ESP_LOGW(TAG, "RX: INVALID duration nibble 0x%02X (full cmd 0x%02X)", duration, cmd1);
        } else if (speed == 0xB0) {
            // HIGH speed
            if (duration == 0x00) {
                ESP_LOGI(TAG, "RX: HIGH PREP (fan is OFF)");
                last_rx_cmd_.valid = true;
                last_rx_cmd_.is_off = true;
            } else if (duration == 0x0F) {
                ESP_LOGI(TAG, "RX: HIGH");
            } else if (duration == 0x01) {
                ESP_LOGI(TAG, "RX: HIGH 1H");
            } else if (duration == 0x02) {
                ESP_LOGI(TAG, "RX: HIGH 2H");
            } else if (duration == 0x04) {
                ESP_LOGI(TAG, "RX: HIGH 4H");
            } else if (duration == 0x08) {
                ESP_LOGI(TAG, "RX: HIGH 8H");
            } else if (duration == 0x0C) {
                ESP_LOGI(TAG, "RX: HIGH 12H");
            } else {
                ESP_LOGD(TAG, "RX: HIGH+UNK_DUR 0x%02X", duration);
            }
            if (duration != 0x00) {
                last_rx_cmd_.valid = true;
                last_rx_cmd_.speed = speed;
                last_rx_cmd_.duration = duration;
            }
        } else if (speed == 0x90) {
            // LOW speed
            if (duration == 0x00) {
                ESP_LOGI(TAG, "RX: LOW PREP (fan is OFF)");
                last_rx_cmd_.valid = true;
                last_rx_cmd_.is_off = true;
            } else if (duration == 0x0F) {
                ESP_LOGI(TAG, "RX: LOW");
            } else if (duration == 0x01) {
                ESP_LOGI(TAG, "RX: LOW 1H");
            } else if (duration == 0x02) {
                ESP_LOGI(TAG, "RX: LOW 2H");
            } else if (duration == 0x04) {
                ESP_LOGI(TAG, "RX: LOW 4H");
            } else if (duration == 0x08) {
                ESP_LOGI(TAG, "RX: LOW 8H");
            } else if (duration == 0x0C) {
                ESP_LOGI(TAG, "RX: LOW 12H");
            } else {
                ESP_LOGD(TAG, "RX: LOW+UNK_DUR 0x%02X", duration);
            }
            if (duration != 0x00) {
                last_rx_cmd_.valid = true;
                last_rx_cmd_.speed = speed;
                last_rx_cmd_.duration = duration;
            }
        } else if (speed == 0xA0) {
            // MEDIUM speed
            if (duration == 0x00) {
                ESP_LOGI(TAG, "RX: MED PREP (fan is OFF)");
                last_rx_cmd_.valid = true;
                last_rx_cmd_.is_off = true;
            } else if (duration == 0x0F) {
                ESP_LOGI(TAG, "RX: MEDIUM");
            } else if (duration == 0x01) {
                ESP_LOGI(TAG, "RX: MEDIUM 1H");
            } else if (duration == 0x02) {
                ESP_LOGI(TAG, "RX: MEDIUM 2H");
            } else if (duration == 0x04) {
                ESP_LOGI(TAG, "RX: MEDIUM 4H");
            } else if (duration == 0x08) {
                ESP_LOGI(TAG, "RX: MEDIUM 8H");
            } else if (duration == 0x0C) {
                ESP_LOGI(TAG, "RX: MEDIUM 12H");
            } else {
                ESP_LOGD(TAG, "RX: MEDIUM+UNK_DUR 0x%02X", duration);
            }
            if (duration != 0x00) {
                last_rx_cmd_.valid = true;
                last_rx_cmd_.speed = speed;
                last_rx_cmd_.duration = duration;
            }
        } else {
            ESP_LOGD(TAG, "RX: UNK 0x%02X", cmd1);
        }
        }  // closes if (speed == 0x90 || ...)
    } else {
        // Minimal logging for speed - just note it's noise
        ESP_LOGV(TAG, "[T+%lu ms] Noise packet (ID mismatch)", timestamp_ms);

        // Check if this might be a partially corrupted packet with some valid bytes
        // Count how many bytes of remote ID match
        uint8_t match_count = 0;
        for (uint8_t i = 0; i < 7; i++) {
            if (packet[7 + i] == remote_id[i]) {
                match_count++;
            }
        }

        if (match_count >= 4) {
            // Partial match - likely corrupted packet from our remote
            ESP_LOGW(TAG, "[T+%lu ms] Partial remote ID match (%d/7 bytes)", timestamp_ms, match_count);
            // Still try to decode the command in case it's intact
            uint8_t cmd1 = packet[14];
            uint8_t cmd2 = packet[15];
            ESP_LOGW(TAG, "[T+%lu ms] Corrupted packet command: 0x%02X 0x%02X", timestamp_ms, cmd1, cmd2);

            // Check if it looks like a known command despite corruption
            if (cmd1 == cmd2) {
                if (cmd1 == 0x66) {
                    ESP_LOGW(TAG, "[T+%lu ms] Likely corrupted WAKE command", timestamp_ms);
                } else if (cmd1 == 0x80) {
                    ESP_LOGW(TAG, "[T+%lu ms] Likely corrupted OFF command", timestamp_ms);
                } else if ((cmd1 & 0xF0) == 0x90) {
                    ESP_LOGW(TAG, "[T+%lu ms] Likely corrupted LOW command", timestamp_ms);
                } else if ((cmd1 & 0xF0) == 0xA0) {
                    ESP_LOGW(TAG, "[T+%lu ms] Likely corrupted MEDIUM command", timestamp_ms);
                } else if ((cmd1 & 0xF0) == 0xB0) {
                    ESP_LOGW(TAG, "[T+%lu ms] Likely corrupted HIGH command", timestamp_ms);
                }
            }
        }

        // Also check if remote ID might be at a different offset (alignment issue)
        for (uint8_t offset = 0; offset <= 13; offset++) {
            if (offset == 7) continue; // Already checked offset 7

            bool alt_match = true;
            for (uint8_t i = 0; i < 7 && (offset + i) < 20; i++) {
                if (packet[offset + i] != remote_id[i]) {
                    alt_match = false;
                    break;
                }
            }
            if (alt_match) {
                ESP_LOGW(TAG, "[T+%lu ms] Remote ID found at offset %d instead of 7!",
                         timestamp_ms, offset);
                // Try to decode command at adjusted offset
                if (offset + 7 < 18) {
                    uint8_t cmd1 = packet[offset + 7];
                    uint8_t cmd2 = packet[offset + 8];
                    ESP_LOGW(TAG, "[T+%lu ms] Command at adjusted offset: 0x%02X 0x%02X",
                             timestamp_ms, cmd1, cmd2);
                }
                break;
            }
        }
    }
}

RxCommand QuietCool::consumeRxCommand() {
    RxCommand cmd = last_rx_cmd_;
    last_rx_cmd_ = RxCommand{};  // Clear after consuming
    return cmd;
}

// Legacy readRxData - kept for compatibility but not used in sliding buffer mode
bool QuietCool::readRxData(uint8_t* buffer, uint8_t max_len, uint8_t* bytes_read) {
    if (!buffer || !bytes_read) {
        ESP_LOGE(TAG, "Invalid buffer or bytes_read pointer");
        return false;
    }

    // Check how many bytes are available
    uint8_t rxbytes = ELECHOUSE_cc1101.SpiReadStatus(0xFB) & 0x7F;

    if (rxbytes == 0) {
        *bytes_read = 0;
        return false;
    }

    // Limit to buffer size
    uint8_t to_read = (rxbytes > max_len) ? max_len : rxbytes;

    // Read data from FIFO
    for (uint8_t i = 0; i < to_read; i++) {
        buffer[i] = readRxByte();
    }

    *bytes_read = to_read;
    return true;
}

}  // namespace quiet_cool
}  // namespace esphome 
