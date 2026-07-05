// Ninebot G30 dashboard <-> Flipsky FSESC75100 Pro (VESC) UART bridge.
// M5StickC Plus2 (ESP32-PICO-V3-02) relay. Throttle/brake only for now.
//
// Dashboard decode logic ported from tonymillion/VescNinebotDash ninebotdash.lisp.
// VESC command IDs / CRC / framing verified against vedderb/bldc (comm/commands.c,
// comm/packet.c, util/crc.c, util/buffer.c) rather than assumed.

#include <M5Unified.h>
#include <HardwareSerial.h>
#include <string.h>

// VESC UART has no relative brake command (only COMM_SET_CURRENT_BRAKE exists over
// UART; the "_REL" relative brake variant is CAN-only, see CAN_PACKET_ID in
// datatypes.h). So brake lever position is sent as an absolute amps value scaled
// against this ceiling, not a 0-1 relative fraction.
//
// MUST stay at or below the VESC's configured Motor Current Max Brake (currently
// -50A on this setup). Deliberately set low for first bench testing -- raise it
// yourself once the relay is confirmed working correctly.
static constexpr float MAX_BRAKE_CURRENT_A = 12.0f;

// ---- Pin definitions (fixed by hardware wiring, do not change) ------------
static constexpr int PIN_DASH_HALF_DUPLEX = 26; // Ninebot dash TX/RX, single-wire half-duplex
static constexpr int PIN_DASH_BUTTON      = 36; // dash button (green) wire, unused for now
static constexpr int PIN_VESC_TX          = 25; // -> VESC CONN6 RX/SDA
static constexpr int PIN_VESC_RX          = 0;  // <- VESC CONN6 TX/SCL

static constexpr uint32_t UART_BAUD = 115200;

HardwareSerial DashSerial(1);
HardwareSerial VescSerial(2);

// =============================================================================
// Ninebot dashboard protocol
// =============================================================================
namespace Dash {

constexpr uint8_t HEADER_0 = 0x5A;
constexpr uint8_t HEADER_1 = 0xA5;
constexpr uint8_t ADDR_ESC = 0x20;
constexpr uint8_t ADDR_DASH = 0x21;
constexpr uint8_t CMD_STATUS_REQUEST = 0x64; // dash asks for a status update
constexpr uint8_t CMD_HALL_UPDATE = 0x65;    // dash sends throttle/brake hall levels

// Raw hall ranges and rel-deadzone, taken directly from ninebotdash.lisp.
constexpr float THROTTLE_LOW = 40.0f;
constexpr float THROTTLE_HIGH = 197.0f - THROTTLE_LOW;
constexpr float BRAKE_LOW = 41.0f;
constexpr float BRAKE_HIGH = 181.0f - BRAKE_LOW;
constexpr float REL_DEADZONE = 0.002f;

enum class RxState : uint8_t {
  WAIT_HEADER_0,
  WAIT_HEADER_1,
  READ_INFO,
  READ_PAYLOAD,
  READ_CRC,
};

RxState state = RxState::WAIT_HEADER_0;
uint8_t infoBuf[5];  // bLen, bSrcAddr, bDstAddr, bCmd, bArg
uint8_t infoIdx = 0;
uint8_t payloadBuf[256];
uint8_t payloadIdx = 0;
uint8_t bLen = 0;
uint8_t crcBuf[2];
uint8_t crcIdx = 0;

float throttleRel = 0.0f;
float brakeRel = 0.0f;
bool brakeActive = false;
volatile bool newData = false;
uint32_t lastRxMs = 0;

// wChecksumLE = 0xFFFF xor (16-bit sum of bLen,bSrcAddr,bDstAddr,bCmd,bArg,payload[]),
// transmitted low-byte-first. This is the formula documented in the reference script's
// comments and correctly implemented by its send-dash-update(). Its calc-crc() (used for
// RX validation) has a variable-capture bug that makes it effectively compare the received
// CRC's byte-swap to itself rather than compute a real checksum, so we use the documented
// formula for both TX and RX instead of porting that bug.
uint16_t checksum(const uint8_t* info, const uint8_t* payload, uint8_t len) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 5; i++) sum += info[i];
  for (uint8_t i = 0; i < len; i++) sum += payload[i];
  return (uint16_t)(sum & 0xFFFF) ^ 0xFFFF;
}

void decodeHallUpdate() {
  // Payload layout per reference comment:
  // <bDataLen> <bThrottleLevel> <bBrakeLevel> <bIsUpdatingBLEFW> <bIsBeeping>
  if (bLen < 3) return;
  uint8_t rawThrottle = payloadBuf[1];
  uint8_t rawBrake = payloadBuf[2];

  float t = (rawThrottle - THROTTLE_LOW) / THROTTLE_HIGH;
  throttleRel = (t > REL_DEADZONE) ? t : 0.0f;
  if (throttleRel < 0.0f) throttleRel = 0.0f;
  if (throttleRel > 1.0f) throttleRel = 1.0f;

  float b = (rawBrake - BRAKE_LOW) / BRAKE_HIGH;
  brakeActive = (b > REL_DEADZONE);
  brakeRel = brakeActive ? b : 0.0f;
  if (brakeRel > 1.0f) brakeRel = 1.0f;

  lastRxMs = millis();
  newData = true;

  Serial.printf("[DASH] cmd=0x%02X raw(thr=%3u brk=%3u) -> thrRel=%.3f brkRel=%.3f%s\n",
                infoBuf[3], rawThrottle, rawBrake, throttleRel, brakeRel,
                brakeActive ? " [BRAKE]" : "");
}

void sendStatusReply() {
  // Minimal 0x64 reply so the dashboard doesn't fault/disconnect.
  // Frame: 5A A5 <bLen=06> 20 21 64 00 <bFlags> <bBattLevel> <bHeadlightLevel> <bBeeps> <bSpeed> <bErrorCode> crcLo crcHi
  uint8_t info[5] = {0x06, ADDR_ESC, ADDR_DASH, CMD_STATUS_REQUEST, 0x00};
  uint8_t payload[6];
  payload[0] = 0x04;  // bFlags: drive mode; no eco/sport/charge/off/lock/mph/hw-problem
  payload[1] = 100;   // bBattLevel: report full, VESC battery level not tracked yet
  payload[2] = 0;      // bHeadlightLevel
  payload[3] = 0;      // bBeeps
  payload[4] = 0;      // bSpeed (0.1 km/h units), not tracked yet
  payload[5] = 0;      // bErrorCode

  uint16_t crc = checksum(info, payload, sizeof(payload));

  uint8_t frame[2 + 5 + 6 + 2];
  size_t i = 0;
  frame[i++] = HEADER_0;
  frame[i++] = HEADER_1;
  memcpy(&frame[i], info, sizeof(info));
  i += sizeof(info);
  memcpy(&frame[i], payload, sizeof(payload));
  i += sizeof(payload);
  frame[i++] = crc & 0xFF;
  frame[i++] = (crc >> 8) & 0xFF;

  DashSerial.write(frame, i);
}

void poll() {
  while (DashSerial.available()) {
    uint8_t b = (uint8_t)DashSerial.read();

    switch (state) {
      case RxState::WAIT_HEADER_0:
        if (b == HEADER_0) state = RxState::WAIT_HEADER_1;
        break;

      case RxState::WAIT_HEADER_1:
        if (b == HEADER_1) {
          state = RxState::READ_INFO;
          infoIdx = 0;
        } else if (b != HEADER_0) {
          state = RxState::WAIT_HEADER_0;
        }
        break;

      case RxState::READ_INFO:
        infoBuf[infoIdx++] = b;
        if (infoIdx >= 5) {
          bLen = infoBuf[0];
          payloadIdx = 0;
          crcIdx = 0;
          state = (bLen > 0) ? RxState::READ_PAYLOAD : RxState::READ_CRC;
        }
        break;

      case RxState::READ_PAYLOAD:
        payloadBuf[payloadIdx++] = b;
        if (payloadIdx >= bLen) {
          crcIdx = 0;
          state = RxState::READ_CRC;
        }
        break;

      case RxState::READ_CRC:
        crcBuf[crcIdx++] = b;
        if (crcIdx >= 2) {
          uint16_t received = crcBuf[0] | (crcBuf[1] << 8);
          uint16_t expected = checksum(infoBuf, payloadBuf, bLen);

          if (received == expected) {
            uint8_t cmd = infoBuf[3];
            if (cmd == CMD_STATUS_REQUEST) {
              // 0x64: dash wants a status reply; per reference comment it also
              // carries hall data, so decode it too.
              decodeHallUpdate();
              sendStatusReply();
            } else if (cmd == CMD_HALL_UPDATE) {
              // 0x65: hall data only, no reply expected.
              decodeHallUpdate();
            }
          } else {
            Serial.printf("[DASH] CRC mismatch: got %04X expected %04X\n", received, expected);
          }

          state = RxState::WAIT_HEADER_0;
        }
        break;
    }
  }
}

}  // namespace Dash

// =============================================================================
// VESC UART protocol (command IDs / CRC / framing from vedderb/bldc)
// =============================================================================
namespace Vesc {

constexpr uint8_t COMM_SET_CURRENT_BRAKE = 7;
constexpr uint8_t COMM_SET_CURRENT_REL = 84;
constexpr uint8_t COMM_ALIVE = 30;

// CRC-16/XMODEM (poly 0x1021, init 0), copied from util/crc.c.
const uint16_t crc16_tab[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
};

uint16_t crc16(const uint8_t* buf, size_t len) {
  uint16_t cksum = 0;
  for (size_t i = 0; i < len; i++) {
    cksum = crc16_tab[((cksum >> 8) ^ buf[i]) & 0xFF] ^ (cksum << 8);
  }
  return cksum;
}

// Standard short VESC UART packet: 0x02 <len> <payload> <crc-hi> <crc-lo> 0x03
void sendPacket(const uint8_t* payload, uint8_t len) {
  uint16_t crc = crc16(payload, len);
  uint8_t frame[3 + 255 + 2];
  size_t i = 0;
  frame[i++] = 0x02;
  frame[i++] = len;
  memcpy(&frame[i], payload, len);
  i += len;
  frame[i++] = (crc >> 8) & 0xFF;
  frame[i++] = crc & 0xFF;
  frame[i++] = 0x03;
  VescSerial.write(frame, i);
}

void appendInt32BE(uint8_t* buf, int32_t v, int& idx) {
  buf[idx++] = (v >> 24) & 0xFF;
  buf[idx++] = (v >> 16) & 0xFF;
  buf[idx++] = (v >> 8) & 0xFF;
  buf[idx++] = v & 0xFF;
}

void setCurrentRel(float rel) {
  uint8_t payload[5];
  int idx = 0;
  payload[idx++] = COMM_SET_CURRENT_REL;
  appendInt32BE(payload, (int32_t)(rel * 100000.0f), idx);
  sendPacket(payload, idx);
}

void setBrakeCurrent(float amps) {
  uint8_t payload[5];
  int idx = 0;
  payload[idx++] = COMM_SET_CURRENT_BRAKE;
  appendInt32BE(payload, (int32_t)(amps * 1000.0f), idx);
  sendPacket(payload, idx);
}

void sendAlive() {
  uint8_t payload[1] = {COMM_ALIVE};
  sendPacket(payload, 1);
}

}  // namespace Vesc

// =============================================================================
// Main relay loop
// =============================================================================
static constexpr uint32_t ALIVE_INTERVAL_MS = 500;  // watchdog backstop, well under the 1s requirement
uint32_t lastAliveMs = 0;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  pinMode(PIN_DASH_BUTTON, INPUT);  // defined for later use, not read yet

  // Dashboard bus is single-wire half-duplex: same GPIO used for RX and TX.
  DashSerial.begin(UART_BAUD, SERIAL_8N1, PIN_DASH_HALF_DUPLEX, PIN_DASH_HALF_DUPLEX);
  VescSerial.begin(UART_BAUD, SERIAL_8N1, PIN_VESC_RX, PIN_VESC_TX);

  Serial.println("Ninebot G30 <-> VESC UART bridge starting");
}

void loop() {
  M5.update();

  Dash::poll();

  if (Dash::newData) {
    Dash::newData = false;
    Vesc::setCurrentRel(Dash::throttleRel);
    if (Dash::brakeActive) {
      Vesc::setBrakeCurrent(Dash::brakeRel * MAX_BRAKE_CURRENT_A);
    }
  }

  uint32_t now = millis();
  if (now - lastAliveMs >= ALIVE_INTERVAL_MS) {
    lastAliveMs = now;
    Vesc::sendAlive();
  }
}
