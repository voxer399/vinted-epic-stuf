// Ninebot G30 dashboard <-> Flipsky FSESC75100 Pro (VESC) UART bridge.
// M5StickC Plus2 (ESP32-PICO-V3-02) relay. Throttle/brake only for now.
//
// Dashboard decode logic ported from tonymillion/VescNinebotDash ninebotdash.lisp.
// VESC command IDs / CRC / framing verified against vedderb/bldc (comm/commands.c,
// comm/packet.c, util/crc.c, util/buffer.c) rather than assumed.

#include <M5Unified.h>
#include <HardwareSerial.h>
#include <string.h>
#include <driver/uart.h>
#include <driver/gpio.h>

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
static constexpr int PIN_VESC_TX          = 25; // -> VESC CONN6 RX/SDA
static constexpr int PIN_VESC_RX          = 0;  // <- VESC CONN6 TX/SCL

static constexpr uint32_t UART_BAUD = 115200;

// The dashboard bus is single-wire half-duplex, so RX and TX share PIN_DASH_HALF_DUPLEX.
// This is deliberately NOT a HardwareSerial: Arduino-ESP32's HardwareSerial/uartSetPins()
// attaches RX and TX through its "peripheral manager" (periman), which tracks one bus
// owner per pin -- attaching TX to a pin that periman already marked as UART_RX detaches
// the RX assignment first. Passing the same pin for both rxPin/txPin to
// HardwareSerial::begin() therefore silently ends up with only TX connected (confirmed
// against arduino-esp32's esp32-hal-uart.c _uartAttachPins()/uartSetPins()), which matches
// the observed symptom: dashboard Error 10, zero bytes ever received.
//
// ESP-IDF's own uart_set_pin() (components/esp_driver_uart/src/uart.c) explicitly supports
// tx_io_num == rx_io_num: it detects tx_rx_same_io and routes both TX-out and RX-in through
// the GPIO matrix onto the one pad. It does NOT configure open-drain though -- per ESP-IDF's
// UART docs, sharing one wire between two drivers needs the pad set open-drain (+ pull-up)
// or the two ends contending on the line can damage it. So we call the raw driver directly
// (uart_driver_install/uart_param_config/uart_set_pin) and then override the pad to
// open-drain + pull-up ourselves with gpio_set_direction()/gpio_set_pull_mode().
static constexpr uart_port_t DASH_UART_NUM = UART_NUM_1;

void dashUartInit() {
  uart_config_t cfg = {};
  cfg.baud_rate = UART_BAUD;
  cfg.data_bits = UART_DATA_8_BITS;
  cfg.parity = UART_PARITY_DISABLE;
  cfg.stop_bits = UART_STOP_BITS_1;
  cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  // UART_SCLK_DEFAULT isn't available in every ESP-IDF version this can build against;
  // UART_SCLK_APB is the original-ESP32 clock source and has been valid since IDF v4.x.
  cfg.source_clk = UART_SCLK_APB;

  ESP_ERROR_CHECK(uart_driver_install(DASH_UART_NUM, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(DASH_UART_NUM, &cfg));
  ESP_ERROR_CHECK(uart_set_pin(DASH_UART_NUM, PIN_DASH_HALF_DUPLEX, PIN_DASH_HALF_DUPLEX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

  // Override the pad uart_set_pin() just configured: open-drain so a TX '1' doesn't
  // fight a dashboard-driven '0' (or vice versa), pull-up so the line idles high like
  // a normal UART line when nobody is driving it.
  gpio_set_direction((gpio_num_t)PIN_DASH_HALF_DUPLEX, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode((gpio_num_t)PIN_DASH_HALF_DUPLEX, GPIO_PULLUP_ONLY);
}

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

// Debug/telemetry state, purely for the on-screen display -- not used for relay logic.
uint32_t rxPacketCount = 0;
uint32_t crcErrorCount = 0;
uint8_t lastCmd = 0;
uint8_t lastLen = 0;
uint8_t lastRawThrottle = 0;
uint8_t lastRawBrake = 0;

// Timestamp of the most recently read byte from the dash UART, in microseconds --
// used to measure our reply turnaround time in sendStatusReply() (see below).
uint32_t lastByteRxUs = 0;

// EXPERIMENTAL: 0 when no reply is pending a self-echo check, else the millis() we
// last transmitted a reply. poll() logs (without discarding or diverting) every raw
// byte it reads for a short window after this timestamp, so we can empirically see
// whether our own transmitted bytes are looping back into our own RX FIFO --
// UART_MODE_RS485_HALF_DUPLEX was never enabled, so nothing currently suppresses our
// receiver while we transmit on the shared pad.
uint32_t lastTxMs = 0;
constexpr uint32_t ECHO_CHECK_WINDOW_MS = 15;

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
  lastRawThrottle = rawThrottle;
  lastRawBrake = rawBrake;

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

// Throttle/brake -> dashboard number-display + beeper feedback, via the fields
// send-dash-update() in the reference actually writes: bSpeed (payload[4]) drives the
// dash's own numeric display, so we show 99/88 there while throttle/brake is held. bBeeps
// (payload[3]) is documented in the reference only as ";bBeeps - beeper", hardcoded to 0
// everywhere with no working nonzero example -- its trigger semantics (level- vs
// edge-triggered, flag vs pattern code) are NOT confirmed from the source. We assume the
// most conservative reading (nonzero = beep on, zero = silent) and approximate a
// fast/slow beep *pattern* with our own millis() oscillator, sampled whenever a reply
// actually goes out (bounded by the dash's own 0x64 poll rate, which we don't control or
// know). Continuous-while-fully-pressed should be reliable either way; the fast-vs-slow
// pulse distinction is best-effort and needs verifying on real hardware. Brake takes
// priority over throttle if both are pressed at once.
constexpr float FULL_PRESS_THRESHOLD = 0.95f;
constexpr uint32_t THROTTLE_PULSE_PERIOD_MS = 150;  // fast pulse, ~3.3 Hz on/off
constexpr uint32_t BRAKE_PULSE_PERIOD_MS = 700;     // slow pulse, ~0.7 Hz on/off

uint8_t feedbackSpeedByte = 0;
bool feedbackBeepOn = false;

void updateFeedback() {
  uint32_t now = millis();

  if (brakeActive) {
    feedbackSpeedByte = 88;
    bool full = brakeRel >= FULL_PRESS_THRESHOLD;
    feedbackBeepOn = full || ((now % BRAKE_PULSE_PERIOD_MS) < (BRAKE_PULSE_PERIOD_MS / 2));
  } else if (throttleRel > 0.0f) {
    feedbackSpeedByte = 99;
    bool full = throttleRel >= FULL_PRESS_THRESHOLD;
    feedbackBeepOn = full || ((now % THROTTLE_PULSE_PERIOD_MS) < (THROTTLE_PULSE_PERIOD_MS / 2));
  } else {
    feedbackSpeedByte = 0;
    feedbackBeepOn = false;
  }
}

void sendStatusReply() {
  updateFeedback();

  // Minimal 0x64 reply so the dashboard doesn't fault/disconnect.
  // Frame: 5A A5 <bLen=06> 20 21 64 00 <bFlags> <bBattLevel> <bHeadlightLevel> <bBeeps> <bSpeed> <bErrorCode> crcLo crcHi
  uint8_t info[5] = {0x06, ADDR_ESC, ADDR_DASH, CMD_STATUS_REQUEST, 0x00};
  uint8_t payload[6];
  payload[0] = 0x04;  // bFlags: drive mode; no eco/sport/charge/off/lock/mph/hw-problem
  payload[1] = 100;   // bBattLevel: report full, VESC battery level not tracked yet
  payload[2] = 0;      // bHeadlightLevel
  payload[3] = feedbackBeepOn ? 1 : 0;  // bBeeps
  payload[4] = feedbackSpeedByte;        // bSpeed: 99/88 feedback, else pass-through (0, not tracked yet)
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

  // Turnaround diagnostics: how long after the last byte of the incoming request was
  // read do we start transmitting the reply, and exactly what bytes are we sending.
  // Our dashUartInit() open-drain hack has no built-in half-duplex guard time (unlike
  // hardware RS485 mode), so if this turnaround is too short the dashboard's own UART
  // may not have switched back to listening yet when our reply hits the wire.
  uint32_t turnaroundUs = micros() - lastByteRxUs;
  Serial.printf("[DASH] TX turnaround=%luus frame:", (unsigned long)turnaroundUs);
  for (size_t j = 0; j < i; j++) {
    Serial.printf(" %02X", frame[j]);
  }
  Serial.println();

  // EXPERIMENTAL: measured turnaround with no guard band was 230-450us -- CRC/structure
  // confirmed correct by hand, so this tests whether the dashboard's own UART simply
  // hasn't switched back to listening yet when our reply lands. Test value only; dial
  // down (or remove) once we know whether/how much guard time this dashboard actually
  // needs. Deliberately blocking -- this is a bus-turnaround requirement, not part of
  // the main non-blocking relay loop, and only runs once per 0x64 reply.
  constexpr uint32_t TX_GUARD_DELAY_US = 1500;
  delayMicroseconds(TX_GUARD_DELAY_US);

  uart_write_bytes(DASH_UART_NUM, reinterpret_cast<const char*>(frame), i);
  lastTxMs = millis();  // arms the post-TX echo-check window in poll()
}

void poll() {
  uint8_t b;
  while (uart_read_bytes(DASH_UART_NUM, &b, 1, 0) == 1) {
    lastByteRxUs = micros();

    // EXPERIMENTAL: log (without altering parsing below) every raw byte seen shortly
    // after our last TX, to empirically check for self-echo. Purely additive -- the
    // switch statement below still processes this same byte normally either way.
    if (lastTxMs != 0) {
      uint32_t sinceTxMs = millis() - lastTxMs;
      if (sinceTxMs < ECHO_CHECK_WINDOW_MS) {
        Serial.printf("[DASH] post-TX byte @+%lums: %02X\n", (unsigned long)sinceTxMs, b);
      } else {
        lastTxMs = 0;  // window elapsed, stop logging until the next reply
      }
    }
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
            rxPacketCount++;
            uint8_t cmd = infoBuf[3];
            lastCmd = cmd;
            lastLen = bLen;
            // EXPERIMENTAL: the reference script's dispatch is two independent `if`s
            // (not `cond`), which -- worked through -- actually calls both
            // send-dash-update() and process_hall_update() for EITHER 0x64 or 0x65,
            // contradicting its own comments ("0x65 does not expect a reply"). We'd
            // previously followed the commented intent (reply only to 0x64); this
            // instead matches the reference's literal runtime behavior, since the
            // dashboard sends ~4x more 0x65 than 0x64 and we were only ever replying
            // to ~20% of its traffic.
            if (cmd == CMD_STATUS_REQUEST || cmd == CMD_HALL_UPDATE) {
              decodeHallUpdate();
              sendStatusReply();
            }
          } else {
            crcErrorCount++;
            Serial.printf("[DASH] CRC mismatch: got %04X expected %04X\n", received, expected);
          }

          state = RxState::WAIT_HEADER_0;
        }
        break;
    }
  }
}

// Periodic diagnostic (was one-shot at boot, but that made it impossible to catch in
// the Serial Monitor unless you were already attached before the board booted): sends
// a byte pattern that could never appear in real dashboard traffic (0x5A/0xA5 are the
// only "special" bytes in this protocol) and immediately checks our own RX for it. If
// it comes back, our open-drain TX is definitely producing valid, receivable logic
// levels on the shared pad -- ruling the TX path itself in as electrically functional.
// If it doesn't come back, that's a real signal something is wrong with TX
// continuity/config, separate from protocol, timing, or pull-up tuning (all already
// ruled out by direct testing).
//
// This dashboard talks fast enough (a full request/reply burst every few ms) that a
// generous listen window reliably catches real dashboard bytes racing into it -- that's
// not a failed loopback, it's contamination. So: a short window (our 5-byte pattern only
// needs ~435us to physically clock out at 115200 baud), and explicit retries whenever
// what comes back looks like real traffic rather than silence or our own pattern.
constexpr uint32_t LOOPBACK_TEST_INTERVAL_MS = 5000;
uint32_t lastLoopbackTestMs = 0;

void loopbackSelfTest() {
  static const uint8_t TEST_PATTERN[] = {0xAA, 0x55, 0xAA, 0x55, 0xAA};
  constexpr size_t TEST_LEN = sizeof(TEST_PATTERN);
  constexpr int MAX_ATTEMPTS = 5;
  constexpr uint32_t WINDOW_MS = 3;  // ~7x the ~435us physical TX time, not 20ms

  for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
    // Discard anything already sitting in the RX FIFO so it can't contaminate this attempt.
    uint8_t discard;
    while (uart_read_bytes(DASH_UART_NUM, &discard, 1, 0) == 1) {
    }

    uart_write_bytes(DASH_UART_NUM, reinterpret_cast<const char*>(TEST_PATTERN), TEST_LEN);

    uint8_t rxBuf[TEST_LEN] = {0};
    size_t rxCount = 0;
    uint32_t deadline = millis() + WINDOW_MS;
    while (rxCount < TEST_LEN && millis() < deadline) {
      uint8_t b;
      if (uart_read_bytes(DASH_UART_NUM, &b, 1, 0) == 1) {
        rxBuf[rxCount++] = b;
      }
    }

    bool match = (rxCount == TEST_LEN) && (memcmp(rxBuf, TEST_PATTERN, TEST_LEN) == 0);

    Serial.printf("[DASH] Loopback self-test attempt %d/%d: sent %u, got %u back:", attempt, MAX_ATTEMPTS,
                  (unsigned)TEST_LEN, (unsigned)rxCount);
    for (size_t i = 0; i < rxCount; i++) {
      Serial.printf(" %02X", rxBuf[i]);
    }
    Serial.println();

    if (match) {
      Serial.println("[DASH] Loopback self-test: PASS (TX reaches RX electrically)");
      return;
    }
    if (rxCount > 0) {
      // Got bytes, but not our pattern -- almost certainly real dashboard traffic that
      // raced into the window, not evidence of anything. Retry rather than count it.
      Serial.println("[DASH] Loopback self-test: unrelated bytes received (likely real dash traffic racing the test) -- retrying");
    }
  }

  Serial.println("[DASH] Loopback self-test: FAIL after all attempts (no confirmed electrical loopback)");
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

uint32_t aliveSentCount = 0;  // debug/telemetry only

void sendAlive() {
  uint8_t payload[1] = {COMM_ALIVE};
  sendPacket(payload, 1);
  aliveSentCount++;
}

}  // namespace Vesc

// =============================================================================
// On-screen debug display (M5StickC Plus2 LCD) -- purely for bring-up, not
// part of the relay logic itself.
// =============================================================================
namespace Screen {

constexpr uint32_t DRAW_INTERVAL_MS = 100;  // ~10 Hz, gated so it can't slow the relay loop
constexpr uint32_t LINK_TIMEOUT_MS = 1000;  // no dash packet in this long -> show link down
uint32_t lastDrawMs = 0;

void init() {
  M5.Display.setRotation(1);  // landscape
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Display.fillScreen(TFT_BLACK);
}

void draw() {
  uint32_t now = millis();
  if (now - lastDrawMs < DRAW_INTERVAL_MS) return;
  lastDrawMs = now;

  uint32_t sinceRx = now - Dash::lastRxMs;
  bool linkUp = (Dash::lastRxMs != 0) && (sinceRx < LINK_TIMEOUT_MS);

  M5.Display.startWrite();
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.printf("DASH: %s\n", linkUp ? "LINK" : "-----");
  M5.Display.printf("cmd=%02X len=%u\n", Dash::lastCmd, Dash::lastLen);
  M5.Display.printf("raw  t=%3u b=%3u\n", Dash::lastRawThrottle, Dash::lastRawBrake);
  M5.Display.printf("rel  t=%.2f b=%.2f%s\n", Dash::throttleRel, Dash::brakeRel,
                     Dash::brakeActive ? " BRK" : "");
  M5.Display.printf("rx=%lu err=%lu\n", (unsigned long)Dash::rxPacketCount,
                     (unsigned long)Dash::crcErrorCount);
  M5.Display.printf("age=%lums alive=%lu\n", (unsigned long)sinceRx,
                     (unsigned long)Vesc::aliveSentCount);
  M5.Display.endWrite();
}

}  // namespace Screen

// =============================================================================
// Main relay loop
// =============================================================================
static constexpr uint32_t ALIVE_INTERVAL_MS = 500;  // watchdog backstop, well under the 1s requirement
uint32_t lastAliveMs = 0;

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  Serial.begin(115200);

  // Dashboard bus is single-wire half-duplex: same GPIO used for RX and TX, via the
  // raw ESP-IDF UART driver + open-drain override (see dashUartInit() above for why).
  dashUartInit();
  VescSerial.begin(UART_BAUD, SERIAL_8N1, PIN_VESC_RX, PIN_VESC_TX);

  Screen::init();

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

  if (now - Dash::lastLoopbackTestMs >= Dash::LOOPBACK_TEST_INTERVAL_MS) {
    Dash::lastLoopbackTestMs = now;
    Dash::loopbackSelfTest();
  }

  // EXPERIMENTAL: temporarily disabled to test whether Screen::draw()'s SPI redraw
  // (fires every DRAW_INTERVAL_MS) stalls poll() long enough to occasionally miss the
  // dashboard's reply deadline, given Error 10 has never once cleared even though most
  // individual replies verify as correct. Re-enable once this is ruled in or out.
  // Screen::draw();
}
