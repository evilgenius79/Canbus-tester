/*
 * ============================================================
 *  CAN Bus Tester
 *  Hardware : M5StickC PLUS2 + M5Stack CAN Unit (CA-IS3050G)
 * ============================================================
 *
 *  WIRING — Grove cable from M5StickC PLUS2 to CAN Unit
 *  ┌─────────────────────────────────────────────────────┐
 *  │  Grove Pin 1  yellow  GPIO32  TWAI RX  ← from CAN  │
 *  │  Grove Pin 2  white   GPIO33  TWAI TX  → to CAN    │
 *  │  Grove Pin 3  red     5 V                           │
 *  │  Grove Pin 4  black   GND                           │
 *  └─────────────────────────────────────────────────────┘
 *
 *  CAN bus termination: each physical end of the bus needs a
 *  120 Ω resistor between CANH and CANL.  The CAN Unit has a
 *  solder-jumper termination that can be enabled.
 *
 *  BUTTONS
 *  ┌──────────────────────────────────────────────────────┐
 *  │  Button A  (front, GPIO37)  short-press → next speed │
 *  │  Button B  (power, GPIO39)  short-press → next mode  │
 *  └──────────────────────────────────────────────────────┘
 *
 *  MODES
 *  ┌─────────────┬───────────────────────────────────────┐
 *  │ LISTEN      │ Silent monitor, never transmits.      │
 *  │             │ Safe to connect to any live bus.      │
 *  ├─────────────┼───────────────────────────────────────┤
 *  │ ACTIVE      │ Sends test frames every 1 s.          │
 *  │             │ Needs ≥1 other node for ACK.          │
 *  ├─────────────┼───────────────────────────────────────┤
 *  │ NO-ACK      │ Sends frames without requiring ACK.  │
 *  │             │ Tests TX path with no partner.        │
 *  ├─────────────┼───────────────────────────────────────┤
 *  │ AUTO-SCAN   │ Silently cycles through all speeds    │
 *  │             │ (125K→250K→500K→1000K) every 3 s     │
 *  │             │ until frames are received, then locks.│
 *  └─────────────┴───────────────────────────────────────┘
 *
 *  DISPLAY LAYOUT  (landscape 240 × 135)
 *  ┌────────────────────────────────────────────────────┐
 *  │  CAN BUS TESTER       [MODE]           [SPEED]     │
 *  │  STATUS: PASS communicating  last ID: 0x1A0        │
 *  │  TX: 5       RX: 42      ERR: 0                    │
 *  │  ─────────────────────────────────────────────     │
 *  │  0x001[8]: CA FE 01 AA 00 00 00 00                 │
 *  │  0x1A0[8]: 10 00 00 00 00 00 80 F0                 │
 *  │  …  (up to 7 most-recent frames)                   │
 *  │  [A] Speed   [B] Mode   total RX: 42               │
 *  └────────────────────────────────────────────────────┘
 */

#include <M5Unified.h>
#include "driver/twai.h"
#include <string.h>

// ─────────────────────────────────────────────────────────────
//  Hardware — Grove port pins on M5StickC PLUS2
//  Grove pin 1 (yellow) = SDA position = GPIO32 = TWAI RX
//  Grove pin 2 (white)  = SCL position = GPIO33 = TWAI TX
// ─────────────────────────────────────────────────────────────
static const gpio_num_t PIN_CAN_TX = GPIO_NUM_33;
static const gpio_num_t PIN_CAN_RX = GPIO_NUM_32;

// ─────────────────────────────────────────────────────────────
//  Display (landscape after setRotation(3))
// ─────────────────────────────────────────────────────────────
static const int SCR_W = 240;
static const int SCR_H = 135;
static M5Canvas  canvas(&M5.Display);

// ─────────────────────────────────────────────────────────────
//  CAN operating modes
// ─────────────────────────────────────────────────────────────
enum CanMode : uint8_t {
    MODE_LISTEN   = 0,
    MODE_ACTIVE   = 1,
    MODE_NOACK    = 2,
    MODE_AUTOSCAN = 3,   // silently steps through speeds until frames arrive
    MODE_COUNT    = 4
};

static const char* const MODE_NAMES[MODE_COUNT] = {
    "LISTEN", "ACTIVE", "NO-ACK", "AUTO-SCAN"
};
static const char* const MODE_DESCS[MODE_COUNT] = {
    "Passive monitor — TX disabled",
    "Active node — ACK partner needed",
    "TX self-test — no ACK required",
    "Cycling speeds — searching for bus..."
};

// ─────────────────────────────────────────────────────────────
//  CAN bus speeds
//  twai_timing_config_t uses C99 designated-initializer macros.
//  Returned via a function so each initialisation is a standalone
//  local variable — guaranteed to compile with gnu++17.
// ─────────────────────────────────────────────────────────────
static const char* const SPEED_LABELS[] = { "125K", "250K", "500K", "1000K" };
static const int SPEED_COUNT = 4;

static twai_timing_config_t speedTiming(int idx)
{
    switch (idx) {
        case 0: { twai_timing_config_t t = TWAI_TIMING_CONFIG_125KBITS(); return t; }
        case 1: { twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); return t; }
        case 2: { twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS(); return t; }
        case 3: { twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();   return t; }
        default:{ twai_timing_config_t t = TWAI_TIMING_CONFIG_250KBITS(); return t; }
    }
}

// ─────────────────────────────────────────────────────────────
//  Application state
// ─────────────────────────────────────────────────────────────
static CanMode   canMode    = MODE_LISTEN;
static int       speedIdx   = 1;       // default 250 K
static bool      canRunning = false;
static uint32_t  txCount    = 0;
static uint32_t  rxCount    = 0;
static uint32_t  errCount   = 0;       // TX failures + TWAI bus errors

// ─────────────────────────────────────────────────────────────
//  Auto-scan state
// ─────────────────────────────────────────────────────────────
static const uint32_t SCAN_PERIOD_MS = 3000;  // time per speed before advancing
static uint32_t  scanStartMs   = 0;     // millis() when current scan step began
static bool      scanLocked    = false; // true once frames are received in AUTO-SCAN

// ─────────────────────────────────────────────────────────────
//  TWAI bus-state snapshot (updated by pollStatus())
// ─────────────────────────────────────────────────────────────
static twai_state_t  busState     = TWAI_STATE_STOPPED;
static uint32_t      busErrCount  = 0;   // hardware TX+RX error counter sum

// ─────────────────────────────────────────────────────────────
//  Frame ring-buffer — stores the last RING_CAP received frames.
//  Display shows at most 7 rows (11 px each) in the frame area.
// ─────────────────────────────────────────────────────────────
struct FrameEntry {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    bool     extd;
    bool     rtr;
};

static const int RING_CAP  = 7;
static FrameEntry ring[RING_CAP];
static uint32_t   ringTotal = 0;

static void ringPush(const twai_message_t& m)
{
    FrameEntry& e = ring[ringTotal % RING_CAP];
    e.id   = m.identifier;
    e.dlc  = m.data_length_code;
    e.extd = (m.extd  != 0);
    e.rtr  = (m.rtr   != 0);
    uint8_t len = (m.data_length_code <= 8) ? m.data_length_code : 8;
    memcpy(e.data, m.data, len);
    ringTotal++;
}

// ─────────────────────────────────────────────────────────────
//  TWAI (CAN) driver lifecycle
// ─────────────────────────────────────────────────────────────
static void canStop()
{
    if (!canRunning) return;
    twai_stop();
    twai_driver_uninstall();
    canRunning   = false;
    busState     = TWAI_STATE_STOPPED;
}

static void canStart()
{
    canStop();

    // AUTO-SCAN always uses listen-only so it never disturbs the bus
    twai_mode_t hwMode;
    switch (canMode) {
        case MODE_LISTEN:
        case MODE_AUTOSCAN: hwMode = TWAI_MODE_LISTEN_ONLY; break;
        case MODE_NOACK:    hwMode = TWAI_MODE_NO_ACK;      break;
        default:            hwMode = TWAI_MODE_NORMAL;      break;
    }

    twai_general_config_t gCfg =
        TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, hwMode);
    gCfg.rx_queue_len = 20;

    const twai_timing_config_t tCfg = speedTiming(speedIdx);
    const twai_filter_config_t fCfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&gCfg, &tCfg, &fCfg) != ESP_OK) {
        errCount++;
        return;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        errCount++;
        return;
    }
    canRunning   = true;
    scanStartMs  = (uint32_t)millis();
}

// ─────────────────────────────────────────────────────────────
//  TWAI status poll — read hardware error counters and bus state
// ─────────────────────────────────────────────────────────────
static void pollStatus()
{
    if (!canRunning) return;
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) return;

    busState    = info.state;
    busErrCount = info.tx_error_counter + info.rx_error_counter;

    // If the controller went bus-off, the speed is almost certainly wrong.
    // Recover automatically so the scan can continue.
    if (info.state == TWAI_STATE_BUS_OFF) {
        twai_initiate_recovery();
        errCount++;
    }
}

// ─────────────────────────────────────────────────────────────
//  RX — drain the TWAI receive queue every loop tick
// ─────────────────────────────────────────────────────────────
static void pollRx()
{
    if (!canRunning) return;
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        rxCount++;
        ringPush(msg);
    }
}

// ─────────────────────────────────────────────────────────────
//  TX — send one test frame per second in ACTIVE / NO-ACK mode
// ─────────────────────────────────────────────────────────────
static uint32_t lastTxMs = 0;
static uint8_t  txSeq    = 0;

static void pollTx()
{
    if (!canRunning || canMode == MODE_LISTEN || canMode == MODE_AUTOSCAN) return;
    uint32_t now = (uint32_t)millis();
    if (now - lastTxMs < 1000u) return;
    lastTxMs = now;

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = 0x7FF;
    msg.data_length_code = 4;
    msg.data[0]          = 0xCA;
    msg.data[1]          = 0xFE;
    msg.data[2]          = txSeq++;
    msg.data[3]          = 0xAA;

    if (twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK) {
        txCount++;
    } else {
        errCount++;
    }
}

// ─────────────────────────────────────────────────────────────
//  Auto-scan — advance to the next speed every SCAN_PERIOD_MS
//  if no frames have been received yet at the current speed.
//  Once frames arrive the scan locks and stops changing speed.
// ─────────────────────────────────────────────────────────────
static void pollAutoScan()
{
    if (canMode != MODE_AUTOSCAN || scanLocked) return;

    // Frames received → lock onto this speed
    if (rxCount > 0) {
        scanLocked = true;
        return;
    }

    uint32_t now     = (uint32_t)millis();
    uint32_t elapsed = now - scanStartMs;
    if (elapsed < SCAN_PERIOD_MS) return;

    // Move to next speed and restart the driver
    speedIdx    = (speedIdx + 1) % SPEED_COUNT;
    txCount     = rxCount = 0;          // reset per-speed counters (keep errCount)
    ringTotal   = 0;
    memset(ring, 0, sizeof(ring));
    lastTxMs    = 0;
    txSeq       = 0;
    canStart();                         // canStart() resets scanStartMs
}

// ─────────────────────────────────────────────────────────────
//  Reset statistics and frame history
// ─────────────────────────────────────────────────────────────
static void resetStats()
{
    txCount = rxCount = errCount = 0;
    busErrCount = 0;
    ringTotal   = 0;
    memset(ring, 0, sizeof(ring));
    lastTxMs    = 0;
    txSeq       = 0;
    scanLocked  = false;
    scanStartMs = 0;
}

// ─────────────────────────────────────────────────────────────
//  Screen rendering — double-buffered via M5Canvas sprite
// ─────────────────────────────────────────────────────────────

static inline uint16_t rgb(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | (b >> 3));
}

static void drawScreen()
{
    canvas.fillSprite(TFT_BLACK);
    canvas.setTextSize(1);
    canvas.setTextWrap(false);

    // ── Header (y 0–12) ─────────────────────────────────────
    const uint16_t HDR_BG = rgb(0, 90, 0);
    canvas.fillRect(0, 0, SCR_W, 13, HDR_BG);
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(3, 3);
    canvas.print("CAN BUS TESTER");

    // Speed badge — flashes amber while scanning, teal when idle/locked
    bool scanning = (canMode == MODE_AUTOSCAN && !scanLocked);
    uint16_t spdBg = scanning ? rgb(140, 80, 0) : rgb(0, 60, 80);
    canvas.fillRect(SCR_W - 55, 1, 54, 11, spdBg);
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(SCR_W - 53, 3);
    canvas.print(SPEED_LABELS[speedIdx]);

    // Mode badge
    uint16_t modeBg;
    if      (canMode == MODE_LISTEN)   modeBg = rgb(0, 0, 100);
    else if (canMode == MODE_ACTIVE)   modeBg = rgb(0, 80, 30);
    else if (canMode == MODE_NOACK)    modeBg = rgb(130, 60, 0);
    else /* AUTOSCAN */                modeBg = scanLocked ? rgb(0, 100, 0)
                                                           : rgb(100, 40, 0);
    canvas.fillRect(SCR_W - 110, 1, 53, 11, modeBg);
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(SCR_W - 108, 3);
    canvas.print(MODE_NAMES[canMode]);

    // ── Status line (y 14–24) ────────────────────────────────
    canvas.setCursor(3, 15);

    if (!canRunning) {
        canvas.setTextColor(TFT_RED);
        canvas.print("STATUS: DRIVER INIT FAILED — check wiring");

    } else if (canMode == MODE_AUTOSCAN && !scanLocked) {
        // Show which speed we're testing and how much time is left
        uint32_t elapsed = (uint32_t)millis() - scanStartMs;
        uint32_t remain  = (elapsed < SCAN_PERIOD_MS)
                           ? (SCAN_PERIOD_MS - elapsed) / 1000u + 1u : 1u;
        canvas.setTextColor(TFT_YELLOW);
        canvas.printf("SCANNING %s...  next in %us",
                      SPEED_LABELS[speedIdx], (unsigned)remain);

    } else if (canMode == MODE_AUTOSCAN && scanLocked) {
        canvas.setTextColor(TFT_GREEN);
        uint32_t lastIdx = (ringTotal - 1) % RING_CAP;
        canvas.printf("LOCKED %s  PASS  last ID: 0x%03X",
                      SPEED_LABELS[speedIdx],
                      (unsigned)ring[lastIdx].id);

    } else if (rxCount > 0) {
        canvas.setTextColor(TFT_GREEN);
        uint32_t lastIdx = (ringTotal - 1) % RING_CAP;
        canvas.printf("PASS  communicating  last ID: 0x%03X",
                      (unsigned)ring[lastIdx].id);

    } else if (busState == TWAI_STATE_BUS_OFF) {
        canvas.setTextColor(TFT_RED);
        canvas.print("BUS-OFF — wrong speed or bad wiring");

    } else if (busErrCount > 10) {
        canvas.setTextColor(TFT_ORANGE);
        canvas.printf("WARN  high errors (%u) — check speed/term",
                      (unsigned)busErrCount);

    } else if (errCount > 0) {
        canvas.setTextColor(TFT_ORANGE);
        canvas.print("WARN  errors — check speed / termination");

    } else if (txCount > 0) {
        canvas.setTextColor(TFT_YELLOW);
        canvas.printf("WAIT  TX ok (%u sent)  no RX yet", (unsigned)txCount);

    } else {
        canvas.setTextColor(TFT_YELLOW);
        canvas.print("WAIT  listening for CAN frames...");
    }

    // ── Counters (y 26–35) ──────────────────────────────────
    canvas.setCursor(3, 27);
    canvas.setTextColor(TFT_YELLOW);
    canvas.printf("TX:%-5u", (unsigned)txCount);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("  RX:%-5u", (unsigned)rxCount);
    uint32_t totalErr = errCount + busErrCount;
    canvas.setTextColor(totalErr ? TFT_RED : TFT_DARKGREY);
    canvas.printf("  ERR:%-4u", (unsigned)totalErr);

    // ── Horizontal divider ──────────────────────────────────
    canvas.drawLine(0, 37, SCR_W - 1, 37, TFT_DARKGREY);

    // ── Frame list (y 39 … SCR_H - 13) ─────────────────────
    uint32_t dispCount = (ringTotal < (uint32_t)RING_CAP)
                         ? ringTotal : (uint32_t)RING_CAP;

    if (dispCount == 0) {
        canvas.setTextColor(TFT_DARKGREY);
        canvas.setCursor(3, 42);
        canvas.print(MODE_DESCS[canMode]);
    } else {
        uint32_t oldest = (ringTotal > (uint32_t)RING_CAP)
                          ? (ringTotal - (uint32_t)RING_CAP) : 0u;

        for (uint32_t i = 0; i < dispCount; i++) {
            int y = 39 + (int)i * 11;
            if (y + 8 > SCR_H - 13) break;

            uint32_t   bufIdx = (oldest + i) % (uint32_t)RING_CAP;
            FrameEntry& f     = ring[bufIdx];

            canvas.setTextColor(f.extd ? TFT_CYAN : TFT_GREEN);
            canvas.setCursor(3, y);
            if (f.extd) {
                canvas.printf("0x%08X[%u]:", (unsigned)f.id, (unsigned)f.dlc);
            } else {
                canvas.printf("0x%03X[%u]:", (unsigned)f.id, (unsigned)f.dlc);
            }

            if (f.rtr) {
                canvas.setTextColor(TFT_YELLOW);
                canvas.print(" RTR");
            } else {
                canvas.setTextColor(TFT_WHITE);
                uint8_t len = (f.dlc <= 8) ? f.dlc : 8;
                for (int b = 0; b < (int)len; b++) {
                    canvas.printf("%02X ", (unsigned)f.data[b]);
                }
            }
        }
    }

    // ── Footer (y SCR_H-12 … SCR_H) ─────────────────────────
    canvas.fillRect(0, SCR_H - 12, SCR_W, 12, rgb(0, 0, 40));
    canvas.setTextColor(TFT_LIGHTGREY);
    canvas.setCursor(3, SCR_H - 10);
    if (canMode == MODE_AUTOSCAN && !scanLocked) {
        canvas.printf("[A] Skip speed  [B] Mode  scanned:%d/%d",
                      speedIdx + 1, SPEED_COUNT);
    } else {
        canvas.printf("[A] Speed  [B] Mode  total RX: %u", (unsigned)rxCount);
    }

    canvas.pushSprite(0, 0);
}

// ─────────────────────────────────────────────────────────────
//  Arduino entry points
// ─────────────────────────────────────────────────────────────
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);

    // Landscape: 240 wide × 135 tall, USB-C port on the right
    M5.Display.setRotation(3);

    canvas.setColorDepth(16);
    canvas.createSprite(SCR_W, SCR_H);
    canvas.setTextWrap(false);

    canStart();
    drawScreen();
}

void loop()
{
    M5.update();

    // Button A — cycle speed manually (also skips current speed in AUTO-SCAN)
    if (M5.BtnA.wasPressed()) {
        speedIdx = (speedIdx + 1) % SPEED_COUNT;
        resetStats();
        canStart();
    }

    // Button B — cycle mode
    if (M5.BtnB.wasPressed()) {
        canMode = static_cast<CanMode>((static_cast<int>(canMode) + 1) % MODE_COUNT);
        resetStats();
        canStart();
    }

    pollStatus();      // read TWAI hardware error counters and bus state
    pollRx();          // drain incoming CAN frames
    pollTx();          // send test frame if due (ACTIVE / NO-ACK only)
    pollAutoScan();    // advance speed if scanning and timeout elapsed

    drawScreen();
    delay(10);
}
