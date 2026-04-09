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
 *  ┌───────────────┬─────────────────────────────────────┐
 *  │ LISTEN        │ TWAI_MODE_LISTEN_ONLY               │
 *  │               │ Silent monitor — never transmits.   │
 *  │               │ Use to observe an existing bus.     │
 *  ├───────────────┼─────────────────────────────────────┤
 *  │ ACTIVE        │ TWAI_MODE_NORMAL                    │
 *  │               │ Sends a test frame every 1 s and    │
 *  │               │ receives frames from the bus.       │
 *  │               │ Requires ≥1 other node for ACK.     │
 *  ├───────────────┼─────────────────────────────────────┤
 *  │ NO-ACK        │ TWAI_MODE_NO_ACK                    │
 *  │               │ Sends frames without needing ACK.   │
 *  │               │ Tests the TX path in isolation.     │
 *  └───────────────┴─────────────────────────────────────┘
 *
 *  DISPLAY LAYOUT  (landscape 240 × 135)
 *  ┌────────────────────────────────────────────────────┐
 *  │  CAN BUS TESTER          [MODE]          [SPEED]   │
 *  │  STATUS: …                                         │
 *  │  TX: 0       RX: 0       ERR: 0                    │
 *  │  ────────────────────────────────────────────────  │
 *  │  0x001[8]: CA FE 01 AA 00 00 00 00                 │
 *  │  0x002[4]: 11 22 33 44                             │
 *  │  …  (up to 7 most-recent frames)                   │
 *  │  [A] Speed   [B] Mode   total RX: 0                │
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
    MODE_COUNT    = 3
};

static const char* const MODE_NAMES[MODE_COUNT] = {
    "LISTEN", "ACTIVE", "NO-ACK"
};
static const char* const MODE_DESCS[MODE_COUNT] = {
    "Passive monitor — TX disabled",
    "Active node — ACK partner needed",
    "TX self-test — no ACK required"
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
static uint32_t  errCount   = 0;

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

static const int RING_CAP  = 7;          // max frames stored / visible
static FrameEntry ring[RING_CAP];
static uint32_t   ringTotal = 0;         // total frames ever pushed

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
    canRunning = false;
}

static void canStart()
{
    canStop();

    twai_mode_t hwMode;
    switch (canMode) {
        case MODE_LISTEN: hwMode = TWAI_MODE_LISTEN_ONLY; break;
        case MODE_NOACK:  hwMode = TWAI_MODE_NO_ACK;      break;
        default:          hwMode = TWAI_MODE_NORMAL;       break;
    }

    twai_general_config_t gCfg =
        TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, hwMode);
    gCfg.rx_queue_len = 20;   // deeper buffer so fast bursts aren't dropped

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
    canRunning = true;
}

// ─────────────────────────────────────────────────────────────
//  RX — drain the TWAI receive queue every loop tick
// ─────────────────────────────────────────────────────────────
static void pollRx()
{
    if (!canRunning) return;
    twai_message_t msg;
    // Timeout 0: return immediately if queue is empty (non-blocking)
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
    if (!canRunning || canMode == MODE_LISTEN) return;
    uint32_t now = (uint32_t)millis();
    if (now - lastTxMs < 1000u) return;
    lastTxMs = now;

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.identifier       = 0x7FF;   // max standard 11-bit ID
    msg.data_length_code = 4;
    msg.data[0]          = 0xCA;
    msg.data[1]          = 0xFE;
    msg.data[2]          = txSeq++;
    msg.data[3]          = 0xAA;

    // 50 ms transmit timeout (enough for low-traffic buses at 125 K)
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(50));
    if (ret == ESP_OK) {
        txCount++;
    } else {
        errCount++;
    }
}

// ─────────────────────────────────────────────────────────────
//  Reset statistics and frame history
// ─────────────────────────────────────────────────────────────
static void resetStats()
{
    txCount = rxCount = errCount = 0;
    ringTotal = 0;
    memset(ring, 0, sizeof(ring));
    lastTxMs = 0;
    txSeq    = 0;
}

// ─────────────────────────────────────────────────────────────
//  Screen rendering (double-buffered via M5Canvas sprite)
// ─────────────────────────────────────────────────────────────

// Build a compact RGB-565 colour from 8-bit components
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

    // Speed badge (right edge of header)
    canvas.fillRect(SCR_W - 55, 1, 54, 11, rgb(0, 60, 80));
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(SCR_W - 53, 3);
    canvas.print(SPEED_LABELS[speedIdx]);

    // Mode badge (left of speed badge)
    uint16_t modeBg = (canMode == MODE_LISTEN) ? rgb(0, 0, 100)  :
                      (canMode == MODE_ACTIVE) ? rgb(0, 80, 30)  :
                                                 rgb(130, 60, 0) ;
    canvas.fillRect(SCR_W - 110, 1, 53, 11, modeBg);
    canvas.setTextColor(TFT_WHITE);
    canvas.setCursor(SCR_W - 108, 3);
    canvas.print(MODE_NAMES[canMode]);

    // ── Status line (y 14–24) ────────────────────────────────
    canvas.setCursor(3, 15);
    if (!canRunning) {
        // Driver failed to start — probably a GPIO conflict
        canvas.setTextColor(TFT_RED);
        canvas.print("STATUS: DRIVER INIT FAILED — check wiring");
    } else if (rxCount > 0) {
        // At least one frame received → bus is communicating
        canvas.setTextColor(TFT_GREEN);
        uint32_t lastIdx = (ringTotal - 1) % RING_CAP;
        canvas.printf("PASS  communicating  last ID: 0x%03X",
                      (unsigned)ring[lastIdx].id);
    } else if (errCount > 0) {
        // Errors without reception → likely wrong speed or no termination
        canvas.setTextColor(TFT_ORANGE);
        canvas.print("WARN  errors — check speed / termination");
    } else if (txCount > 0) {
        // Frames sent, nothing received yet
        canvas.setTextColor(TFT_YELLOW);
        canvas.printf("WAIT  TX ok (%u sent)  no RX yet", (unsigned)txCount);
    } else {
        // Idle — waiting for any bus activity
        canvas.setTextColor(TFT_YELLOW);
        canvas.print("WAIT  listening for CAN frames...");
    }

    // ── Counters (y 26–35) ──────────────────────────────────
    canvas.setCursor(3, 27);
    canvas.setTextColor(TFT_YELLOW);
    canvas.printf("TX:%-5u", (unsigned)txCount);
    canvas.setTextColor(TFT_CYAN);
    canvas.printf("  RX:%-5u", (unsigned)rxCount);
    canvas.setTextColor(errCount ? TFT_RED : TFT_DARKGREY);
    canvas.printf("  ERR:%-4u", (unsigned)errCount);

    // ── Horizontal divider ──────────────────────────────────
    canvas.drawLine(0, 37, SCR_W - 1, 37, TFT_DARKGREY);

    // ── Frame list (y 39 … SCR_H - 13) ─────────────────────
    uint32_t dispCount = (ringTotal < (uint32_t)RING_CAP)
                         ? ringTotal : (uint32_t)RING_CAP;

    if (dispCount == 0) {
        // Nothing received yet: show the mode hint
        canvas.setTextColor(TFT_DARKGREY);
        canvas.setCursor(3, 42);
        canvas.print(MODE_DESCS[canMode]);
    } else {
        // Display frames oldest → newest (top → bottom)
        uint32_t oldest = (ringTotal > (uint32_t)RING_CAP)
                          ? (ringTotal - (uint32_t)RING_CAP) : 0u;

        for (uint32_t i = 0; i < dispCount; i++) {
            int y = 39 + (int)i * 11;
            // Stop before we overlap the footer
            if (y + 8 > SCR_H - 13) break;

            uint32_t  bufIdx = (oldest + i) % (uint32_t)RING_CAP;
            FrameEntry& f    = ring[bufIdx];

            // Frame ID  (green = standard 11-bit, cyan = extended 29-bit)
            canvas.setTextColor(f.extd ? TFT_CYAN : TFT_GREEN);
            canvas.setCursor(3, y);
            if (f.extd) {
                canvas.printf("0x%08X[%u]:", (unsigned)f.id, (unsigned)f.dlc);
            } else {
                canvas.printf("0x%03X[%u]:", (unsigned)f.id, (unsigned)f.dlc);
            }

            // Frame payload
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
    canvas.printf("[A] Speed  [B] Mode  total RX: %u", (unsigned)rxCount);

    // Push the completed sprite to the physical display in one blit
    canvas.pushSprite(0, 0);
}

// ─────────────────────────────────────────────────────────────
//  Arduino entry points
// ─────────────────────────────────────────────────────────────
void setup()
{
    // M5Unified auto-detects the hardware and configures display,
    // buttons, IMU, RTC, and power management.
    auto cfg = M5.config();
    M5.begin(cfg);

    // Landscape mode: 240 wide × 135 tall.
    // Rotation 3 = landscape with USB-C port on the right.
    M5.Display.setRotation(3);

    // Create the off-screen sprite (16-bit colour, same size as display)
    canvas.setColorDepth(16);
    canvas.createSprite(SCR_W, SCR_H);
    canvas.setTextWrap(false);

    // Start the TWAI driver in the default mode (LISTEN, 250 K)
    canStart();

    // Initial render so the display is not blank while waiting for frames
    drawScreen();
}

void loop()
{
    // Update button states (call once per loop, before reading buttons)
    M5.update();

    // Button A — cycle through CAN speeds
    if (M5.BtnA.wasPressed()) {
        speedIdx = (speedIdx + 1) % SPEED_COUNT;
        resetStats();
        canStart();
    }

    // Button B — cycle through CAN modes
    if (M5.BtnB.wasPressed()) {
        canMode = static_cast<CanMode>((static_cast<int>(canMode) + 1) % MODE_COUNT);
        resetStats();
        canStart();
    }

    pollRx();    // drain incoming CAN frames
    pollTx();    // send a test frame if due (ACTIVE / NO-ACK modes)
    drawScreen();

    // 10 ms sleep keeps CPU utilisation low without noticeable lag
    delay(10);
}
