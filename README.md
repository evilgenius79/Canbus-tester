# CAN Bus Tester

A portable CAN bus diagnostic tool built on the M5StickC PLUS2.  
Connect to any vehicle or embedded CAN network to verify communication, identify bus speed, and monitor live traffic.

---

## Hardware Required

| Item | Details |
|------|---------|
| [M5StickC PLUS2](https://docs.m5stack.com/en/core/M5StickC%20PLUS2) | ESP32-PICO-V3-02, 1.14" 240×135 TFT, 200 mAh |
| [M5Stack CAN Unit (CA-IS3050G)](https://docs.m5stack.com/en/unit/can) | Isolated CAN transceiver, Grove connector |
| Grove HY2.0 cable | Included with the CAN Unit |

---

## Wiring

Connect the CAN Unit to the M5StickC PLUS2's Grove port using the included cable.  
No additional components are needed — the Grove cable carries power and signals.

```
M5StickC PLUS2 Grove Port
┌───────────────────────────────────────────────────┐
│  Pin 1  yellow ──► GPIO32 ──► TWAI RX (from bus)  │
│  Pin 2  white  ──► GPIO33 ──► TWAI TX (to bus)    │
│  Pin 3  red    ──► 5 V                             │
│  Pin 4  black  ──► GND                             │
└───────────────────────────────────────────────────┘
```

### CAN Bus Termination

CAN bus requires a **120 Ω resistor between CANH and CANL at each physical end** of the bus.

- The CAN Unit has a built-in termination resistor that can be enabled via a **solder jumper** on the PCB.
- Enable it when the tester is at the end of the bus.
- If connecting to a vehicle's OBD-II port, the vehicle's ECU network is already terminated — leave the CAN Unit's jumper **open**.

```
  [Node A]──────────────[CAN Unit]──────────────[Node B]
  120 Ω ↑                                        ↑ 120 Ω
  (terminator)                              (terminator)
```

---

## Build & Flash

This project uses [PlatformIO](https://platformio.org/).

```bash
# Install PlatformIO CLI or use the VS Code extension, then:
git clone <repo-url>
cd Canbus-tester

# Build
pio run

# Flash (connect M5StickC PLUS2 via USB-C)
pio run --target upload

# Serial monitor (optional)
pio device monitor
```

**Dependencies** (resolved automatically by PlatformIO):

| Library | Purpose |
|---------|---------|
| `m5stack/M5Unified` | Display, buttons, power management |
| `m5stack/M5GFX` | Graphics driver |

No external CAN library is needed — the ESP32's built-in **TWAI** hardware CAN controller is used directly via the ESP-IDF driver.

---

## Controls

| Button | Location | Action |
|--------|----------|--------|
| **Button A** | Front face (below screen) | Cycle CAN speed: 125K → 250K → 500K → 1000K |
| **Button B** | Right side (power button) | Cycle mode: LISTEN → ACTIVE → NO-ACK → AUTO-SCAN |
| **Button B** (hold ~2 s) | Right side | Power off |

In **AUTO-SCAN** mode, Button A skips the current speed immediately.

---

## Modes

### LISTEN
Uses `TWAI_MODE_LISTEN_ONLY` — the device **never transmits**.  
Safe to connect to any live bus without risking interference.  
Use this to verify bus activity from an existing network (vehicle, machine, etc.).

### ACTIVE
Uses `TWAI_MODE_NORMAL` — sends a test frame (`ID 0x7FF, data CA FE seq AA`) every second and receives frames from the bus.  
Requires at least one other CAN node on the bus to provide the ACK bit.  
Use this when testing communication between two devices.

### NO-ACK
Uses `TWAI_MODE_NO_ACK` — sends frames without waiting for acknowledgement.  
Useful for verifying the TX hardware path (CAN Unit wiring, transceiver power) without a second node.  
No loopback: you will not receive your own frames back.

### AUTO-SCAN
Silently listens at each speed in sequence, spending **3 seconds** per speed:

```
125K → 250K → 500K → 1000K → 125K → ...
```

As soon as frames are received the scan **locks** onto that speed and stops changing.  
The display shows a countdown, which speed is currently being tested, and how many speeds have been tried.  
Button A skips to the next speed immediately.

This is the recommended starting mode for an unknown vehicle or device.

---

## Display Layout

```
┌─────────────────────────────────────────────────────┐
│  CAN BUS TESTER              [AUTO-SCAN]    [250K]  │  ← header
│  SCANNING 250K...  next in 2s                       │  ← status
│  TX: 0       RX: 0           ERR: 0                 │  ← counters
│  ───────────────────────────────────────────────    │
│  0x7FF[4]: CA FE 01 AA                              │  \
│  0x1A0[8]: 00 10 00 00 00 00 80 F0                  │   up to
│  0x0C0[3]: FF 00 12                                 │   7 frames
│  ...                                                │  /
│  [A] Skip speed  [B] Mode  scanned: 2/4             │  ← footer
└─────────────────────────────────────────────────────┘
```

### Status Line

| Text | Meaning |
|------|---------|
| `PASS  communicating  last ID: 0x1A0` | Frames received — bus is working |
| `LOCKED 500K  PASS  last ID: 0x1A0` | AUTO-SCAN found the speed |
| `SCANNING 250K...  next in 2s` | AUTO-SCAN in progress |
| `WAIT  listening for CAN frames...` | No frames yet, no errors |
| `WAIT  TX ok (3 sent)  no RX yet` | Frames sent but nothing received |
| `WARN  errors — check speed / termination` | TX errors with no reception |
| `WARN  high errors (32) — check speed/term` | High TWAI error counter |
| `BUS-OFF — wrong speed or bad wiring` | Severe errors, auto-recovering |
| `STATUS: DRIVER INIT FAILED — check wiring` | TWAI driver could not start |

### Frame Display

```
0x1A0[8]: 10 00 00 00 00 00 80 F0
│   │  │   └─ payload bytes (hex)
│   │  └───── DLC (data length)
│   └──────── ID (green = standard 11-bit, cyan = extended 29-bit)
└──────────── RTR frames shown as "0x001[4]: RTR" in yellow
```

The 7 most recently received frames are shown, oldest at top, newest at bottom.  
The ring buffer overwrites the oldest frame once full.

### Error Counter

The `ERR:` field combines:
- TX failures from `twai_transmit()`
- Hardware TX + RX error counter from the TWAI controller

A non-zero count with no frames received almost always means the **bus speed is wrong** or **termination is missing**.

---

## Common CAN Bus Speeds by Vehicle / Protocol

| Speed | Common Use |
|-------|------------|
| 125 Kbps | Body / comfort ECUs, older J1939 |
| 250 Kbps | Agricultural (J1939), some OBDII, body buses |
| 500 Kbps | Most modern passenger car powertrain CAN (ISO 15765-4) |
| 1000 Kbps | CAN FD backbone buses, some high-speed chassis systems |

> **Note:** CAN FD variable-rate frames are not supported — the ESP32 TWAI controller is classical CAN only.  
> Extended IDs (29-bit, shown in cyan) are common in J1939 and UDS-based networks.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `DRIVER INIT FAILED` | GPIO conflict or hardware fault | Check Grove cable is fully seated; reflash |
| `BUS-OFF` repeatedly | Wrong speed, bad termination, or wiring short | Try AUTO-SCAN; check CANH/CANL aren't shorted |
| `WARN errors` but no frames | Speed mismatch | Cycle speeds with Button A or use AUTO-SCAN |
| No frames in LISTEN mode on live bus | Missing termination, wrong speed | Enable CAN Unit termination jumper; try AUTO-SCAN |
| Frames appear briefly then stop | Bus overloaded or RX queue overflow | Normal in heavy-traffic systems; all recent frames visible |
| ACTIVE mode always fails TX | No other node connected | Switch to NO-ACK mode to test TX path alone |

---

## Technical Notes

- The **CA-IS3050G** is a galvanically isolated CAN transceiver (1000 V isolation).  It converts the ESP32's TTL-level TWAI TX/RX signals to/from differential CANH/CANL.  No SPI, I2C, or UART protocol is involved — the ESP32's hardware TWAI controller handles the CAN framing.
- The display is double-buffered: all drawing happens on an off-screen `M5Canvas` sprite and is pushed to the LCD in one blit, preventing flicker.
- The TWAI RX queue depth is set to 20 frames to absorb bursts on busy networks without dropping frames.
- AUTO-SCAN uses `TWAI_MODE_LISTEN_ONLY` — it will never transmit, so it cannot disrupt a running network.
- BUS-OFF recovery is triggered automatically during AUTO-SCAN so a speed mismatch does not permanently stall the controller.
