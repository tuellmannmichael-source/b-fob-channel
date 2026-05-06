# System Architecture

## Scope and Target Baseline

This repository targets **nRF54L15** with **nRF Connect SDK (NCS) v2.9.x**
(Zephyr 3.7 baseline) as stated in the project prerequisites. The feature
statements below are intentionally limited to what is realistically available in
that stack for application development.

## Channel Sounding Reality Check (nRF54L15 + NCS v2.9.x)

For this project baseline, treat BLE Channel Sounding (CS) as **not available as
an application-ready feature** on nRF54L15:

- **Roles**: no stable, documented Zephyr/NCS application API to run CS in
  initiator/reflector roles in this codebase baseline.
- **API maturity**: effectively **experimental / not production-ready** from the
  perspective of an app team building on standard Zephyr Bluetooth APIs.
- **Practical implication**: the ranging implementation must use
  connection-based RSSI (and optionally Direction Finding / CTE later), not BLE
  CS procedures.
- **Known limitation**: CS-related controller capabilities may exist in silicon
  evolution, but without stable host/controller integration in the selected
  SDK, we cannot depend on them for deliverables.

## Target Operating Mode (Project)

### Initial mode (MVP)

- **Topology**: **1 Receiver + 1 Keyfob**
- **Method**: BLE connection-based P2P ranging from RSSI trend / filtered link
  metrics
- **Objective**: robust proximity + coarse direction for vehicle use-cases

### Later optional expansion

- **Multi-anchor**: >1 receiver node sharing measurements
- **Multi-antenna**: phased or switched-antenna upgrades (AoA/CTE path)
- **Condition**: only after the P2P mode is stable and validated in field tests

## Capability Matrix (current baseline)

| Capability | Initial (supported now) | Later (optional) | Notes / constraints |
|---|---|---|---|
| PHY / link mode | LE 1M connection | LE 2M or coded PHY (if link budget / latency needs change) | Start with LE 1M for interoperability and stable RSSI cadence |
| Ranging method | RSSI-based P2P | RSSI + multi-anchor fusion; optional AoA/CTE | BLE Channel Sounding not assumed available in NCS v2.9.x app flow |
| Node roles | 1 central receiver + 1 peripheral keyfob | 1 keyfob + N receivers | Current firmware defaults to central/peripheral split |
| Expected measurement rate | ~10 Hz raw RSSI samples at 100 ms interval | 5–20 Hz depending on CI / scheduling | Effective filtered output typically slower than raw sample cadence |
| Typical range accuracy | ~1–3 m (open/LOS), worse in multipath | ~0.5–2 m with multi-anchor fusion / calibration | RSSI-only is environment-sensitive; calibration is mandatory |
| Known failure modes | Multipath fading, human body shadowing, antenna detuning, co-channel BLE/Wi‑Fi interference, connection drops | Same + anchor clock/sync issues in multi-node mode | Require filtering, hysteresis, reconnect logic, and outlier rejection |

## Overview

A single aftermarket device installed in the delivery vehicle contains:

- **nRF54L15** MCU running BLE central + position engine
- **Skyworks SKYA21039** SP3T RF switch routing the nRF54L15's single radio to
  one of three external antennas
- **Three roof-mounted 2.4 GHz antennas** connected via UFL pigtail + 2m SMA
  cable

The device connects to (or scans for) the BLE keyfob, rapidly cycles the RF
switch between antennas, reads RSSI on each, and computes proximity + direction.

## Block Diagram

```
┌──────────────────────────────────────────────────────────┐
│                  Aftermarket Receiver                     │
│                                                          │
│  ┌───────────┐    RF     ┌────────────┐    SMA cables   │
│  │           │◄─────────►│ SKYA21039  │──► Ant A (front) │
│  │  nRF54L15 │  single   │  SP3T RF   │──► Ant B (L-rear)│
│  │           │  antenna  │  switch    │──► Ant C (R-rear)│
│  │  - BLE    │  port     └─────┬──────┘                  │
│  │  - Kalman │            GPIO │ V1, V2                  │
│  │  - Prox/  │◄────────────────┘                         │
│  │    Dir    │                                           │
│  │  engine   │    SPI    ┌────────────┐                  │
│  │           │◄─────────►│ MCP2518FD  │──► CAN bus       │
│  │           │    SPI    ┌────────────┐                  │
│  │           │◄─────────►│ W25Q128    │   (flash)        │
│  └───────────┘           └────────────┘                  │
│                                                          │
└──────────────────────────────────────────────────────────┘

         ~~~~ BLE 2.4 GHz ~~~~

┌──────────────┐
│   Keyfob     │
│  nRF54L15    │
│  (peripheral)│
└──────────────┘
```

## RF Switch: SKYA21039

The Skyworks SKYA21039 is a **single-pole, three-throw (SP3T)** absorptive
switch designed for 0.1-6.0 GHz. At 2.4 GHz:

| Parameter | Value |
|-----------|-------|
| Insertion loss | 0.45 dB typical |
| Isolation | 28 dB typical |
| Switching speed | < 0.5 us |
| Control | 2 GPIO pins (V1, V2) |
| Supply | 1.8 V (compatible with nRF54L15 GPIO) |

### Control truth table

| V1 | V2 | Active port |
|----|----|-------------|
| 0  | 0  | RFC to RF1 (Antenna A) |
| 1  | 0  | RFC to RF2 (Antenna B) |
| 0  | 1  | RFC to RF3 (Antenna C) |
| 1  | 1  | All ports off (isolation) |

Switching takes < 0.5 us. The RSSI settling time after a switch is dominated
by the BLE radio's AGC settling (~10-50 us), not the switch itself.

## Antenna Switching Strategy

### Sequential RSSI cycling (MVP)

```
Time ──────────────────────────────────────────────────►

 BLE event N        BLE event N+1      BLE event N+2
┌────────────┐    ┌────────────┐    ┌────────────┐
│ Ant A      │    │ Ant B      │    │ Ant C      │
│ read RSSI  │    │ read RSSI  │    │ read RSSI  │
└────────────┘    └────────────┘    └────────────┘
      │                 │                 │
      ▼                 ▼                 ▼
  filter[A].update  filter[B].update  filter[C].update
                                          │
                                          ▼
                                   compute proximity
                                   + direction
```

With a 100 ms connection interval, one full 3-antenna cycle takes 300 ms
(~3.3 Hz position update rate). This is sufficient for walking-speed tracking.

### Faster option: intra-event switching

If the connection interval is long enough, switch antennas multiple times
within a single connection event. The BLE packet (including preamble, header,
payload) is ~1 ms on LE 1M PHY. Between packets in the same event, the switch
can toggle and re-read RSSI. This requires hooking into the radio ISR.

### Future: CTE antenna switching

For Direction Finding (AoA), the nRF54L15 radio can be configured to switch
antennas during the Constant Tone Extension (CTE) appended to a BLE packet.
The SKYA21039's < 0.5 us switching speed is fast enough for the 1-2 us CTE
sample slots. The same V1/V2 GPIO pins would be driven by the radio's
antenna-switching pattern instead of software.

## Antenna Placement (Roof-Mounted)

```
          Vehicle (top-down view)
    ┌──────────────────────────────────┐
    │                                  │
    │           * Ant A                │  Front
    │          (0.0, 4.5)              │
    │                                  │
    │                                  │
    │                                  │
    │  * Ant B              Ant C *    │  Rear
    │ (-0.9, 0.0)        (0.9, 0.0)   │
    │                                  │
    └──────────────────────────────────┘

    Coordinate system: origin = center of rear axle
    X = right, Y = forward
    Units: meters
```

The triangle formed by the three antennas has:
- Base (B-C): 1.8 m
- Height (A to B-C midpoint): 4.5 m
- This gives good GDOP (geometric dilution of precision) for discriminating
  front vs rear and left vs right.

## SMA Cable Loss Budget

| Segment | Loss |
|---------|------|
| UFL pigtail (~15 cm) | ~0.3 dB |
| SMA cable (2 m, RG316) | ~1.5 dB at 2.4 GHz |
| SMA connectors (x2) | ~0.2 dB |
| RF switch insertion | ~0.5 dB |
| **Total per path** | **~2.5 dB** |

This loss is constant and identical for all paths (same cable lengths), so it
cancels out in RSSI comparison. It does reduce absolute sensitivity by ~2.5 dB,
reducing maximum detection range. For a vehicle-proximity use case (0-15 m)
this is acceptable.

**Important:** If cable lengths differ between antennas, the RSSI offset must
be calibrated per-antenna.

## Data Flow

```
┌─────────┐     BLE conn/adv      ┌───────────────────────────────────┐
│ Keyfob  │ ─────────────────────► │ nRF54L15 Radio                   │
│nRF54L15 │                        │                                   │
└─────────┘                        │  1. Receive packet on current ant │
                                   │  2. Read RSSI from radio register │
                                   │  3. Switch RF switch to next ant  │
                                   └──────────────┬────────────────────┘
                                                  │
                                   ┌──────────────▼────────────────────┐
                                   │ Kalman Filter (per antenna)       │
                                   │  rssi_filtered = KF.update(rssi)  │
                                   └──────────────┬────────────────────┘
                                                  │
                                   ┌──────────────▼────────────────────┐
                                   │ Position Engine                   │
                                   │  - RSSI to distance (per antenna) │
                                   │  - Direction: strongest antenna   │
                                   │    + RSSI ratio interpolation     │
                                   │  - Proximity: weighted avg dist   │
                                   │  - Optional: trilateration (2D)   │
                                   └──────────────┬────────────────────┘
                                                  │
                                   ┌──────────────▼────────────────────┐
                                   │ Output                            │
                                   │  - CAN bus (vehicle integration)  │
                                   │  - UART log                       │
                                   │  - Flash log                      │
                                   └───────────────────────────────────┘
```

## Power Budget

| Component | Active current | Notes |
|-----------|---------------|-------|
| nRF54L15 (BLE active) | ~5 mA | BLE central, 100 ms interval |
| SKYA21039 | ~10 uA | Negligible |
| MCP2518FD (CAN) | ~15 mA | Only when transmitting |
| External flash | ~15 mA active, 1 uA standby | SPI access |
| **Total (typical)** | **~20-35 mA** | Powered by vehicle 12V via LDO |

Keyfob (nRF54L15 peripheral):
- Advertising every 100 ms: ~5 mA average, CR2032 lasts ~3-6 months
- Connected, 100 ms interval: ~3-5 mA average, similar battery life
