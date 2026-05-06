<div align="center">

# b-fob-channel

**BLE keyfob proximity & direction around a delivery vehicle**

*nRF54L15 receiver · RF switch (Skyworks SKYA21039) · three roof antennas · hybrid Channel Sounding + RSSI*

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![nRF Connect SDK](https://img.shields.io/badge/nRF%20Connect%20SDK-2.9%2B-00ADD8?logo=nordicsemiconductor)](https://developer.nordicsemi.com/)

</div>

---

## What this is

Detect **proximity** and **direction** of a BLE keyfob relative to a vehicle using a **single** Nordic **nRF54L15** on the roof, an **SP3T RF switch** cycling three external antennas, and firmware that combines **Channel Sounding (CS)** ranging with **RSSI** fallback and triangulation-oriented processing.

---

## Interactive notebooks (HTML)

Static tools you can open locally after clone (double-click or serve the repo root, e.g. `python -m http.server 8080`):

| File | Purpose |
|------|---------|
| [`cs-estimator-visualization.html`](cs-estimator-visualization.html) | CS distance pipeline: tone model, phase slope vs. IFFT, fusion |
| [`cs-phase-unwrap-viz.html`](cs-phase-unwrap-viz.html) | Naive vs. **gap-/quality-aware** phase unwrap (matches `firmware/src/ble/cs_phase_unwrap.c`) |

---

## System overview

```
                 Top-down view of vehicle roof
    ┌──────────────────────────────────────────┐
    │              Ant A (front)                │
    │                  *                        │
    │                 / \                       │
    │  ┌────────┐   /   \                      │
    │  │nRF54L15│  / dir? \    BLE connection   │
    │  │ + RF   │ / dist?  \   to keyfob        │
    │  │ switch │/          \  Hybrid ranging: │
    │  └────────┘            \ CS + RSSI fbk   │
    │           *─────────────*                 │
    │       Ant B (left)   Ant C (right)        │
    │          rear           rear              │
    └──────────────────────────────────────────┘
```

---

## Repository layout

| Path | Contents |
|------|----------|
| [`firmware/`](firmware/) | Receiver firmware (Zephyr / nRF Connect SDK): BLE manager, CS adapter & estimator, triangulation |
| [`keyfob/`](keyfob/) | Peripheral / keyfob firmware (nRF54L15) |
| [`docs/`](docs/) | Architecture, math, BLE connection management, validation & rollout |
| [`simulation/`](simulation/) | Python models (`requirements.txt`, triangulation & RSSI helpers) |
| [`tests/`](tests/) | Native unit tests for filters / triangulation |

<details>
<summary>Tree (abbreviated)</summary>

```
.
├── cs-estimator-visualization.html
├── cs-phase-unwrap-viz.html
├── docs/
│   ├── architecture.md
│   ├── triangulation-math.md
│   ├── ble-connection-management.md
│   └── validation-rollout-plan.md
├── firmware/
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── Kconfig
│   └── src/
│       ├── main.c
│       ├── ble/
│       ├── triangulation/
│       └── config/
├── keyfob/
├── simulation/
└── tests/
```

</details>

---

## Quick start

### Prerequisites

- **nRF Connect SDK** v2.9+ (nRF54L15 support)
- Nordic CLI tools (`nrfjprog`, `nrfutil`)
- **Python 3.10+** (simulation)

### Build & flash — receiver

```bash
cd firmware
west build -b nrf54l15dk/nrf54l15/cpuapp
west flash
```

### Build & flash — keyfob

```bash
cd keyfob
west build -b nrf54l15dk/nrf54l15/cpuapp
west flash
```

### Simulation

```bash
cd simulation
pip install -r requirements.txt
python triangulation_sim.py
```

---

## Design highlights

1. **Single-device baseline** — One MCU + RF switch; structured for antenna rotation while keeping RSSI filtering and position logic modular.
2. **Proximity + direction** — RSSI ratios between antennas for direction sector; averaged RSSI for distance band (robust vs. full 2D trilateration where not needed).
3. **Connection-first with scan fallback** — Stable RSSI cadence; auto-reconnect with backoff.
4. **Hybrid ranging** — CS session lifecycle when enabled; explicit RSSI fallback path.
5. **Robust CS phase unwrap** — Predictive, gap- and quality-aware unwrap before phase-slope regression (see `cs_phase_unwrap.c` and the HTML viz above).
6. **Kalman smoothing per antenna** — Raw RSSI smoothed before fallback distance.
7. **CTE / AoA path** — RF switch timing can later tie into Nordic direction-finding patterns.

---

## Hardware (reference)

| Role | Part |
|------|------|
| Receiver MCU | nRF54L15 |
| RF switch | Skyworks SKYA21039 (SP3T) |
| Antennas | 3× external 2.4 GHz BLE + pigtails |
| CAN | MCP2518FD |
| External flash | Winbond W25Q128JVSI (128 Mbit) |
| Keyfob MCU | nRF54L15 |

---

## License

This project is licensed under the **MIT License** — see [`LICENSE`](LICENSE).
