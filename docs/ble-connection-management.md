# BLE Connection Management

How to maintain a reliable BLE link between the single nRF54L15 receiver
(with RF-switched antennas) and the nRF54L15 keyfob, and how to recover
gracefully when the link drops.

---

## Connection vs Scanning

| | Passive Scanning | Active Connection |
|---|---|---|
| How it works | Listen for advertisements | Establish a GATT connection |
| RSSI source | Advertisement RSSI | Connection event RSSI |
| Interval control | Determined by keyfob ad interval | Configurable connection interval |
| Reliability | May miss ads | Guaranteed data at each interval |
| Power (keyfob) | Lower (just advertising) | Higher (connection maintenance) |
| Latency | Variable | Deterministic |
| **Antenna switching** | **Must be on correct ant when ad arrives** | **Can switch per event** |

**Recommended approach:** Maintain an active connection for predictable, high-
rate RSSI sampling with controlled antenna switching. Fall back to passive
scanning when the connection is lost and auto-reconnect in the background.

### Why connection-based is better for RF switching

With a connection, the receiver knows exactly when the next packet arrives
(at the connection interval). It can pre-switch the antenna before the event.
With passive scanning, advertisements arrive unpredictably -- the receiver
might be on the wrong antenna when the ad arrives, wasting that sample.

---

## Connection Lifecycle

```
                    ┌─────────────┐
          ┌────────►│   SCANNING  │◄────────────────┐
          │         │  (fallback) │                  │
          │         └──────┬──────┘                  │
          │                │ Keyfob found            │
          │                ▼                         │
          │         ┌─────────────┐                  │
          │         │ CONNECTING  │                  │
          │         └──────┬──────┘                  │
          │                │ Connected               │
          │                ▼                         │
          │         ┌─────────────┐    Disconnect    │
          │         │  CONNECTED  │─────────────────►│
          │         │  (ant cycle)│                  │
          │         └──────┬──────┘                  │
          │                │ Supervision timeout     │
          │                ▼                         │
          │         ┌─────────────┐                  │
          └─────────│RECONNECTING │──────────────────┘
                    │ (backoff)   │  Max retries
                    └─────────────┘  exceeded
```

---

## Connection Parameters

### Recommended settings

```c
#define CONN_INTERVAL_MIN       80   /* 100 ms  (units of 1.25 ms) */
#define CONN_INTERVAL_MAX       80   /* 100 ms */
#define CONN_LATENCY            0    /* no skipped events -- we need every RSSI */
#define CONN_SUPERVISION_TIMEOUT 400 /* 4 seconds (units of 10 ms) */
```

**Why 100 ms?** It gives 10 RSSI samples/second across all antennas. With 3
antennas in round-robin, each antenna gets ~3.3 samples/second. This is enough
for walking-speed tracking.

**Why 0 latency?** Peripheral latency allows the keyfob to skip connection
events to save power. For RSSI tracking we need every event, so set to 0.

**Why 4 s supervision timeout?** If we miss 4 seconds of events the link is
declared dead. This is long enough to survive brief interference but short
enough to detect a genuine disconnect quickly.

### Keyfob advertising parameters (when not connected)

```c
#define ADV_INTERVAL_MIN   160   /* 100 ms (units of 0.625 ms) */
#define ADV_INTERVAL_MAX   160   /* 100 ms */
#define ADV_TYPE           BT_LE_ADV_CONN_NAME  /* connectable */
```

---

## CS-Betriebsmodus (festgelegt)

Für die Firmware wird **Connection Sampling (CS) pro Messzyklus mit rotierendem
RF-Switch** gefahren (A→B→C). Single-Antenna-CS bleibt als Kconfig-Fallback
enthalten, ist aber nicht der Default.

- Default: `CONFIG_BLE_CS_MODE_ROTATING=y`
- Fallback: `CONFIG_BLE_CS_MODE_SINGLE=y`

Ziel: Jede Antenne erhält unter gleichen Link-Bedingungen regelmäßig Samples,
so dass Filter und Trilateration vergleichbar bleiben.

---

## Antenna Switching During Connection Events

This is the critical new element compared to a multi-node architecture.

### Per-event antenna rotation

Der Scheduler läuft deterministisch auf dem Connection-Intervall und korrigiert
Jitter über einen absoluten Zielzeitstempel (`next_poll_deadline_ms`).

```
CI[n] anchor ---- sample(Ant X) ---- CI[n+1] anchor ---- sample(Ant Y)
            ^ corrected timestamp     ^ corrected timestamp
```

- Der nächste Poll wird auf `deadline += CONN_INTERVAL_MS` gesetzt, nicht nur
  relativ zu „jetzt“. Damit driftet die Sequenz nicht weg.
- Der übergebene Sample-Zeitstempel wird um Session-Overhead korrigiert
  (`CS_SESSION_TIMING_US`), damit A/B/C-Samples zeitlich vergleichbar bleiben.
- Vor dem nächsten Event wird auf die nächste Antenne geschaltet und ein
  definiertes Settling-Budget eingehalten (`CS_SWITCH_SETTLE_US`).

### Ablauf pro Connection-Intervall

1. `t = anchor_n`: RSSI auf aktiver Antenne `X` lesen.
2. Zeitstempel korrigieren: `t_sample = now - session_overhead`.
3. RSSI in Filter `X` einspeisen.
4. Nächste Antenne nach Round-Robin bestimmen (A→B→C).
5. RF-Switch umschalten, Settling-Zeitbudget abwarten.
6. Nächsten Poll exakt auf `anchor_{n+1}` terminieren (Drift-Korrektur aktiv).

### Single-Antenna-CS (Fallback)

Ist Single-Antenna-CS aktiv, entfällt Schritt 4/5; alle Samples gehen auf
`MY_ANTENNA_ID`. Der deterministische Zeitplan bleibt identisch.

---

## Keep-Alive Strategy

### Potential causes of drops

1. **Interference** -- 2.4 GHz is crowded (WiFi, other BLE devices).
2. **Range** -- Keyfob moves out of range temporarily.
3. **Body absorption** -- Driver's body blocks the signal.
4. **Metal reflections** -- Vehicle body causes multipath fading.
5. **Antenna mismatch** -- Current antenna may be in a null for this keyfob
   position. Unlike multi-node, only one antenna is active at a time.

### Mitigation

1. **Use all 37 data channels** -- BLE frequency hopping helps with
   narrowband interference.

2. **Application-level heartbeat** -- The keyfob sends a 1-byte GATT
   notification every connection event. This confirms end-to-end data flow.
   The RSSI from this notification is used for position estimation.

3. **PHY selection** -- Use LE 1M PHY for maximum range and robustness.
   LE Coded PHY (S=8) extends range significantly at the cost of throughput.

   ```c
   struct bt_conn_le_phy_param phy = {
       .options = BT_CONN_LE_PHY_OPT_CODED_S8,
       .pref_tx_phy = BT_GAP_LE_PHY_CODED,
       .pref_rx_phy = BT_GAP_LE_PHY_CODED,
   };
   bt_conn_le_phy_update(conn, &phy);
   ```

4. **Antenna watchdog for systematic failures** -- Jede Antenne hat einen
   Fehlerzähler für aufeinanderfolgende RSSI-Read-Fehler. Wird
   `CS_WATCHDOG_MAX_FAILURES` erreicht, wird die Antenne für
   `CS_WATCHDOG_COOLDOWN_CYCLES` Rotationen temporär ausgesetzt.

   Regeln:
   - Erfolg auf Antenne `i` -> Fehlerzähler `i = 0`
   - Fehler auf Antenne `i` -> Fehlerzähler `i++`
   - Schwellwert erreicht -> `cooldown[i] = N`, Antenne bei Rotation skippen
   - Nach Cooldown -> Antenne automatisch wieder zulassen

---

## Auto-Reconnect Implementation

### State machine

```c
enum ble_state {
    BLE_STATE_IDLE,
    BLE_STATE_SCANNING,
    BLE_STATE_CONNECTING,
    BLE_STATE_CONNECTED,
    BLE_STATE_RECONNECTING,
};
```

### Reconnect with exponential backoff

When a connection drops, immediately start scanning. If the keyfob is found,
attempt to reconnect. If the attempt fails, back off:

```
Attempt 1: wait 100 ms, then scan for 1 s
Attempt 2: wait 200 ms, then scan for 1 s
Attempt 3: wait 400 ms, then scan for 2 s
Attempt 4: wait 800 ms, then scan for 2 s
Attempt 5: wait 1600 ms, then scan for 5 s
Attempt 6+: wait 3000 ms, then scan for 5 s  (cap)
```

After a successful reconnect, reset the backoff counter.

### Antenna handling during scan (reconnect fallback)

While scanning for the keyfob, rotate the antenna on a timer:

```c
/* During scanning: rotate antenna every 200 ms to cover all directions */
static void scan_antenna_timer_handler(struct k_timer *timer)
{
    current_antenna = (current_antenna + 1) % NUM_ANTENNAS;
    rf_switch_select(current_antenna);
}
```

This ensures the scanner can hear the keyfob regardless of which direction it's
in. The scanned RSSI is fed into the Kalman filter for the current antenna,
maintaining (reduced accuracy) position tracking during reconnect.

### Implementation sketch

```c
#define RECONNECT_MAX_BACKOFF_MS 3000
#define RECONNECT_INITIAL_MS     100

static uint32_t reconnect_delay_ms = RECONNECT_INITIAL_MS;

static void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_WRN("Disconnected (reason 0x%02x), starting reconnect", reason);
    state = BLE_STATE_RECONNECTING;
    reconnect_delay_ms = RECONNECT_INITIAL_MS;
    k_work_reschedule(&reconnect_work, K_MSEC(reconnect_delay_ms));

    /* Start scan antenna rotation */
    k_timer_start(&scan_antenna_timer, K_MSEC(200), K_MSEC(200));
}

static void on_connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        reconnect_delay_ms = MIN(reconnect_delay_ms * 2,
                                 RECONNECT_MAX_BACKOFF_MS);
        k_work_reschedule(&reconnect_work, K_MSEC(reconnect_delay_ms));
        return;
    }
    state = BLE_STATE_CONNECTED;
    reconnect_delay_ms = RECONNECT_INITIAL_MS;

    /* Stop scan antenna rotation, use per-event rotation */
    k_timer_stop(&scan_antenna_timer);
}
```

---

## Connection Security

1. **Bonding** -- Pair the keyfob with the receiver once. Subsequent
   connections use stored keys and reconnect faster.
2. **LE Secure Connections** -- Use LESC (P-256 ECDH) for strong pairing.
3. **Encrypted link** -- Encrypt to prevent spoofing. RSSI itself can't be
   spoofed (measured at the receiver's radio), but preventing unauthorized
   devices from impersonating the keyfob is important.

```c
/* Require bonding and LESC */
bt_conn_auth_cb_register(&auth_cbs);

/* After connection, request security level 4 (LESC + encryption) */
bt_conn_set_security(conn, BT_SECURITY_L4);
```

---

## nRF54L15 Specific Considerations

- **Single connection** -- Only one connection to the keyfob is needed (no
  multi-central complexity). The nRF54L15 supports multiple connections but
  we only use one.
- **Radio API** -- The nRF54L15 uses a different radio peripheral (RADIO with
  DPPIC) compared to nRF5340. Check nRF Connect SDK v2.9+ for nRF54L15
  radio event hooks.
- **GPIO voltage** -- nRF54L15 GPIO is 1.8V by default. The SKYA21039 V1/V2
  inputs accept 1.8V logic levels (VIH = 1.0V, VIL = 0.4V).
- **Power** -- nRF54L15 has lower active current than nRF5340 (~3.5 mA at
  64 MHz), beneficial for the always-on receiver.
