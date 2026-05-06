# Validation-, Logging- und Rollout-Plan (BLE CS + RSSI Fallback)

Dieses Dokument definiert die Teststufen, Abnahmekriterien, ein einheitliches
Logging-Format und die Rollout-Strategie für die Einführung von
BLE Channel Sounding (CS) mit RSSI-Fallback.

## 1) Teststufen

### Stufe A: Lab (stationär)

**Ziel:** Reproduzierbare Baseline bei minimalen Störfaktoren.

- Aufbau:
  - Feste Receiver-Positionen (A/B/C), definierte Antennenorientierung.
  - Keyfob stationär auf markierten Referenzpunkten (z. B. 0.5 m, 1 m, 2 m,
    3 m, 5 m).
- Umgebung:
  - Innenraum mit kontrollierter Interferenz (wenige bewegte Objekte).
- Fokus:
  - Kalibrierung und Stabilität von Distanz-/Sektorschätzung.
  - Reproduzierbarkeit über mehrere Messreihen.

### Stufe B: Parkplatz (statisch + Gehpfade)

**Ziel:** Realistischere Multipath-Bedingungen bei kontrolliertem Bewegungsprofil.

- Aufbau:
  - Fahrzeug oder Testobjekt in typischer Außenumgebung.
  - Keyfob statisch an Referenzpunkten + definierte Gehpfade
    (front/left/right/rear, Annäherung/Entfernung).
- Umgebung:
  - Moderate Interferenz durch Fahrzeuge, Reflektionen, Personenbewegung.
- Fokus:
  - Tracking-Kontinuität entlang Gehpfaden.
  - Einfluss von Körperabschattung und Fahrzeugreflexionen.

### Stufe C: Realbetrieb (urban, Interferenz)

**Ziel:** Verifikation unter produktionsnaher Last und starker Funkinterferenz.

- Aufbau:
  - Mehrere reale Nutzungsszenarien (Straßenrand, Parkhaus, Innenstadt).
  - Unterschiedliche Tageszeiten und Funkauslastung.
- Umgebung:
  - Hohe Interferenz (WLAN/BLE-Geräte, reflektierende Flächen,
    veränderliche Bewegungsmuster).
- Fokus:
  - Robustheit, Reconnect-Verhalten, Degradation-Erkennung.
  - Stabiler Fallback bei CS-Problemen.

## 2) Abnahmekriterien pro Stufe

Die folgenden Kriterien gelten je Stufe; Zielwerte können je Programmphase
(MVP/Pilot/SOP) verschärft werden.

### 2.1 Positions-/Sektor-P90

- **Positions-P90:** 90%-Quantil des absoluten Distanzfehlers
  (`|d_est - d_ref|`).
- **Sektor-P90:** 90%-Quantil des Sektorfehlers (Abweichung zwischen
  geschätztem und Referenz-Sektor in Grad oder Klassen).

**Mindestanforderung pro Stufe:**

- Lab:
  - Distanzfehler-P90 <= 1.0 m
  - Sektorfehler-P90 <= 30°
- Parkplatz:
  - Distanzfehler-P90 <= 1.5 m
  - Sektorfehler-P90 <= 45°
- Realbetrieb:
  - Distanzfehler-P90 <= 2.5 m
  - Sektorfehler-P90 <= 60°

### 2.2 Reconnect-Zeit

- Metrik: Zeit zwischen Linkverlust und erstem wieder gültigen
  Positions-/Sektor-Update.
- Kennzahl: P90 Reconnect-Zeit.

**Mindestanforderung pro Stufe:**

- Lab: P90 <= 2 s
- Parkplatz: P90 <= 3 s
- Realbetrieb: P90 <= 5 s

### 2.3 Messlückenrate

- Metrik: Anteil erwarteter Messfenster ohne gültigen Messdatensatz.
- Formel: `Messlückenrate = fehlende_frames / erwartete_frames`.

**Mindestanforderung pro Stufe:**

- Lab: <= 1%
- Parkplatz: <= 3%
- Realbetrieb: <= 5%

## 3) Einheitliches Logging-Format (Receiver + Keyfob)

Es wird parallel CSV (schnelle Auswertung) und JSON (strukturierte Events)
verwendet. Beide Formate nutzen dasselbe Feldschema.

### 3.1 Pflichtfelder

- `ts_ms`: Unix-Zeitstempel in Millisekunden
- `device_role`: `receiver` | `keyfob`
- `device_id`: Eindeutige Knoten-ID
- `session_id`: Mess-/Testlauf-ID
- `mode`: `cs` | `rssi` | `hybrid`
- `event`: `measurement` | `state_change` | `reconnect` | `error`
- `rssi_dbm`: RSSI-Wert (falls verfügbar)
- `distance_m`: geschätzte Distanz (falls verfügbar)
- `sector_deg`: geschätzter Sektorwinkel (falls verfügbar)
- `conn_state`: `connected` | `scanning` | `reconnecting` | `idle`
- `gap_flag`: `0|1` (Messlücke im aktuellen Fenster)
- `err_code`: Fehlercode oder `0`

### 3.2 CSV-Beispiel

```csv
ts_ms,device_role,device_id,session_id,mode,event,rssi_dbm,distance_m,sector_deg,conn_state,gap_flag,err_code
1710500000123,receiver,rx-a,test-2025-03-15,cs,measurement,-64,2.1,35,connected,0,0
1710500000231,keyfob,kf-01,test-2025-03-15,rssi,state_change,-71,,,connected,0,0
```

### 3.3 JSON-Beispiel

```json
{
  "ts_ms": 1710500000123,
  "device_role": "receiver",
  "device_id": "rx-a",
  "session_id": "test-2025-03-15",
  "mode": "cs",
  "event": "measurement",
  "rssi_dbm": -64,
  "distance_m": 2.1,
  "sector_deg": 35,
  "conn_state": "connected",
  "gap_flag": 0,
  "err_code": 0
}
```

## 4) Rollout-Strategie

### 4.1 Feature-Flag: `CONFIG_BLE_CS_ENABLE`

- `CONFIG_BLE_CS_ENABLE=n` (Default): RSSI-only Betrieb.
- `CONFIG_BLE_CS_ENABLE=y`: CS-Pipeline aktivieren (stufenweiser Rollout).

Empfohlene Phasen:

1. **Lab-Only:** Flag nur auf internen Testbuilds.
2. **Parking Pilot:** Kleiner Fahrzeug-/Nutzerkreis.
3. **Urban Pilot:** Erhöhte Interferenz, Monitoring aktiv.
4. **Broad Rollout:** Graduelle Ausweitung mit KPI-Gates.

### 4.2 Fallback auf RSSI bei CS-Degradation

- Trigger für Degradation:
  - CS-Frames fehlen wiederholt,
  - CS-Latenz über Schwellwert,
  - CS-Qualitätsindikator unter Mindestwert.
- Verhalten:
  1. `mode` auf `rssi` setzen,
  2. Positions-/Sektor-Updates mit RSSI fortführen,
  3. Degradation im Log markieren (`event=error|state_change`),
  4. periodisch CS-Recovery-Versuch durchführen.

Damit bleibt das System auch bei temporären CS-Problemen betriebsfähig,
bei reduzierter Genauigkeit statt vollständigem Ausfall.
