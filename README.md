# Microservizio: `mar-lidar-service`

**Perimetral 2D Perception & Saccade Proximity Trigger**  
*Parte dell'architettura a microservizi M.A.R. (Multi-legged Autonomous Robot)*

---

## Ruolo nell'Architettura a Microservizi

`mar-lidar-service` è il primo anello della catena di **Percezione Attiva (Active Perception)** del quadrupede:
1. **Scansione Perimetrale 360° ad Alta Frequenza**:
   - Driver per unità LiDAR serie LD06 / LD19 / D300 / OKdo HAT (protocollo seriale 0x54 a 230400 baud) e RPLidar USB.
   - Applicazione dell'offset fisico di montaggio del robot (`OFFSET_ANGOLO_LIDAR = 90.0°`).
   - Filtraggio statistico outlier basato su range interquartile (IQR) e smorzamento a media mobile esponenziale (EMA $\alpha = 0.35$).
   - Segmentazione a 8 settori: `Fronte`, `Front-DX`, `Destra`, `Retro-DX`, `Retro`, `Retro-SX`, `Sinistra`, `Front-SX`.
2. **Generatore di Trigger Saccadico**:
   - Calcola la distanza minima da ciascun ostacolo. Se $D < D_{\text{soglia}}$, calcola l'angolo azimutale $\theta_{\text{target}}$ e trasmette l'alert al `mar-decision-engine`.
3. **Flusso Radar per la Dashboard**:
   - Pubblica una scansione 360° compressa sul topic `mar/lidar/scan`, utilizzata dalla dashboard Next.js per il rendering della mappa radar real-time.
4. **Modalità Simulazione (Zero-Hardware Dev)**:
   - Include un generatore sintetico di scansioni a 360° per sviluppare e testare la pipeline senza hardware collegato.

---

## Interfacce e Topic MQTT

| Topic | Tipo | Formato Payload | Descrizione |
| :--- | :--- | :--- | :--- |
| `mar/lidar/alert` | Evento (Pub) | `{"theta": 35.0, "sector": "Front-DX", "dist_m": 0.85, "is_emergency": false}` | Trigger di puntamento attivo per il Decision Engine |
| `mar/lidar/scan` | Flusso (Pub) | `{"timestamp": ..., "points": [[0, 2.5], [4, 2.4]], "sector_mins": {...}}` | Dati point cloud per la mappa radar 2D |

---

## Struttura del Repository

```text
mar-lidar-service/
├── README.md              # Questo documento
├── requirements.txt       # pyserial, paho-mqtt, numpy
├── lidar_driver.py        # Driver hardware LD06/OKdo HAT/RPLIDAR con fallback simulato
├── sector_analyzer.py     # Logica di segmentazione a 8 settori, filtri IQR ed EMA
└── main.py                # Demone del microservizio
```

---

## Avvio del Microservizio

### 1. Installazione Dipendenze
```bash
pip install -r requirements.txt
```

### 2. Esecuzione
```bash
# Esecuzione standard con LiDAR su porta UART/USB
LIDAR_PORT=/dev/ttyAMA0 LIDAR_BAUD=230400 python main.py

# In modalità simulata (senza hardware)
LIDAR_SIMULATE=true python main.py
```
