# 📡 Microservizio: `mar-lidar-service`

**Perimetral 2D Perception & Saccade Proximity Trigger**  
*Parte dell'architettura a microservizi M.A.R. (Multi-legged Autonomous Robot)*

---

## 🎯 Ruolo nell'Architettura a Microservizi

`mar-lidar-service` è il primo anello della catena di **Percezione Attiva (Active Perception)** del quadrupede:
1. **Scansione Perimetrale 360° ad Alta Frequenza**:
   - Acquisisce costantemente la point cloud 2D dall'unità LiDAR USB (0.15 m – 12 m).
   - Rimuove outlier e segmenta lo spazio in 6 settori: `FRONT`, `FRONT_LEFT`, `FRONT_RIGHT`, `LEFT`, `RIGHT`, `REAR`.
2. **Generatore di Trigger Saccadico**:
   - Calcola la distanza minima da ciascun ostacolo. Se $D < D_{\text{soglia}}$, calcola l'angolo azimutale $\theta_{\text{target}}$ e trasmette l'alert al `mar-decision-engine`.
3. **Flusso Radar per la Dashboard**:
   - Pubblica una scansione 360° alleggerita e compressa sul topic `mar/lidar/scan`, utilizzata dalla dashboard Next.js per il rendering della mappa radar real-time.
4. **Modalità Simulazione (Zero-Hardware Dev)**:
   - Include un generatore sintetico di ostacoli per sviluppare e testare la logica su PC/Mac senza dover collegare il sensore fisico.

---

## 📡 Interfacce e Topic MQTT

| Topic | Tipo | Formato Payload | Descrizione |
| :--- | :--- | :--- | :--- |
| `mar/lidar/alert` | Evento (Pub) | `{"theta": 35.0, "sector": "FRONT_RIGHT", "dist_m": 0.85, "is_emergency": false}` | Trigger di puntamento attivo per il Decision Engine |
| `mar/lidar/scan` | Flusso (Pub) | `{"timestamp": ..., "points": [[0, 2.5], [4, 2.4]], "sector_mins": {...}}` | Dati point cloud per la mappa radar 2D |

---

## 📂 Struttura della Cartella

```text
mar-lidar-service/
├── README.md              # Questo documento
├── requirements.txt       # rplidar-roboticia, paho-mqtt, numpy
├── lidar_driver.py        # Driver hardware RPLIDAR con fallback simulato
├── sector_analyzer.py     # Logica geometrica di segmentazione a settori e alert
└── main.py                # Demone del microservizio
```

---

## 🚀 Avvio del Microservizio

### 1. Installazione Dipendenze
```bash
pip install -r requirements.txt
```

### 2. Esecuzione
```bash
# Con sensore hardware collegato via USB
LIDAR_PORT=/dev/ttyUSB0 python main.py

# In modalità simulata (senza hardware)
LIDAR_SIMULATE=true python main.py
```
