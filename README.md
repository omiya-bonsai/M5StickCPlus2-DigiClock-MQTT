# M5StickC Plus2 Digi‑Clock + MQTT Sensor Monitor

[日本語はこちら / README‑ja.md](README-ja.md)

An Arduino sketch for **M5StickC Plus2** that:
- Displays **CO₂** and **THI (Thermal Comfort Index)** on the built‑in LCD (alternating view).
- Drives the **M5Stack Digi‑Clock Unit** (4‑digit 7‑segment) via Grove (I²C) to show network‑synced time in `HH:MM`.
- **Auto‑rotates** the LCD by **180° using gravity** so the UI stays upright whether the main button is on the right (default) or left.

> The private settings live in `config.h` which should **NOT** be committed. In this repo, we ship **`config.example.h`** — copy it to `config.h` and edit for your environment.

---

## Features

- **MQTT subscribe** to a single topic and parse JSON payloads (ArduinoJson).
- **CO₂ / THI** large-font view with title, NTP clock, and MQTT status.
- **Digi‑Clock** shows minute‑accurate time without flicker (updates only when minute changes).
- **NTP time** via `pool.ntp.org`, JST offset by +9h (configurable).
- **Gravity‑based auto‑rotation** with low‑pass filtering, tilt threshold, and hysteresis — all configurable in `config.h`.

---

## Hardware

- M5Stack **M5StickC Plus2**
- M5Stack **Digi‑Clock Unit** (I²C via Grove)
- Wi‑Fi network and an MQTT broker (e.g., Mosquitto)

**Wiring** (Grove on StickC Plus2):
- SDA = **G32**, SCL = **G33** (handled in code with `Wire.begin(32, 33)`)

---

## Libraries

Install these in Arduino IDE:
- **M5StickCPlus2** (M5Stack / M5Unified based)
- **PubSubClient**
- **ArduinoJson**
- **NTPClient**
- **M5UNIT_DIGI_CLOCK**
- (ESP32 core for Arduino)

---

## Configure

1. Copy **`config.example.h`** → **`config.h`** (same folder as the sketch).
2. Open `config.h` and set Wi‑Fi, MQTT, and display parameters.

Key fields (excerpt):
```cpp
// Wi-Fi
const char* WIFI_NETWORK_NAME     = "Your_WiFi_SSID";
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";

// MQTT
const char* MQTT_BROKER_ADDRESS   = "192.168.1.100";
const int   MQTT_BROKER_PORT      = 1883;
const char* MQTT_TOPIC_NAME       = "sensor_data";
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";

// NTP / Time
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";
const long  JAPAN_TIME_OFFSET_SECONDS = 32400; // +9h
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000;

// Auto-rotation
const bool  ENABLE_GRAVITY_AUTO_ROTATE    = true;
const unsigned long ORIENTATION_CHECK_INTERVAL_MS = 200;
const float ORIENTATION_TILT_THRESHOLD_G  = 0.20f;
const float ORIENTATION_HYSTERESIS_G      = 0.05f;
const int   DISPLAY_ROTATION_NORMAL       = 1;
const int   DISPLAY_ROTATION_FLIPPED      = 3;
const bool  ORIENTATION_INVERT_X          = false;
```

---

## Build & Upload (Arduino IDE)

1. Open the folder containing the `.ino` and `config.h`.
2. Select the proper **Board** and **Port** for M5StickC Plus2.
3. **Verify** and **Upload**.

---

## MQTT Payload

This sketch subscribes to `MQTT_TOPIC_NAME`. Example JSON:
```json
{
  "co2": 820,
  "thi": 75.2,
  "temperature": 27.3,
  "humidity": 60.0,
  "comfort_level": "Warm",
  "timestamp": 1724639200
}
```
All keys are optional; present ones will be displayed. The screen alternates **CO₂** and **THI** every few seconds.

---

## Troubleshooting

- **MQTT Connection Failed, rc = -2**  
  PubSubClient’s `state()` code **-2** means **MQTT_CONNECT_FAILED** (transport error). Check:
  - Wi‑Fi is connected and the device has an IP.
  - `MQTT_BROKER_ADDRESS` is reachable from the StickC (same subnet / no firewall).
  - Broker is listening on `MQTT_BROKER_PORT` (1883 by default), no TLS for this sketch.
  - If your broker requires **username/password**, extend the code: `connect(clientId, user, pass)`.

- **Digi‑Clock shows nothing**  
  Confirm Grove cable orientation; I²C pins are **G32 (SDA), G33 (SCL)**. Power‑cycle after attaching.

- **Time stays `--` or wrong**  
  Ensure NTP server reachable from your network; adjust offset if you’re not in JST.

- **JSON Error** on screen  
  Check that your payload is valid JSON and all quotes are correct.

---

## Repository Hygiene

- Commit **`config.example.h`**.  
- Add **`config.h`** to `.gitignore` to avoid leaking secrets.

---

## License

MIT (see `LICENSE`).
