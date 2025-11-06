# M5StickC Plus2 Digi‑Clock + MQTT センサーモニター

[English here / README.md](README.md)

**M5StickC Plus2** で以下を実現する Arduino スケッチです：
- 本体 LCD に **CO₂** と **THI（温熱快適性指数）** を大きく交互表示
- **Digi‑Clock Unit**（4桁 7セグ）に NTP 同期した `HH:MM` を表示（分が変わった時のみ更新）
- **重力を利用した 180° 自動回転**（メインボタンが右/左のどちら向きでも表示が正立）

さらに **24/7 運用に耐える堅牢化** を実施：
- Wi‑Fi / MQTT の自動再接続（指数バックオフ）
- MQTT の LWT（異常切断時の最終意思）と KeepAlive / タイムアウト / バッファ調整
- NTP の強制再同期（最長間隔の上限を設けドリフト防止）
- ESP32 タスク WDT（ハング検出と復帰）
- OTA（ArduinoOTA）対応
- 右上の軽量ステータス表示（Wi‑Fi/MQTT/空きヒープ）

> 機微情報は **`config.h`** に記述します（**Git 管理対象外**）。リポジトリには **`config.example.h`** を同梱しますので、**`config.h` にコピーしてから編集**してください。

![IMG_7642](https://github.com/user-attachments/assets/22282b3d-a9fc-4c1a-8e17-125ba6dedae2)
---

## 特長

- **MQTT 受信**（単一トピック）＆ JSON パース（ArduinoJson）
- タイトル・NTP 時刻・MQTT 接続状態と **CO₂ / THI** の大文字表示
- **Digi‑Clock** は分が変化した時のみ更新してチラつきを抑制
- **NTP 同期**：`pool.ntp.org`、JST（+9h）を既定（変更可）
- **重力ベースの自動回転**：ローパス、傾きしきい値、ヒステリシスを `config.h` で調整可能
- **24/7 向け強化**：バックオフ再接続、LWT、WDT、OTA、ステータスオーバーレイ

---

## ハードウェア

- M5Stack **M5StickC Plus2**
- M5Stack **Digi‑Clock Unit**（Grove I²C）
- Wi‑Fi と MQTT ブローカ（例：Mosquitto）

**配線**（StickC Plus2 の Grove）  
SDA = **G32**、SCL = **G33**（コード内で `Wire.begin(32, 33)` を使用）

![IMG_7641](https://github.com/user-attachments/assets/447d5299-f0c0-49b3-81fc-6e6596c49433)
---

## 使用ライブラリ

Arduino IDE でインストール：
- **M5StickCPlus2**（M5Unified ベース）
- **PubSubClient**
- **ArduinoJson**
- **NTPClient**
- **M5UNIT_DIGI_CLOCK**
- （ESP32 Arduino Core）

---

## 設定

1. **`config.example.h`** を **`config.h`** にコピー。  
2. `config.h` を開き、Wi‑Fi／MQTT／自動回転などを編集。

主要項目（抜粋）:
```cpp
// Wi‑Fi
const char* WIFI_NETWORK_NAME     = "Your_WiFi_SSID";
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";

// MQTT
const char* MQTT_BROKER_ADDRESS   = "192.168.1.100";
const int   MQTT_BROKER_PORT      = 1883;
const char* MQTT_TOPIC_NAME       = "sensor_data";
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";
const bool  MQTT_USE_AUTH         = false;   // 認証が必要なら true
const char* MQTT_USERNAME         = "";
const char* MQTT_PASSWORD         = "";
// LWT（異常切断時の最終意思）
const char* MQTT_LWT_TOPIC        = "device/m5stickcplus2/status";
const char* MQTT_LWT_MESSAGE      = "offline";
const int   MQTT_LWT_QOS          = 1;
const bool  MQTT_LWT_RETAIN       = true;
// PubSubClient 調整
const uint16_t MQTT_KEEPALIVE_SECONDS      = 30;
const uint16_t MQTT_SOCKET_TIMEOUT_SECONDS = 10;
const size_t   MQTT_BUFFER_SIZE            = 1024;

// NTP / 時刻
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";
const long  JAPAN_TIME_OFFSET_SECONDS = 32400; // +9時間
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000; // 60秒
const unsigned long TIME_MAX_FORCE_RESYNC_MILLISECONDS = 3600000; // 最低でも1時間に1回は強制再同期

// 自動回転
const bool  ENABLE_GRAVITY_AUTO_ROTATE    = true;
const unsigned long ORIENTATION_CHECK_INTERVAL_MS = 200;
const float ORIENTATION_TILT_THRESHOLD_G  = 0.20f;
const float ORIENTATION_HYSTERESIS_G      = 0.05f;
const int   DISPLAY_ROTATION_NORMAL       = 1;
const int   DISPLAY_ROTATION_FLIPPED      = 3;
const bool  ORIENTATION_INVERT_X          = false;

// 24/7 バックオフ & ヘルス
const unsigned long WIFI_RETRY_BACKOFF_INITIAL_MS = 1000;  // 1秒
const unsigned long WIFI_RETRY_BACKOFF_MAX_MS     = 60000; // 最大60秒
const unsigned long MQTT_RETRY_BACKOFF_INITIAL_MS = 1000;
const unsigned long MQTT_RETRY_BACKOFF_MAX_MS     = 60000;
const bool   ENABLE_TASK_WATCHDOG  = true;
const int    WDT_TIMEOUT_SECONDS   = 10;
const bool   ENABLE_STATUS_OVERLAY = true;
// UI のタイミング
const unsigned long CONNECTION_SUCCESS_DISPLAY_TIME = 1500;   // 成功/通知画面の表示時間 (ms)
const unsigned long INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS = 3000; // CO2/THI の交互表示間隔 (ms)
```

---

## ビルド & 書き込み（Arduino IDE）

1. `.ino` と `config.h` があるフォルダを開く。  
2. **ボード**と**ポート**で M5StickC Plus2 を選択。  
3. **検証** → **書き込み**。

---

## MQTT ペイロード

購読トピックは `MQTT_TOPIC_NAME` です。例：
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
すべてのキーは任意です。受信した項目のみ表示し、**CO₂** と **THI** を数秒ごとに交互表示します。

## UI タイミング早見表

- 成功/通知画面の表示時間: `CONNECTION_SUCCESS_DISPLAY_TIME`（既定: 1500 ms）
- CO₂/THI の交互表示間隔: `INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS`（既定: 3000 ms）
- Digi‑Clock は分が変わったときのみ更新（明示的な待機なし）
- 自動回転のポーリング間隔: `ORIENTATION_CHECK_INTERVAL_MS`（既定: 200 ms）

`config.h` でこれらを調整すると、表示のテンポを用途に合わせて変更できます。

---

## トラブルシュート

- **OTA が Arduino IDE に出てこない**
  - PC とデバイスが同じ LAN で mDNS が有効か確認
  - スケッチは起動時に `ArduinoOTA.begin()` を呼びます。ネットワーク到達性を確認
  - それでも見つからない場合は IDE から IP を直接指定するか、FW 設定を見直し

- **MQTT Connection Failed, rc = -2**  
  PubSubClient の `state()` が **-2** の場合は **MQTT_CONNECT_FAILED**（トランスポート層の失敗）です。以下をご確認ください：  
  - Wi‑Fi へ接続済みで、IP が取得できているか。  
  - `MQTT_BROKER_ADDRESS` が到達可能か（同一サブネット／FW 制限なし）。  
  - ブローカが `MQTT_BROKER_PORT`（標準 1883）で待受中か。TLS には未対応のスケッチです。  
  - ブローカがユーザー名／パスワードを要求する場合、`connect(clientId, user, pass)` へ拡張してください。

- **Digi‑Clock が表示しない**  
  Grove ケーブルの向き、I²C ピン（**G32/SDA、G33/SCL**）をご確認ください。接続後に電源再投入が確実です。

- **時刻が `--` のまま／不正確**  
  NTP サーバへ到達できるネットワークか確認してください。JST 以外はオフセットを変更。

- **JSON Error が出る**  
  JSON の整形・引用符・桁数をご確認ください。

---

## リポジトリ運用

- **`config.example.h`** をコミットし、**`config.h`** は `.gitignore` に追加してください（秘密情報保護）。

## 24/7 運用メモ

- 再接続は指数バックオフで行い、AP やブローカへの負荷を避けます。
- タスク WDT（10秒）を有効化しています。長い `delay()` の追加は避けてください。
- NTP の強制再同期を 1 時間に 1 回は行い、時刻ドリフトを抑制します。
- `config.h` の `ENABLE_PERIODIC_SOFT_RESTART` で定期再起動も有効化できます（任意）。

### 最近の変更
- 成功画面などの表示時間を 2000 ms → 1500 ms に短縮しました（`CONNECTION_SUCCESS_DISPLAY_TIME`）。

---

## ライセンス

MIT（`LICENSE` を参照）。
