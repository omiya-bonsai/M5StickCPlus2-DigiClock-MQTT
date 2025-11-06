/*
 * @file M5StickCPlus2_DigiClock_MQTT.ino
 * @brief M5StickCPlus2 + Digi-Clock Unit: MQTT monitor + NTP clock
 *        + (NEW) Gravity-based auto-rotation of the built-in display.
 *
 * NOTE:
 *  - Requires libraries: M5StickCPlus2 (based on M5Unified), M5GFX, PubSubClient, ArduinoJson, NTPClient
 *  - Place this .ino and config.h in the same sketch folder.
 */

#include <M5StickCPlus2.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>
#include "config.h"
#include <M5UNIT_DIGI_CLOCK.h>

// 24/7 運用向け: OTA と WDT（ESP32）
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <esp_system.h>
#endif

// =================================================================
// 2. データ構造体
// =================================================================
struct SensorDataPacket
{
  int carbonDioxideLevel;
  float thermalComfortIndex;
  float ambientTemperature;
  float relativeHumidity;
  String comfortLevelDescription;
  unsigned long dataTimestamp;
  bool hasValidData;
};

// =================================================================
// 3. グローバル変数
// =================================================================

// --- ネットワーク関連 ---
WiFiUDP networkUdpClient;
NTPClient timeClient(networkUdpClient, TIME_SERVER_ADDRESS, JAPAN_TIME_OFFSET_SECONDS, TIME_UPDATE_INTERVAL_MILLISECONDS);
WiFiClient networkWifiClient;
PubSubClient mqttCommunicationClient(networkWifiClient);

// 再接続・バックオフ管理
static bool wifiIsConnected = false;
static unsigned long wifiBackoffMs = WIFI_RETRY_BACKOFF_INITIAL_MS;
static unsigned long wifiNextReconnectAt = 0;
static unsigned long mqttBackoffMs = MQTT_RETRY_BACKOFF_INITIAL_MS;
static unsigned long mqttNextReconnectAt = 0;
static unsigned long lastNtpOkMillis = 0;
static unsigned long bootMillis = 0;

// --- センサーデータ関連 ---
SensorDataPacket currentSensorReading = {0, 0.0, 0.0, 0.0, "", 0, false};

// --- 表示制御関連 ---
unsigned long lastDisplayUpdateTime = 0;
unsigned long lastInteractiveDisplayTime = 0;
bool displayCO2 = true;

// --- Digi-Clock Unit 関連 ---
M5UNIT_DIGI_CLOCK digi_clock;
int last_digiclock_minute = -1;

// --- NEW: 自動回転関連（重力ベース） ---
int currentDisplayRotation = DISPLAY_ROTATION_NORMAL;
unsigned long lastOrientationCheckTime = 0;
static float lp_ax = 0.0f, lp_ay = 0.0f, lp_az = 1.0f;
static bool lp_init = false;

// =================================================================
// 4. 前方宣言
// =================================================================

// ディスプレイ関連
void initializeDisplaySystem();
void showSystemStartupMessage();
void displayWiFiConnectionSuccess();
void displayNTPSynchronizationResult(bool wasSuccessful);
void displayMQTTConnectionSuccess();
void displayMQTTConnectionFailure();
void refreshEntireDisplay();
void updateDisplayIfIntervalElapsed();
void displayApplicationTitle();
void displayCurrentSystemTime();
void displaySensorDataOrErrorMessage();
void displayCO2ConcentrationData();
void displayTHIComfortData();
void displayNoDataAvailableMessage();
void displayNetworkConnectionStatus();
void displayJSONParsingError(const char *errorDescription);
void showConnectionStatusMessage(const char *statusMessage);
void clearDisplayScreenWithColor(uint16_t backgroundColor);

// WiFi
void establishWiFiConnection();
bool checkWiFiConnectionStatus();

// NTP
void synchronizeSystemTimeWithNTP();
bool attemptNTPTimeSynchronization();
void updateSystemNetworkTime();

// MQTT
void configureMQTTConnection();
void establishMQTTBrokerConnection();
String generateUniqueMQTTClientId();
bool attemptMQTTBrokerConnection(const String &clientIdentifier);
void subscribeToMQTTDataTopic();
void handleIncomingMQTTMessage(char *topicName, byte *messagePayload, unsigned int messageLength);
bool validateJSONDataIntegrity(const String &jsonData);
String convertRawPayloadToString(byte *rawPayload, unsigned int payloadLength);
SensorDataPacket parseJSONSensorData(const String &jsonString);
void updateCurrentSensorData(const SensorDataPacket &newSensorData);
void maintainMQTTBrokerConnection();
void processIncomingMQTTMessages();
void printMQTTSubscriptionDebugInfo();

// Digi-Clock Unit
void initializeDigiClock();
void updateDigiClockDisplay();

// NEW: Gravity-based auto rotation
void autoRotateDisplayByGravity();

// 24/7: 監視・接続維持・OTA・WDT
void onWiFiEvent(WiFiEvent_t event);
void ensureWiFiConnection();
void scheduleWiFiReconnect();
void resetWiFiBackoff();
void configureMQTTConnection();
void ensureMQTTConnection();
void resetMQTTBackoff();
void setupTaskWatchdog();
void setupOTA();
void handleOTA();
void updateStatusOverlay();
String formatUptime();

// =================================================================
// 5. setup
// =================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("\n========== M5StickCPlus2 & Digi-Clock Monitor 起動 ==========");

  bootMillis = millis();
// リセット理由をログ（ESP32）
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  Serial.printf("Reset reason: %d\n", (int)esp_reset_reason());
#endif

  // Step 1: 本体表示の初期化 + IMU 初期化
  initializeDisplaySystem();
  showSystemStartupMessage();

  // Step 1.5: WDT 準備
  setupTaskWatchdog();

  // Step 2: Digi-Clock Unit 初期化
  initializeDigiClock();

  // Step 3: Wi-Fi 接続
  establishWiFiConnection();
  // WiFi イベント購読（切断/再接続を検知）
  WiFi.onEvent(onWiFiEvent);
  // OTA（WiFi接続時のみ開始、切断時も handle は安全）
  setupOTA();

  // Step 4: NTP 同期
  synchronizeSystemTimeWithNTP();

  // Step 5: MQTT 準備・接続
  configureMQTTConnection();
  establishMQTTBrokerConnection();

  // Step 6: 初期描画
  refreshEntireDisplay();

  Serial.println("========== 初期化処理完了：システム稼働開始 ==========");
}

// =================================================================
// 6. loop
// =================================================================
void loop()
{
// WDT クリア（ESP32）
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (ENABLE_TASK_WATCHDOG)
    esp_task_wdt_reset();
#endif

  // NEW: 重力による自動回転を先に評価（描画前に回転を確定させる）
  autoRotateDisplayByGravity();

  // WiFi / MQTT の維持（非ブロッキング）
  ensureWiFiConnection();
  ensureMQTTConnection();

  // 1. MQTT 再接続監視
  maintainMQTTBrokerConnection();

  // 2. MQTTメッセージ処理
  processIncomingMQTTMessages();

  // 3. 交互表示の更新（CO2 / THI）
  updateDisplayIfIntervalElapsed();

  // 4. NTP時刻の内部更新
  updateSystemNetworkTime();

  // 5. Digi-Clock 表示更新
  updateDigiClockDisplay();

  // 6. OTA
  handleOTA();

  // 7. ステータスオーバーレイ
  updateStatusOverlay();

// 8. 任意: 定期ソフト再起動
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (ENABLE_PERIODIC_SOFT_RESTART && (millis() - bootMillis) > SOFT_RESTART_INTERVAL_MS)
  {
    Serial.println("🔁 Periodic soft restart");
    ESP.restart();
  }
#endif

  // 6. 少し休む
  delay(MAIN_LOOP_DELAY_MILLISECONDS);
}

// =================================================================
// 7. 各関数
// =================================================================

// ---------------- Digi-Clock Unit ----------------
void initializeDigiClock()
{
  Wire.begin(32, 33);
  Serial.println("⚙️  I2C for Digi-Clock Unit starting...");

  if (!digi_clock.begin(&Wire))
  {
    Serial.println("❌ Digi-Clock Unit not found!");
    M5.Display.setCursor(10, 50);
    M5.Display.setTextColor(RED);
    M5.Display.println("DigiClock ERR");
// 長い待機でもWDT/OTAに配慮
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (ENABLE_TASK_WATCHDOG)
      esp_task_wdt_reset();
#endif
    delay(1500);
  }
  else
  {
    Serial.println("✅ Digi-Clock Unit found and initialized.");
    digi_clock.setBrightness(80);
    digi_clock.setString("----");
  }
}

void updateDigiClockDisplay()
{
  if (timeClient.getEpochTime() > 1672531200)
  {
    int minute = timeClient.getMinutes();
    if (minute != last_digiclock_minute)
    {
      int hour = timeClient.getHours();
      char time_string[6];
      sprintf(time_string, "%02d:%02d", hour, minute);
      digi_clock.setString(time_string);
      last_digiclock_minute = minute;
    }
  }
}

// ---------------- 本体ディスプレイ ----------------
void initializeDisplaySystem()
{
  // M5Unified ベースの初期化
  auto cfg = M5.config();
  M5.begin(cfg);

  // （Plus2 ライブラリ経由の IMU 初期化も明示しておく）
  StickCP2.Imu.begin();

  // 既定の回転を設定（右ボタン＝通常）
  M5.Display.setRotation(DISPLAY_ROTATION_NORMAL);

  clearDisplayScreenWithColor(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);

  Serial.println("✅ Display & IMU Initialized.");
}

void showSystemStartupMessage()
{
  clearDisplayScreenWithColor(BLACK);
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  M5.Display.println("Starting...");
  Serial.println("📱 Displaying startup message.");
}

void refreshEntireDisplay()
{
  clearDisplayScreenWithColor(BLACK);
  displayApplicationTitle();
  displayCurrentSystemTime();
  displayNetworkConnectionStatus();

  if (currentSensorReading.hasValidData)
  {
    if (displayCO2)
    {
      displayCO2ConcentrationData();
    }
    else
    {
      displayTHIComfortData();
    }
  }
  else
  {
    displayNoDataAvailableMessage();
  }
}

void updateDisplayIfIntervalElapsed()
{
  unsigned long currentSystemTime = millis();
  if (currentSystemTime - lastInteractiveDisplayTime >= INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS)
  {
    clearDisplayScreenWithColor(BLACK);
    displayApplicationTitle();
    displayCurrentSystemTime();
    displayNetworkConnectionStatus();

    if (currentSensorReading.hasValidData)
    {
      if (displayCO2)
      {
        displayCO2ConcentrationData();
      }
      else
      {
        displayTHIComfortData();
      }
      displayCO2 = !displayCO2;
    }
    else
    {
      displayNoDataAvailableMessage();
    }
    lastInteractiveDisplayTime = currentSystemTime;
  }
}

void displayApplicationTitle()
{
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(CYAN);
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  M5.Display.println("Sensor Monitor");
}

void displayCurrentSystemTime()
{
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(TIME_DISPLAY_X, TIME_DISPLAY_Y);
  M5.Display.println(timeClient.getFormattedTime());
}

void displayNetworkConnectionStatus()
{
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(mqttCommunicationClient.connected() ? GREEN : RED);
  M5.Display.setCursor(CONNECTION_STATUS_X, CONNECTION_STATUS_Y);
  M5.Display.println(mqttCommunicationClient.connected() ? "MQTT:OK" : "MQTT:NG");
}

void displayCO2ConcentrationData()
{
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(GREEN);
  M5.Display.setCursor(LARGE_LABEL_X, LARGE_LABEL_Y);
  M5.Display.println("CO2:");

  M5.Display.setTextSize(8);
  M5.Display.setTextColor(GREEN);
  M5.Display.setTextDatum(TR_DATUM);
  String co2Value = String(currentSensorReading.carbonDioxideLevel);
  M5.Display.drawString(co2Value, M5.Display.width() - DISPLAY_RIGHT_MARGIN, LARGE_VALUE_Y);
  M5.Display.setTextDatum(TL_DATUM);
}

void displayTHIComfortData()
{
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setCursor(LARGE_LABEL_X, LARGE_LABEL_Y);
  M5.Display.println("THI:");

  M5.Display.setTextSize(8);
  M5.Display.setTextColor(ORANGE);
  M5.Display.setTextDatum(TR_DATUM);
  String thiValue = String(currentSensorReading.thermalComfortIndex, 1);
  M5.Display.drawString(thiValue, M5.Display.width() - DISPLAY_RIGHT_MARGIN, LARGE_VALUE_Y);
  M5.Display.setTextDatum(TL_DATUM);
}

void displayNoDataAvailableMessage()
{
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(RED);
  M5.Display.setCursor(NO_DATA_MESSAGE_X, NO_DATA_MESSAGE_Y);
  M5.Display.println("No Data");
}

void displayJSONParsingError(const char *errorDescription)
{
  clearDisplayScreenWithColor(BLACK);

  M5.Display.setTextSize(1);
  M5.Display.setTextColor(CYAN);
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  M5.Display.println("Sensor Monitor");

  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(TIME_DISPLAY_X, TIME_DISPLAY_Y);
  M5.Display.println(timeClient.getFormattedTime());

  M5.Display.setTextSize(1);
  M5.Display.setTextColor(mqttCommunicationClient.connected() ? GREEN : RED);
  M5.Display.setCursor(CONNECTION_STATUS_X, CONNECTION_STATUS_Y);
  M5.Display.println(mqttCommunicationClient.connected() ? "MQTT:OK" : "MQTT:NG");

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(RED);
  M5.Display.setCursor(20, 50 + VERTICAL_OFFSET);
  M5.Display.println("JSON Error");

  M5.Display.setTextSize(1);
  M5.Display.setCursor(20, 80 + VERTICAL_OFFSET);
  M5.Display.println(errorDescription);
}

// ---------------- ネットワーク ----------------
void establishWiFiConnection()
{
  Serial.println("🌐 Attempting to connect to WiFi...");
  showConnectionStatusMessage("WiFi connecting...");
  WiFi.begin(WIFI_NETWORK_NAME, WIFI_NETWORK_PASSWORD);
  unsigned long start = millis();
  // 初回だけは数秒待機してUXを確保。その後は非ブロッキング再接続へ移行。
  while (!checkWiFiConnectionStatus() && millis() - start < 8000)
  {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (ENABLE_TASK_WATCHDOG)
      esp_task_wdt_reset();
#endif
    delay(250);
    M5.Display.print(".");
    Serial.print(".");
  }
  if (checkWiFiConnectionStatus())
  {
    displayWiFiConnectionSuccess();
    Serial.println("\n✅ WiFi Connection Successful.");
    Serial.print("   IP Address: ");
    Serial.println(WiFi.localIP());
    wifiIsConnected = true;
    resetWiFiBackoff();
  }
  else
  {
    Serial.println("\n⌛ WiFi connect pending (background retries with backoff)...");
    scheduleWiFiReconnect();
  }
}

bool checkWiFiConnectionStatus()
{
  return WiFi.status() == WL_CONNECTED;
}

void displayWiFiConnectionSuccess()
{
  clearDisplayScreenWithColor(BLACK);
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  M5.Display.println("WiFi Connected!");
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y + 20);
  M5.Display.println(WiFi.localIP());
// 長い待機でもWDT/OTAに配慮
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (ENABLE_TASK_WATCHDOG)
    esp_task_wdt_reset();
#endif
  delay(CONNECTION_SUCCESS_DISPLAY_TIME);
}

void synchronizeSystemTimeWithNTP()
{
  Serial.println("🕐 Starting NTP time synchronization...");
  showConnectionStatusMessage("NTP Sync...");
  timeClient.begin();
  bool synchronizationSuccess = attemptNTPTimeSynchronization();
  displayNTPSynchronizationResult(synchronizationSuccess);
  if (synchronizationSuccess)
  {
    lastNtpOkMillis = millis();
  }
}

bool attemptNTPTimeSynchronization()
{
  for (int i = 0; i < MAXIMUM_NTP_RETRY_ATTEMPTS; i++)
  {
    if (timeClient.update())
    {
      Serial.println("✅ NTP Time Synced Successfully.");
      return true;
    }
    timeClient.forceUpdate();
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (ENABLE_TASK_WATCHDOG)
      esp_task_wdt_reset();
#endif
    delay(1000);
    M5.Display.print(".");
    Serial.print(".");
  }
  Serial.println("\n❌ NTP Time Sync Failed.");
  return false;
}

void displayNTPSynchronizationResult(bool wasSuccessful)
{
  clearDisplayScreenWithColor(BLACK);
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  if (wasSuccessful)
  {
    M5.Display.println("NTP Synced!");
    M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y + 20);
    M5.Display.println(timeClient.getFormattedTime());
    Serial.print("   Synced Time: ");
    Serial.println(timeClient.getFormattedTime());
  }
  else
  {
    M5.Display.println("NTP Failed!");
  }
// 長い待機でもWDT/OTAに配慮
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (ENABLE_TASK_WATCHDOG)
    esp_task_wdt_reset();
#endif
  delay(CONNECTION_SUCCESS_DISPLAY_TIME);
}

// ---------------- MQTT ----------------
void configureMQTTConnection()
{
  mqttCommunicationClient.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  mqttCommunicationClient.setCallback(handleIncomingMQTTMessage);
  mqttCommunicationClient.setKeepAlive(MQTT_KEEPALIVE_SECONDS);
  mqttCommunicationClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_SECONDS);
  mqttCommunicationClient.setBufferSize(MQTT_BUFFER_SIZE);
  Serial.println("⚙️ MQTT Connection Configured.");
}

void establishMQTTBrokerConnection()
{
  Serial.println("📡 Attempting to connect to MQTT broker...");
  showConnectionStatusMessage("MQTT connecting...");
  while (!mqttCommunicationClient.connected())
  {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    if (ENABLE_TASK_WATCHDOG)
      esp_task_wdt_reset();
#endif
    String uniqueClientId = generateUniqueMQTTClientId();
    if (attemptMQTTBrokerConnection(uniqueClientId))
    {
      subscribeToMQTTDataTopic();
      displayMQTTConnectionSuccess();
      break;
    }
    else
    {
      displayMQTTConnectionFailure();
      // 過度なスピンを避けるため短い待機
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
      if (ENABLE_TASK_WATCHDOG)
        esp_task_wdt_reset();
#endif
      delay(250);
    }
  }
}

String generateUniqueMQTTClientId()
{
  return String(MQTT_CLIENT_ID_PREFIX) + String(random(0xffff), HEX);
}

bool attemptMQTTBrokerConnection(const String &clientIdentifier)
{
  bool connectionEstablished = false;
  if (MQTT_USE_AUTH)
  {
    connectionEstablished = mqttCommunicationClient.connect(
        clientIdentifier.c_str(),
        MQTT_USERNAME, MQTT_PASSWORD,
        MQTT_LWT_TOPIC, MQTT_LWT_QOS, MQTT_LWT_RETAIN, MQTT_LWT_MESSAGE);
  }
  else
  {
    connectionEstablished = mqttCommunicationClient.connect(
        clientIdentifier.c_str(),
        MQTT_LWT_TOPIC, MQTT_LWT_QOS, MQTT_LWT_RETAIN, MQTT_LWT_MESSAGE);
  }
  if (connectionEstablished)
  {
    Serial.println("✅ MQTT Connection Successful.");
    Serial.print("   Client ID: ");
    Serial.println(clientIdentifier);
    resetMQTTBackoff();
  }
  else
  {
    Serial.print("❌ MQTT Connection Failed, rc=");
    Serial.println(mqttCommunicationClient.state());
  }
  return connectionEstablished;
}

void subscribeToMQTTDataTopic()
{
  mqttCommunicationClient.subscribe(MQTT_TOPIC_NAME);
  Serial.print("📬 Subscribed to MQTT topic: ");
  Serial.println(MQTT_TOPIC_NAME);
  // オンライン通知（LWTの対応として現在の状態を明示）
  mqttCommunicationClient.publish(MQTT_LWT_TOPIC, MQTT_ONLINE_MESSAGE, MQTT_LWT_RETAIN);
}

void displayMQTTConnectionSuccess()
{
  M5.Display.println("MQTT Connected!");
// 長い待機でもWDT/OTAに配慮
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (ENABLE_TASK_WATCHDOG)
    esp_task_wdt_reset();
#endif
  delay(1000);
}

void displayMQTTConnectionFailure()
{
  M5.Display.print("Failed, rc=");
  M5.Display.print(mqttCommunicationClient.state());
  M5.Display.println(" retry");
}

void handleIncomingMQTTMessage(char *topicName, byte *messagePayload, unsigned int messageLength)
{
  String jsonMessageString = convertRawPayloadToString(messagePayload, messageLength);
  Serial.println("\n--- New MQTT Message Received ---");
  Serial.printf("Topic: %s\n", topicName);
  Serial.printf("Payload: '%s'\n", jsonMessageString.c_str());

  if (!validateJSONDataIntegrity(jsonMessageString))
  {
    Serial.println("❌ Invalid JSON data detected.");
    displayJSONParsingError("Invalid JSON");
    return;
  }

  SensorDataPacket parsedSensorData = parseJSONSensorData(jsonMessageString);
  if (parsedSensorData.hasValidData)
  {
    updateCurrentSensorData(parsedSensorData);
    Serial.printf("✅ Sensor data updated: CO2=%d, THI=%.1f\n",
                  parsedSensorData.carbonDioxideLevel, parsedSensorData.thermalComfortIndex);
    refreshEntireDisplay();
  }
  else
  {
    Serial.println("❌ Sensor data parsing failed.");
    displayJSONParsingError("Parse Failed");
  }
  Serial.println("---------------------------------");
}

bool validateJSONDataIntegrity(const String &jsonData)
{
  String trimmedData = jsonData;
  trimmedData.trim();
  if (trimmedData.length() == 0)
    return false;
  if (!trimmedData.startsWith("{"))
    return false;
  if (!trimmedData.endsWith("}"))
    return false;
  return true;
}

String convertRawPayloadToString(byte *rawPayload, unsigned int payloadLength)
{
  String convertedMessage;
  convertedMessage.reserve(payloadLength + 1);
  for (unsigned int i = 0; i < payloadLength; i++)
  {
    if (rawPayload[i] >= 32 && rawPayload[i] <= 126)
    {
      convertedMessage += (char)rawPayload[i];
    }
  }
  return convertedMessage;
}

SensorDataPacket parseJSONSensorData(const String &jsonString)
{
  SensorDataPacket extractedData = {0, 0.0, 0.0, 0.0, "", 0, false};
  DynamicJsonDocument jsonDocument(JSON_PARSING_MEMORY_SIZE);
  DeserializationError parseError = deserializeJson(jsonDocument, jsonString);
  if (parseError)
  {
    Serial.printf("❌ JSON parsing failed: %s\n", parseError.c_str());
    return extractedData;
  }
  if (jsonDocument.containsKey("co2"))
    extractedData.carbonDioxideLevel = jsonDocument["co2"];
  if (jsonDocument.containsKey("thi"))
    extractedData.thermalComfortIndex = jsonDocument["thi"];
  if (jsonDocument.containsKey("temperature"))
    extractedData.ambientTemperature = jsonDocument["temperature"];
  if (jsonDocument.containsKey("humidity"))
    extractedData.relativeHumidity = jsonDocument["humidity"];
  if (jsonDocument.containsKey("comfort_level"))
    extractedData.comfortLevelDescription = jsonDocument["comfort_level"].as<String>();
  if (jsonDocument.containsKey("timestamp"))
    extractedData.dataTimestamp = jsonDocument["timestamp"];
  extractedData.hasValidData = true;
  return extractedData;
}

void updateCurrentSensorData(const SensorDataPacket &newSensorData)
{
  currentSensorReading = newSensorData;
}

void maintainMQTTBrokerConnection()
{
  if (!mqttCommunicationClient.connected())
  {
    Serial.println("⚠️ MQTT connection lost. Reconnecting...");
    // Backoff 管理された再接続（非ブロッキング）
    ensureMQTTConnection();
  }
}

void processIncomingMQTTMessages()
{
  mqttCommunicationClient.loop();
}

void updateSystemNetworkTime()
{
  bool ok = timeClient.update();
  if (ok)
  {
    lastNtpOkMillis = millis();
  }
  else
  {
    // 一定時間 NTP 成功がない場合は強制更新
    unsigned long now = millis();
    if (now - lastNtpOkMillis > TIME_MAX_FORCE_RESYNC_MILLISECONDS)
    {
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
      if (ENABLE_TASK_WATCHDOG)
        esp_task_wdt_reset();
#endif
      bool ok2 = timeClient.forceUpdate();
      if (ok2)
      {
        lastNtpOkMillis = now;
      }
    }
  }
}

// ---------------- ユーティリティ ----------------
void showConnectionStatusMessage(const char *statusMessage)
{
  clearDisplayScreenWithColor(BLACK);
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  M5.Display.println(statusMessage);
}

void clearDisplayScreenWithColor(uint16_t backgroundColor)
{
  M5.Display.fillScreen(backgroundColor);
}

void printMQTTSubscriptionDebugInfo()
{
  Serial.println("--- MQTT Subscription Status ---");
  Serial.printf("Broker: %s:%d\n", MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  Serial.printf("Topic: %s\n", MQTT_TOPIC_NAME);
  Serial.printf("Connected: %s\n", mqttCommunicationClient.connected() ? "Yes" : "No");
  Serial.printf("Client State Code: %d\n", mqttCommunicationClient.state());
  Serial.println("------------------------------");
}

void displaySensorDataOrErrorMessage()
{ /* legacy */
}

// ---------------- NEW: 重力ベースの自動回転 ----------------
void autoRotateDisplayByGravity()
{
  if (!ENABLE_GRAVITY_AUTO_ROTATE)
    return;

  unsigned long now = millis();
  if (now - lastOrientationCheckTime < ORIENTATION_CHECK_INTERVAL_MS)
    return;
  lastOrientationCheckTime = now;

  // IMU更新（Plus2: StickCP2.Imu）
  bool imu_update = StickCP2.Imu.update();
  if (!imu_update)
    return;

  auto data = StickCP2.Imu.getImuData();
  float ax = data.accel.x;
  float ay = data.accel.y;
  float az = data.accel.z;

  // ローパス
  if (!lp_init)
  {
    lp_ax = ax;
    lp_ay = ay;
    lp_az = az;
    lp_init = true;
  }
  else
  {
    lp_ax = lp_ax * 0.9f + ax * 0.1f;
    lp_ay = lp_ay * 0.9f + ay * 0.1f;
    lp_az = lp_az * 0.9f + az * 0.1f;
  }

  // XY平面で十分に傾いていない場合は判定しない（机にベタ置き時の誤判定防止）
  float xy = sqrtf(lp_ax * lp_ax + lp_ay * lp_ay);
  if (xy < ORIENTATION_TILT_THRESHOLD_G)
    return;

  // X軸の符号で 0° / 180° を判定（必要なら config.h で反転可）
  float xsel = ORIENTATION_INVERT_X ? -lp_ax : lp_ax;
  int desired = (xsel >= 0.0f) ? DISPLAY_ROTATION_NORMAL : DISPLAY_ROTATION_FLIPPED;

  if (desired != currentDisplayRotation)
  {
    if (fabsf(xsel) > (ORIENTATION_TILT_THRESHOLD_G + ORIENTATION_HYSTERESIS_G))
    {
      currentDisplayRotation = desired;
      M5.Display.setRotation(currentDisplayRotation);
      refreshEntireDisplay(); // 回転時は再描画
      Serial.printf("🔄 AutoRotate: rotation=%d (ax=%.2f, ay=%.2f, az=%.2f)\n",
                    currentDisplayRotation, ax, ay, az);
    }
  }
}

// ---------------- 24/7: WiFi/MQTT/OTA/WDT 補助 ----------------
void onWiFiEvent(WiFiEvent_t event)
{
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    Serial.println("[WiFi] Connected to AP");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    wifiIsConnected = true;
    resetWiFiBackoff();
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("[WiFi] Disconnected");
    wifiIsConnected = false;
    scheduleWiFiReconnect();
    break;
  default:
    break;
  }
}

void ensureWiFiConnection()
{
  if (checkWiFiConnectionStatus())
    return;
  unsigned long now = millis();
  if (now >= wifiNextReconnectAt)
  {
    Serial.println("[WiFi] Reconnect attempt...");
    WiFi.disconnect();
    WiFi.begin(WIFI_NETWORK_NAME, WIFI_NETWORK_PASSWORD);
    // 次回までのバックオフを増やす
    wifiBackoffMs = min(wifiBackoffMs * 2, WIFI_RETRY_BACKOFF_MAX_MS);
    wifiNextReconnectAt = now + wifiBackoffMs;
  }
}

void scheduleWiFiReconnect()
{
  unsigned long now = millis();
  if (wifiNextReconnectAt == 0 || now + WIFI_RETRY_BACKOFF_INITIAL_MS < wifiNextReconnectAt)
  {
    wifiNextReconnectAt = now + wifiBackoffMs;
  }
}

void resetWiFiBackoff()
{
  wifiBackoffMs = WIFI_RETRY_BACKOFF_INITIAL_MS;
  wifiNextReconnectAt = millis() + wifiBackoffMs;
}

void ensureMQTTConnection()
{
  if (!checkWiFiConnectionStatus())
    return; // WiFiが先
  if (mqttCommunicationClient.connected())
    return;
  unsigned long now = millis();
  if (now >= mqttNextReconnectAt)
  {
    String uniqueClientId = generateUniqueMQTTClientId();
    if (attemptMQTTBrokerConnection(uniqueClientId))
    {
      subscribeToMQTTDataTopic();
      displayMQTTConnectionSuccess();
    }
    else
    {
      // バックオフ増大
      mqttBackoffMs = min(mqttBackoffMs * 2, MQTT_RETRY_BACKOFF_MAX_MS);
      mqttNextReconnectAt = now + mqttBackoffMs;
      Serial.printf("[MQTT] Next retry in %lums\n", mqttBackoffMs);
    }
  }
}

void resetMQTTBackoff()
{
  mqttBackoffMs = MQTT_RETRY_BACKOFF_INITIAL_MS;
  mqttNextReconnectAt = millis() + mqttBackoffMs;
}

void setupTaskWatchdog()
{
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (!ENABLE_TASK_WATCHDOG)
    return;
  esp_task_wdt_config_t twdt_cfg = {};
  twdt_cfg.timeout_ms = WDT_TIMEOUT_SECONDS * 1000;
  twdt_cfg.trigger_panic = true;
  esp_task_wdt_init(&twdt_cfg);
  esp_task_wdt_add(NULL); // current task (loop task)
  Serial.printf("🛡️  Task WDT enabled: %ds\n", WDT_TIMEOUT_SECONDS);
#endif
}

void setupOTA()
{
// ここでは簡易設定。必要ならホスト名/パスワードなどを追加可能。
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  ArduinoOTA.setHostname("M5StickCPlus2");
  // セキュリティ（任意）：Arduino IDE 側の設定に合わせる
  // ArduinoOTA.setPassword("your_ota_password");

  ArduinoOTA.onStart([]()
                     { Serial.println("[OTA] Start"); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("[OTA] End"); });
  ArduinoOTA.onError([](ota_error_t error)
                     { Serial.printf("[OTA] Error[%u]\n", error); });
  ArduinoOTA.begin();
  Serial.println("📡 OTA ready (Arduino IDE -> Network Ports)。");
#endif
}

void handleOTA()
{
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  ArduinoOTA.handle();
#endif
}

void updateStatusOverlay()
{
  if (!ENABLE_STATUS_OVERLAY)
    return;
  int16_t x = 0, y = 0;
  uint16_t w = 0, h = 0;
  String status = String("W:") + (checkWiFiConnectionStatus() ? "OK" : "NG") +
                  String(" M:") + (mqttCommunicationClient.connected() ? "OK" : "NG");
  status += String(" H:") + (uint32_t)ESP.getFreeHeap();

  // 右上に小さく上書き
  M5.Display.setTextSize(1);
  M5.Display.setTextColor(0x7BEF); // 50% gray in RGB565
  M5.Display.setTextDatum(TR_DATUM);
  M5.Display.drawString(status, M5.Display.width() - 2, 12);
  M5.Display.setTextDatum(TL_DATUM);
}

String formatUptime()
{
  unsigned long s = (millis() - bootMillis) / 1000;
  unsigned long d = s / 86400;
  s %= 86400;
  unsigned long h = s / 3600;
  s %= 3600;
  unsigned long m = s / 60;
  s %= 60;
  char buf[32];
  snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", d, h, m, s);
  return String(buf);
}
