/**
 * @file M5StickCPlus2_DigiClock_MQTT.ino
 * @brief M5StickCPlus2とDigi-Clock Unitを使った、MQTTセンサーモニター兼NTPデジタル時計
 * @details
 * このプログラムは、2つの役割を同時にこなします。
 * 1. M5StickCPlus2本体のカラー液晶画面に、MQTTプロトコルで受信したセンサー情報（CO2濃度など）を表示します。
 * 2. Groveポートに接続した外部の7セグメントLED（Digi-Clock Unit）に、インターネット経由で取得した
 * 正確な日本時刻を「HH:MM」形式（24時間表記）で安定して表示します。
 *
 * Wi-Fiの接続情報などは、プライバシー保護のため、同じフォルダにある `config.h` という別のファイルに記述されています。
 *
 * @author omiya-bonsai
 * @date 2025-07-09
 */

// =================================================================
// 1. ライブラリと設定ファイルの読み込み
// =================================================================
// 機能を拡張するための「ライブラリ」と呼ばれる、便利なプログラム部品を読み込みます。

#include <M5StickCPlus2.h>      // M5StickCPlus2本体の機能（画面、電源など）を簡単に使うためのライブラリ
#include <WiFi.h>               // Wi-Fi機能を使うためのライブラリ。インターネットに接続するために必要です
#include <PubSubClient.h>       // MQTTという通信方法を使うためのライブラリ。MQTTはIoT機器同士で軽量なデータをやり取りするための規格です
#include <ArduinoJson.h>        // JSONというデータ形式を簡単に扱うためのライブラリ。JSONは構造化されたデータを扱うための標準的な形式です
#include <NTPClient.h>          // NTPという、時刻を合わせるための通信方法を使うライブラリ。NTPはインターネット経由で正確な時刻を取得する仕組みです
#include <WiFiUdp.h>            // NTP通信の基礎となるUDP通信を使うためのライブラリ。UDPはインターネット上でデータを送受信する方式の一つです
#include <Wire.h>               // I2C通信（Groveポート）を使うための標準ライブラリ（明示的に追加）
#include "config.h"             // Wi-FiやMQTTの接続情報など、個人情報を記述した設定ファイルを読み込みます
#include <M5UNIT_DIGI_CLOCK.h>  // M5Stackの「Digi-Clock Unit」を制御するための専用ライブラリ。7セグメントLEDの表示を制御します

// =================================================================
// 2. データ構造体の定義
// =================================================================
/**
 * @brief MQTTで受信するセンサーデータをまとめて管理するための「設計図」
 * @details
 * 構造体（struct）とは、関連するデータを一つのまとまりとして扱うための「箱」のようなものです。
 * この構造体では、センサーから取得した様々な種類のデータを一つの変数にまとめて管理します。
 */
struct SensorDataPacket {
  int carbonDioxideLevel;          // CO2濃度（ppm単位）- 部屋の空気の質を示す重要な指標
  float thermalComfortIndex;       // 温熱快適性指数（THI）- 温度と湿度から算出される快適さの指標
  float ambientTemperature;        // 環境温度（℃）- センサーで測定した周囲の温度
  float relativeHumidity;          // 相対湿度（%）- センサーで測定した空気中の湿度
  String comfortLevelDescription;  // 快適レベルの説明文（「快適」「やや暑い」など）
  unsigned long dataTimestamp;     // データのタイムスタンプ - このデータがいつ測定されたか
  bool hasValidData;               // 有効なデータかどうかのフラグ - trueなら有効、falseなら無効
};

// =================================================================
// 3. グローバル変数の定義
// =================================================================
// プログラムの様々な場所から参照・変更される変数を「グローバル変数」としてここで宣言します。
// グローバル変数は、プログラム全体から見える「共有変数」のようなものです。

// --- ネットワーク関連 ---
WiFiUDP networkUdpClient;  // UDP通信のためのオブジェクト。NTPサーバーとの通信に使用します
NTPClient timeClient(networkUdpClient, TIME_SERVER_ADDRESS, JAPAN_TIME_OFFSET_SECONDS, TIME_UPDATE_INTERVAL_MILLISECONDS);
// NTPクライアント。インターネットから正確な時刻を取得するために使います
// 引数: UDP通信用クライアント、NTPサーバーアドレス、タイムゾーンオフセット（日本は+9時間=32400秒）、更新間隔
WiFiClient networkWifiClient;  // WiFi接続のためのクライアントオブジェクト。MQTTクライアントの基盤として使用します
PubSubClient mqttCommunicationClient(networkWifiClient);
// MQTTクライアント。センサーデータを受信するために使用します
// 引数: WiFiクライアントオブジェクト

// --- センサーデータ関連 ---
SensorDataPacket currentSensorReading = { 0, 0.0, 0.0, 0.0, "", 0, false };
// 現在のセンサー読み取り値を保存する変数。初期値はすべてゼロまたは空で、データ無効フラグ

// --- 表示制御関連 ---
unsigned long lastDisplayUpdateTime = 0;       // 最後に画面を更新した時刻（ミリ秒）- 定期的な画面更新の管理に使用
unsigned long lastInteractiveDisplayTime = 0;  // 最後にインタラクティブ表示を更新した時刻（ミリ秒）
bool displayCO2 = true;                        // 表示モード切替用フラグ: trueならCO2濃度表示、falseならTHI（熱快適性指数）表示

// --- Digi-Clock Unit 関連 ---
M5UNIT_DIGI_CLOCK digi_clock;    // Digi-Clock Unitを制御するためのオブジェクト
int last_digiclock_minute = -1;  // 最後にDigi-Clockに表示した「分」を記憶する変数（チラツキ防止用）
// -1で初期化することで、最初の更新を確実に行わせます

// =================================================================
// 4. 関数の前方宣言
// =================================================================
// C++では、関数は呼び出される前に「こういう名前の関数がありますよ」と宣言しておく必要があります。
// 前方宣言をすることで、関数の実装が後にあっても、コンパイラはその関数の存在を認識できます。

// ディスプレイ関連の関数
void initializeDisplaySystem();                               // M5StickCPlus2の画面を初期化する関数
void showSystemStartupMessage();                              // システム起動メッセージを表示する関数
void displayWiFiConnectionSuccess();                          // WiFi接続成功時のメッセージを表示
void displayNTPSynchronizationResult(bool wasSuccessful);     // NTP同期結果を表示
void displayMQTTConnectionSuccess();                          // MQTT接続成功時のメッセージを表示
void displayMQTTConnectionFailure();                          // MQTT接続失敗時のメッセージを表示
void refreshEntireDisplay();                                  // 画面全体を更新
void updateDisplayIfIntervalElapsed();                        // 一定時間経過後に画面を更新
void displayApplicationTitle();                               // アプリケーションのタイトルを表示
void displayCurrentSystemTime();                              // 現在のシステム時刻を表示
void displaySensorDataOrErrorMessage();                       // センサーデータまたはエラーメッセージを表示
void displayCO2ConcentrationData();                           // CO2濃度データを表示
void displayTHIComfortData();                                 // 温熱快適性指数を表示
void displayNoDataAvailableMessage();                         // データがない時のメッセージを表示
void displayNetworkConnectionStatus();                        // ネットワーク接続状態を表示
void displayJSONParsingError(const char *errorDescription);   // JSONパースエラーを表示
void showConnectionStatusMessage(const char *statusMessage);  // 接続状態メッセージを表示
void clearDisplayScreenWithColor(uint16_t backgroundColor);   // 画面を指定色でクリア

// WiFi関連の関数
void establishWiFiConnection();    // WiFi接続を確立
bool checkWiFiConnectionStatus();  // WiFi接続状態をチェック

// NTP（時刻同期）関連の関数
void synchronizeSystemTimeWithNTP();   // NTPサーバーと時刻を同期
bool attemptNTPTimeSynchronization();  // NTP同期を試みる
void updateSystemNetworkTime();        // システム時刻を更新

// MQTT（データ通信）関連の関数
void configureMQTTConnection();                                                                     // MQTT接続を設定
void establishMQTTBrokerConnection();                                                               // MQTTブローカーへの接続を確立
String generateUniqueMQTTClientId();                                                                // 一意のMQTTクライアントIDを生成
bool attemptMQTTBrokerConnection(const String &clientIdentifier);                                   // MQTTブローカー接続を試みる
void subscribeToMQTTDataTopic();                                                                    // MQTTトピックをサブスクライブ
void handleIncomingMQTTMessage(char *topicName, byte *messagePayload, unsigned int messageLength);  // 受信したMQTTメッセージを処理
bool validateJSONDataIntegrity(const String &jsonData);                                             // JSONデータの整合性を検証
String convertRawPayloadToString(byte *rawPayload, unsigned int payloadLength);                     // 生のペイロードを文字列に変換
struct SensorDataPacket parseJSONSensorData(const String &jsonString);                              // JSONからセンサーデータを解析
void updateCurrentSensorData(const SensorDataPacket &newSensorData);                                // 現在のセンサーデータを更新
void maintainMQTTBrokerConnection();                                                                // MQTT接続を維持
void processIncomingMQTTMessages();                                                                 // 受信したMQTTメッセージを処理
void printMQTTSubscriptionDebugInfo();                                                              // MQTTサブスクリプションのデバッグ情報を表示

// Digi-Clock Unit関連の関数
void initializeDigiClock();     // Digi-Clock Unitを初期化
void updateDigiClockDisplay();  // Digi-Clock Unitの表示を更新

// =================================================================
// 5. メインの初期化関数 (setup)
// =================================================================
/**
 * @brief セットアップ関数。マイコンの電源が入った時に一度だけ実行されます。
 * @details 各種ハードウェアや通信機能の初期化を順番に行います。
 * Arduinoプログラムでは、この関数が電源投入時やリセット後に最初に一度だけ実行されます。
 */
void setup() {
  // PCとの通信（シリアルモニタ）を開始。デバッグ情報の表示に使う。
  // 115200はボーレート（通信速度）を表し、この値が大きいほど通信が速くなります
  Serial.begin(115200);
  Serial.println("\n========== M5StickCPlus2 & Digi-Clock Monitor 起動 ==========");

  // Step 1: M5StickCPlus2本体のディスプレイを初期化
  // ディスプレイに何かを表示するには、まず初期化が必要です
  initializeDisplaySystem();
  showSystemStartupMessage();

  // Step 2: 外部接続したDigi-Clock Unitを初期化
  // 外部の7セグメントLEDディスプレイを使えるようにします
  initializeDigiClock();

  // Step 3: Wi-Fiネットワークへの接続
  // インターネットにアクセスするために必要です
  establishWiFiConnection();

  // Step 4: インターネット上の時刻サーバーと時刻を同期
  // 正確な時刻を取得するために、NTPサーバーと通信します
  synchronizeSystemTimeWithNTP();

  // Step 5: MQTT通信の準備とサーバーへの接続
  // センサーデータを受信するための通信設定をします
  configureMQTTConnection();
  establishMQTTBrokerConnection();

  // Step 6: 全ての準備が整ったので、メインの表示画面を描画
  // 初期画面を表示します
  refreshEntireDisplay();

  Serial.println("========== 初期化処理完了：システム稼働開始 ==========");
}

// =================================================================
// 6. メインのループ関数 (loop)
// =================================================================
/**
 * @brief メインループ関数。setup()の実行後、電源が切れるまでずっと繰り返し実行されます。
 * @details このループ内で、通信の監視や画面の更新など、定期的に行う必要がある処理を実行します。
 * Arduinoプログラムでは、setup()の後にこの関数が無限に繰り返し実行され続けます。
 */
void loop() {
  // 1. MQTTサーバーとの接続が切れていないか確認し、切れていたら再接続する
  // 通信が不安定な場合に、自動的に再接続するための処理です
  maintainMQTTBrokerConnection();

  // 2. MQTTサーバーから新しいメッセージが届いていないか確認し、届いていれば処理する
  // センサーから送られてくるデータを受信するための処理です
  processIncomingMQTTMessages();

  // 3. M5StickCPlus2本体の画面を、一定時間ごとに更新する（CO2とTHIの交互表示）
  // 画面に表示する内容を定期的に切り替えるための処理です
  updateDisplayIfIntervalElapsed();

  // 4. NTP時刻を、内部で定期的に更新する
  // 時計の精度を保つために、定期的に正確な時刻を取得します
  updateSystemNetworkTime();

  // 5. Digi-Clock Unitの時刻表示を、必要に応じて更新する
  // 外部の7セグメントLEDの表示を更新します
  updateDigiClockDisplay();

  // 6. 次のループまで少し待機する（CPUを少し休ませて、消費電力を抑える）
  // 連続して処理を行うとCPUが過熱したり、電力を無駄に消費するため、
  // 短い時間休ませることで効率的な動作を実現します
  delay(MAIN_LOOP_DELAY_MILLISECONDS);  // (この値はconfig.hで定義)
}

// =================================================================
// 7. 各関数の具体的な処理内容
// =================================================================

// -----------------------------------------------------------------
// Digi-Clock Unit関連の関数
// -----------------------------------------------------------------

/**
 * @brief Digi-Clock Unitを初期化する
 * @details Groveポート経由でI2C通信を開始し、7セグメントLEDディスプレイの設定を行います
 */
void initializeDigiClock() {
  // M5StickCPlus2のGroveポート(G32, G33)でI2C通信を開始
  // I2Cはシリアル通信の一種で、少ない配線で複数のデバイスと通信できるプロトコルです
  // 引数はI2Cで使用するSDA（データ線）とSCL（クロック線）のピン番号です
  Wire.begin(32, 33);
  Serial.println("⚙️  I2C for Digi-Clock Unit starting...");

  // ユニットの初期化を試み、失敗した場合はエラーメッセージを表示する
  // begin()メソッドはデバイスとの通信が成功すればtrue、失敗すればfalseを返します
  if (!digi_clock.begin(&Wire)) {
    // 初期化失敗時の処理
    Serial.println("❌ Digi-Clock Unit not found!");
    // 本体画面にもエラーを表示
    M5.Display.setCursor(10, 50);         // テキストの開始位置を設定（x=10, y=50）
    M5.Display.setTextColor(RED);         // テキスト色を赤に設定
    M5.Display.println("DigiClock ERR");  // エラーメッセージを表示
    delay(2000);                          // 2秒間エラーを表示（ユーザーが確認できる時間を確保）
  } else {
    // 初期化成功時の処理
    Serial.println("✅ Digi-Clock Unit found and initialized.");
    digi_clock.setBrightness(80);  // 明るさを設定 (0-100の範囲で指定)
    digi_clock.setString("----");  // 起動時はハイフンを表示しておく（時刻が取得できるまでの一時表示）
  }
}

/**
 * @brief Digi-Clock Unitの時刻表示を更新する（チラツキ防止・安定版）
 * @details NTPで取得した正確な時刻を7セグメントLEDに表示します。
 * ディスプレイのチラツキを防ぐため、分が変わった時だけ更新します。
 */
void updateDigiClockDisplay() {
  // NTPで時刻が正しく同期されている場合のみ、処理を実行
  // 2023年1月1日のUNIXタイムスタンプは1672531200なので、それより大きければ正しい時刻とみなす
  if (timeClient.getEpochTime() > 1672531200) {  // 2023年以降の時刻ならOK

    // 現在の「分」を取得
    int minute = timeClient.getMinutes();

    // 「分」が変わった時にだけ、ディスプレイの表示を更新する
    // これにより、毎ループでの更新によるチラツキを防止します
    if (minute != last_digiclock_minute) {
      // 現在の「時」を取得
      int hour = timeClient.getHours();

      // 時刻を格納するための文字列バッファ
      char time_string[6];  // "HH:MM" + 終端文字('\\0')で5+1=6文字必要

      // HH:MM形式でコロンを常時点灯させる
      // sprintf関数で、数値を指定したフォーマットの文字列に変換
      // %02d は「2桁の数値、1桁の場合は0で埋める」という指定
      sprintf(time_string, "%02d:%02d", hour, minute);

      // 7セグメントLEDに時刻文字列を設定
      digi_clock.setString(time_string);

      // 更新した「分」の値を記憶しておく（次回の比較用）
      last_digiclock_minute = minute;
    }
  }
}

// -----------------------------------------------------------------
// M5StickCPlus2 本体画面関連の関数
// -----------------------------------------------------------------

/**
 * @brief M5StickCPlus2のディスプレイシステムを初期化する
 * @details 画面の向きや背景色、テキストサイズなど基本的な設定を行います
 */
void initializeDisplaySystem() {
  // M5StickCPlus2の全機能を初期化
  M5.begin();

  // ディスプレイの向きを設定（1=横向き、時計回りに90度回転）
  M5.Display.setRotation(1);

  // 画面を黒で塗りつぶして初期化
  clearDisplayScreenWithColor(BLACK);

  // テキストの色を白に設定
  M5.Display.setTextColor(WHITE);

  // テキストサイズを2倍に設定（デフォルトより大きく表示）
  M5.Display.setTextSize(2);

  Serial.println("✅ M5StickCPlus2 Display Initialized.");
}

/**
 * @brief システム起動時のメッセージを表示
 * @details 初期化中であることをユーザーに知らせるためのメッセージを表示します
 */
void showSystemStartupMessage() {
  // 画面を黒でクリア
  clearDisplayScreenWithColor(BLACK);

  // テキストのカーソル位置を設定
  // TITLE_POSITION_XとTITLE_POSITION_Yはconfig.hで定義された定数
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);

  // 「Starting...」というメッセージを表示
  M5.Display.println("Starting...");

  Serial.println("📱 Displaying startup message.");
}

/**
 * @brief 画面全体を更新する
 * @details タイトル、時刻、ネットワーク状態、センサーデータなど全ての表示要素を更新します
 */
void refreshEntireDisplay() {
  // まず画面を黒でクリア
  clearDisplayScreenWithColor(BLACK);

  // アプリケーションのタイトルを表示
  displayApplicationTitle();

  // 現在の時刻を表示
  displayCurrentSystemTime();

  // ネットワーク接続状態を表示
  displayNetworkConnectionStatus();

  // センサーデータが有効な場合
  if (currentSensorReading.hasValidData) {
    // 表示モードに応じてCO2濃度かTHIを表示
    if (displayCO2) {
      displayCO2ConcentrationData();
    } else {
      displayTHIComfortData();
    }
  } else {
    // 有効なデータがない場合はエラーメッセージ
    displayNoDataAvailableMessage();
  }
}

/**
 * @brief 一定時間が経過したら画面表示を更新する
 * @details CO2濃度と快適性指数（THI）を交互に表示するための関数です
 */
void updateDisplayIfIntervalElapsed() {
  // 現在の時刻（プログラム起動からの経過ミリ秒）を取得
  unsigned long currentSystemTime = millis();

  // 前回の表示更新から指定時間が経過しているかチェック
  // INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDSはconfig.hで定義された定数
  if (currentSystemTime - lastInteractiveDisplayTime >= INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS) {
    // 画面を黒でクリア
    clearDisplayScreenWithColor(BLACK);

    // タイトル、時刻、ネットワーク状態を表示
    displayApplicationTitle();
    displayCurrentSystemTime();
    displayNetworkConnectionStatus();

    // センサーデータが有効な場合
    if (currentSensorReading.hasValidData) {
      // 表示モードに応じてCO2濃度かTHIを表示
      if (displayCO2) {
        displayCO2ConcentrationData();
      } else {
        displayTHIComfortData();
      }
      // 次回表示モードを切り替え（交互表示）
      displayCO2 = !displayCO2;
    } else {
      // 有効なデータがない場合はエラーメッセージ
      displayNoDataAvailableMessage();
    }

    // 最終更新時刻を記録
    lastInteractiveDisplayTime = currentSystemTime;
  }
}

/**
 * @brief アプリケーションのタイトルを画面に表示
 */
void displayApplicationTitle() {
  // テキストサイズを小さく設定（1）
  M5.Display.setTextSize(1);

  // テキストの色をシアン（水色）に設定
  M5.Display.setTextColor(CYAN);

  // テキストのカーソル位置を設定
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);

  // タイトル「Sensor Monitor」を表示
  M5.Display.println("Sensor Monitor");
}

/**
 * @brief 現在のシステム時刻を画面に表示
 * @details NTPから取得した時刻を表示します
 */
void displayCurrentSystemTime() {
  // テキストの色を白に設定
  M5.Display.setTextColor(WHITE);

  // テキストのカーソル位置を設定
  // TIME_DISPLAY_XとTIME_DISPLAY_Yはconfig.hで定義された定数
  M5.Display.setCursor(TIME_DISPLAY_X, TIME_DISPLAY_Y);

  // NTPクライアントから取得した時刻を「HH:MM:SS」形式で表示
  M5.Display.println(timeClient.getFormattedTime());
}

/**
 * @brief ネットワーク接続状態を画面に表示
 * @details MQTT接続の状態（成功/失敗）を色分けして表示します
 */
void displayNetworkConnectionStatus() {
  // テキストサイズを小さく設定（1）
  M5.Display.setTextSize(1);

  // MQTT接続状態に応じてテキスト色を設定（接続成功=緑、失敗=赤）
  M5.Display.setTextColor(mqttCommunicationClient.connected() ? GREEN : RED);

  // テキストのカーソル位置を設定
  // CONNECTION_STATUS_XとCONNECTION_STATUS_Yはconfig.hで定義された定数
  M5.Display.setCursor(CONNECTION_STATUS_X, CONNECTION_STATUS_Y);

  // MQTT接続状態に応じたメッセージを表示
  M5.Display.println(mqttCommunicationClient.connected() ? "MQTT:OK" : "MQTT:NG");
}

/**
 * @brief CO2濃度データを大きく画面に表示
 * @details 現在のCO2濃度の値を目立つように表示します
 */
void displayCO2ConcentrationData() {
  // 「CO2:」ラベルの表示設定
  M5.Display.setTextSize(2);                           // テキストサイズ：中
  M5.Display.setTextColor(GREEN);                      // 色：緑
  M5.Display.setCursor(LARGE_LABEL_X, LARGE_LABEL_Y);  // 位置設定
  M5.Display.println("CO2:");                          // ラベル表示

  // CO2濃度値の表示設定
  M5.Display.setTextSize(8);       // テキストサイズ：大
  M5.Display.setTextColor(GREEN);  // 色：緑

  // テキスト揃えを右寄せに設定（値を右端に揃えるため）
  M5.Display.setTextDatum(TR_DATUM);

  // CO2濃度値を文字列に変換
  String co2Value = String(currentSensorReading.carbonDioxideLevel);

  // CO2値を画面の右側に表示（右マージンを考慮）
  M5.Display.drawString(co2Value, M5.Display.width() - DISPLAY_RIGHT_MARGIN, LARGE_VALUE_Y);

  // テキスト揃えを元の左揃えに戻す
  M5.Display.setTextDatum(TL_DATUM);
}

/**
 * @brief 温熱快適性指数（THI）を大きく画面に表示
 * @details 現在のTHI値と快適レベルを目立つように表示します
 */
void displayTHIComfortData() {
  // 「THI:」ラベルの表示設定
  M5.Display.setTextSize(2);                           // テキストサイズ：中
  M5.Display.setTextColor(ORANGE);                     // 色：オレンジ
  M5.Display.setCursor(LARGE_LABEL_X, LARGE_LABEL_Y);  // 位置設定
  M5.Display.println("THI:");                          // ラベル表示

  // THI値の表示設定
  M5.Display.setTextSize(8);        // テキストサイズ：大
  M5.Display.setTextColor(ORANGE);  // 色：オレンジ

  // テキスト揃えを右寄せに設定
  M5.Display.setTextDatum(TR_DATUM);

  // THI値を小数点1桁まで表示する文字列に変換
  String thiValue = String(currentSensorReading.thermalComfortIndex, 1);

  // THI値を画面の右側に表示
  M5.Display.drawString(thiValue, M5.Display.width() - DISPLAY_RIGHT_MARGIN, LARGE_VALUE_Y);

  // テキスト揃えを元の左揃えに戻す
  M5.Display.setTextDatum(TL_DATUM);
}

/**
 * @brief データが利用できない場合のメッセージを表示
 * @details センサーデータがまだ受信されていない場合などに表示します
 */
void displayNoDataAvailableMessage() {
  // テキストサイズを中くらいに設定
  M5.Display.setTextSize(2);

  // テキスト色を赤に設定（警告色）
  M5.Display.setTextColor(RED);

  // テキスト位置を設定
  // NO_DATA_MESSAGE_XとNO_DATA_MESSAGE_Yはconfig.hで定義された定数
  M5.Display.setCursor(NO_DATA_MESSAGE_X, NO_DATA_MESSAGE_Y);

  // 「データなし」メッセージを表示
  M5.Display.println("No Data");
}

/**
 * @brief JSONデータの解析エラーを表示
 * @param errorDescription エラー内容の説明文
 * @details JSON形式のセンサーデータを解析できなかった場合のエラー表示
 */
void displayJSONParsingError(const char *errorDescription) {
  // 画面を黒でクリア
  clearDisplayScreenWithColor(BLACK);

  // 基本的な情報（タイトル、時刻、接続状態）を表示
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

  // エラーメッセージを表示
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(RED);
  M5.Display.setCursor(20, 50 + VERTICAL_OFFSET);
  M5.Display.println("JSON Error");

  // エラーの詳細説明を小さいサイズで表示
  M5.Display.setTextSize(1);
  M5.Display.setCursor(20, 80 + VERTICAL_OFFSET);
  M5.Display.println(errorDescription);
}

// -----------------------------------------------------------------
// ネットワーク関連の関数
// -----------------------------------------------------------------

/**
 * @brief WiFi接続を確立する
 * @details config.hに設定されているSSIDとパスワードを使用してWiFiに接続します
 */
void establishWiFiConnection() {
  Serial.println("🌐 Attempting to connect to WiFi...");

  // 接続中メッセージを表示
  showConnectionStatusMessage("WiFi connecting...");

  // WiFi接続を開始（config.hで定義されたSSIDとパスワードを使用）
  WiFi.begin(WIFI_NETWORK_NAME, WIFI_NETWORK_PASSWORD);

  // 接続が完了するまで待機
  while (!checkWiFiConnectionStatus()) {
    delay(500);             // 0.5秒待機
    M5.Display.print(".");  // 画面にドットを表示して進行状況を示す
    Serial.print(".");      // シリアルにも同様に表示
  }

  // 接続成功時の表示
  displayWiFiConnectionSuccess();

  Serial.println("\\n✅ WiFi Connection Successful.");
  Serial.print("   IP Address: ");
  Serial.println(WiFi.localIP());  // 割り当てられたIPアドレスを表示
}

/**
 * @brief WiFi接続状態をチェックする
 * @return 接続されていればtrue、そうでなければfalse
 */
bool checkWiFiConnectionStatus() {
  // WiFi.status()がWL_CONNECTED（接続済み）かどうかをチェック
  return WiFi.status() == WL_CONNECTED;
}

/**
 * @brief WiFi接続成功時のメッセージを表示
 */
void displayWiFiConnectionSuccess() {
  // 画面をクリア
  clearDisplayScreenWithColor(BLACK);

  // 接続成功メッセージを表示
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);
  M5.Display.println("WiFi Connected!");

  // 割り当てられたIPアドレスを表示
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y + 20);
  M5.Display.println(WiFi.localIP());

  // メッセージを一定時間表示（ユーザーが確認できる時間）
  // CONNECTION_SUCCESS_DISPLAY_TIMEはconfig.hで定義された表示時間（ミリ秒）
  delay(CONNECTION_SUCCESS_DISPLAY_TIME);
}

/**
 * @brief NTP時刻同期を行う
 * @details インターネット上の時刻サーバー（NTPサーバー）から正確な時刻を取得し、内部時計を合わせます
 */
void synchronizeSystemTimeWithNTP() {
  Serial.println("🕐 Starting NTP time synchronization...");

  // NTP同期中メッセージを表示
  showConnectionStatusMessage("NTP Sync...");

  // NTPクライアントを開始
  timeClient.begin();

  // 時刻同期を試行
  bool synchronizationSuccess = attemptNTPTimeSynchronization();

  // 同期結果を表示
  displayNTPSynchronizationResult(synchronizationSuccess);
}

/**
 * @brief NTP時刻同期を試みる
 * @return 同期成功ならtrue、失敗ならfalse
 * @details 複数回の試行を行い、タイムアウトを処理します
 */
bool attemptNTPTimeSynchronization() {
  // 設定された最大試行回数まで同期を試みる
  // MAXIMUM_NTP_RETRY_ATTEMPTSはconfig.hで定義された定数
  for (int i = 0; i < MAXIMUM_NTP_RETRY_ATTEMPTS; i++) {
    // timeClient.update()が成功（true）を返したら
    if (timeClient.update()) {
      Serial.println("✅ NTP Time Synced Successfully.");
      return true;  // 成功
    }

    // 強制的に更新を試みる（通常の更新が失敗した場合）
    timeClient.forceUpdate();

    // 1秒待機
    delay(1000);

    // 進行状況を画面とシリアルに表示
    M5.Display.print(".");
    Serial.print(".");
  }

  // 全ての試行が失敗
  Serial.println("\\n❌ NTP Time Sync Failed.");
  return false;
}

/**
 * @brief NTP同期結果を画面に表示
 * @param wasSuccessful 同期が成功したかどうか（true/false）
 */
void displayNTPSynchronizationResult(bool wasSuccessful) {
  // 画面をクリア
  clearDisplayScreenWithColor(BLACK);

  // テキスト位置を設定
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);

  if (wasSuccessful) {
    // 同期成功時の表示
    M5.Display.println("NTP Synced!");
    M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y + 20);
    M5.Display.println(timeClient.getFormattedTime());  // 同期された時刻を表示

    Serial.print("   Synced Time: ");
    Serial.println(timeClient.getFormattedTime());
  } else {
    // 同期失敗時の表示
    M5.Display.println("NTP Failed!");
  }

  // メッセージを一定時間表示
  delay(CONNECTION_SUCCESS_DISPLAY_TIME);
}

/**
 * @brief MQTT接続を設定する
 * @details MQTTサーバーのアドレスとコールバック関数を設定します
 */
void configureMQTTConnection() {
  // MQTTブローカー（サーバー）のアドレスとポートを設定
  // MQTT_BROKER_ADDRESSとMQTT_BROKER_PORTはconfig.hで定義された定数
  mqttCommunicationClient.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);

  // メッセージ受信時のコールバック関数を設定
  // この関数は、MQTTメッセージを受信した時に自動的に呼び出される
  mqttCommunicationClient.setCallback(handleIncomingMQTTMessage);

  Serial.println("⚙️ MQTT Connection Configured.");
}

/**
 * @brief MQTTブローカーへの接続を確立
 * @details 一意のクライアントIDを生成し、MQTTサーバーに接続します
 */
void establishMQTTBrokerConnection() {
  Serial.println("📡 Attempting to connect to MQTT broker...");

  // MQTT接続中メッセージを表示
  showConnectionStatusMessage("MQTT connecting...");

  // 接続が確立されるまでループ
  while (!mqttCommunicationClient.connected()) {
    // 一意のクライアントIDを生成（同じIDで複数の接続を避けるため）
    String uniqueClientId = generateUniqueMQTTClientId();

    // MQTTブローカーへの接続を試みる
    if (attemptMQTTBrokerConnection(uniqueClientId)) {
      // 接続成功時：トピックをサブスクライブし、成功メッセージを表示
      subscribeToMQTTDataTopic();
      displayMQTTConnectionSuccess();
      break;  // ループを抜ける
    } else {
      // 接続失敗時：エラーを表示して再試行
      displayMQTTConnectionFailure();
    }
  }
}

/**
 * @brief 一意のMQTTクライアントIDを生成
 * @return 生成されたクライアントID文字列
 * @details 固定のプレフィックスとランダムな16進数を組み合わせて一意のIDを作成します
 */
String generateUniqueMQTTClientId() {
  // MQTT_CLIENT_ID_PREFIXはconfig.hで定義されたプレフィックス（例："M5Stick-"）
  // random(0xffff)で0〜65535のランダムな数値を生成し、16進数表記（HEX）に変換
  // 例："M5Stick-4a3f"のような一意のIDになる
  return String(MQTT_CLIENT_ID_PREFIX) + String(random(0xffff), HEX);
}

/**
 * @brief MQTTブローカーへの接続を試みる
 * @param clientIdentifier 接続に使用する一意のクライアントID
 * @return 接続成功ならtrue、失敗ならfalse
 */
bool attemptMQTTBrokerConnection(const String &clientIdentifier) {
  // MQTTブローカーへ接続
  // connect()メソッドは接続成功時にtrue、失敗時にfalseを返す
  bool connectionEstablished = mqttCommunicationClient.connect(clientIdentifier.c_str());

  if (connectionEstablished) {
    // 接続成功のログ
    Serial.println("✅ MQTT Connection Successful.");
    Serial.print("   Client ID: ");
    Serial.println(clientIdentifier);
  } else {
    // 接続失敗のログ（エラーコード付き）
    Serial.print("❌ MQTT Connection Failed, rc=");
    Serial.println(mqttCommunicationClient.state());
    // エラーコードの意味:
    // -4: MQTT_CONNECTION_TIMEOUT - サーバー接続がタイムアウト
    // -3: MQTT_CONNECTION_LOST - ネットワーク接続が切断された
    // -2: MQTT_CONNECT_FAILED - ネットワーク接続に失敗
    // -1: MQTT_DISCONNECTED - クライアントが切断された
    // 0: MQTT_CONNECTED - クライアントは接続されている
    // 1: MQTT_CONNECT_BAD_PROTOCOL - サーバーがプロトコルをサポートしていない
    // 2: MQTT_CONNECT_BAD_CLIENT_ID - クライアントIDが拒否された
    // 3: MQTT_CONNECT_UNAVAILABLE - サーバーが利用できない
    // 4: MQTT_CONNECT_BAD_CREDENTIALS - ユーザー名/パスワードが拒否された
    // 5: MQTT_CONNECT_UNAUTHORIZED - 認証されていない
  }

  return connectionEstablished;
}

/**
 * @brief MQTTデータトピックをサブスクライブ（購読）する
 * @details 指定されたトピックからメッセージを受信できるようにします
 */
void subscribeToMQTTDataTopic() {
  // 指定されたトピック名にサブスクライブ
  // MQTT_TOPIC_NAMEはconfig.hで定義されたトピック名（例："home/sensors/climate"）
  mqttCommunicationClient.subscribe(MQTT_TOPIC_NAME);

  // サブスクライブ成功のログ
  Serial.print("📬 Subscribed to MQTT topic: ");
  Serial.println(MQTT_TOPIC_NAME);
}

/**
 * @brief MQTT接続成功時のメッセージを表示
 */
void displayMQTTConnectionSuccess() {
  // 接続成功メッセージを画面に表示
  M5.Display.println("MQTT Connected!");

  // メッセージを1秒間表示
  delay(1000);
}

/**
 * @brief MQTT接続失敗時のメッセージとエラーコードを表示
 */
void displayMQTTConnectionFailure() {
  // 接続失敗メッセージとエラーコードを画面に表示
  M5.Display.print("Failed, rc=");
  M5.Display.print(mqttCommunicationClient.state());  // エラーコードを表示
  M5.Display.println(" retry in 5s");

  // 次の再試行までの待機時間
  // MQTT_RECONNECTION_DELAY_MILLISECONDSはconfig.hで定義された待機時間（通常5000ms=5秒）
  delay(MQTT_RECONNECTION_DELAY_MILLISECONDS);
}

// -----------------------------------------------------------------
// MQTT データ処理関連の関数
// -----------------------------------------------------------------

/**
 * @brief 受信したMQTTメッセージを処理する（コールバック関数）
 * @param topicName メッセージを受信したトピック名
 * @param messagePayload メッセージの内容（バイト配列）
 * @param messageLength メッセージのバイト長
 * @details この関数は、MQTTメッセージを受信した時に自動的に呼び出されます
 */
void handleIncomingMQTTMessage(char *topicName, byte *messagePayload, unsigned int messageLength) {
  // 受信したバイト配列を文字列に変換
  String jsonMessageString = convertRawPayloadToString(messagePayload, messageLength);

  // 受信ログをシリアルに出力
  Serial.println("\\n--- New MQTT Message Received ---");
  Serial.printf("Topic: %s\\n", topicName);                      // トピック名
  Serial.printf("Payload: '%s'\\n", jsonMessageString.c_str());  // メッセージ内容

  // JSONデータの整合性をチェック（有効なJSONかどうか）
  if (!validateJSONDataIntegrity(jsonMessageString)) {
    Serial.println("❌ Invalid JSON data detected.");
    displayJSONParsingError("Invalid JSON");
    return;  // 不正なJSONなら処理を中断
  }

  // JSONデータをパースしてセンサーデータ構造体に変換
  SensorDataPacket parsedSensorData = parseJSONSensorData(jsonMessageString);

  if (parsedSensorData.hasValidData) {
    // パースが成功した場合：センサーデータを更新して画面を更新
    updateCurrentSensorData(parsedSensorData);
    Serial.printf("✅ Sensor data updated: CO2=%d, THI=%.1f\\n",
                  parsedSensorData.carbonDioxideLevel, parsedSensorData.thermalComfortIndex);
    refreshEntireDisplay();
  } else {
    // パースが失敗した場合：エラーメッセージを表示
    Serial.println("❌ Sensor data parsing failed.");
    displayJSONParsingError("Parse Failed");
  }

  Serial.println("---------------------------------");
}

/**
 * @brief JSONデータの整合性を検証する
 * @param jsonData 検証するJSON文字列
 * @return 有効なJSONならtrue、そうでなければfalse
 * @details 基本的なJSON形式の正当性チェックを行います
 */
bool validateJSONDataIntegrity(const String &jsonData) {
  // 文字列の前後の空白を削除
  String trimmedData = jsonData;
  trimmedData.trim();

  // 空のJSONは無効
  if (trimmedData.length() == 0)
    return false;

  // 正しいJSONは「{」で始まる必要がある
  if (!trimmedData.startsWith("{"))
    return false;

  // 正しいJSONは「}」で終わる必要がある
  if (!trimmedData.endsWith("}"))
    return false;

  // 基本的なチェックに合格
  return true;
}

/**
 * @brief 受信したバイト配列を文字列に変換
 * @param rawPayload バイトデータ配列
 * @param payloadLength データ長
 * @return 変換された文字列
 * @details バイナリデータから印字可能なASCII文字のみを抽出
 */
String convertRawPayloadToString(byte *rawPayload, unsigned int payloadLength) {
  String convertedMessage;

  // 必要なメモリをあらかじめ確保（最適化）
  convertedMessage.reserve(payloadLength + 1);

  // バイト配列を1バイトずつ処理
  for (unsigned int i = 0; i < payloadLength; i++) {
    // 印字可能なASCII文字（32-126）のみを文字列に追加
    // これにより、制御文字やバイナリデータが含まれていても適切に処理できる
    if (rawPayload[i] >= 32 && rawPayload[i] <= 126) {
      convertedMessage += (char)rawPayload[i];
    }
  }

  return convertedMessage;
}

/**
 * @brief JSON文字列をパースしてセンサーデータ構造体に変換
 * @param jsonString パース対象のJSON文字列
 * @return センサーデータ構造体（パースエラー時はhasValidData=false）
 * @details ArduinoJsonライブラリを使用してJSONをパースします
 */
SensorDataPacket parseJSONSensorData(const String &jsonString) {
  // 初期値がすべてゼロの構造体を作成
  SensorDataPacket extractedData = { 0, 0.0, 0.0, 0.0, "", 0, false };

  // JSONパース用のドキュメントオブジェクトを作成
  // JSON_PARSING_MEMORY_SIZEはconfig.hで定義されたJSONパース用メモリサイズ
  DynamicJsonDocument jsonDocument(JSON_PARSING_MEMORY_SIZE);

  // JSON文字列をパース
  DeserializationError parseError = deserializeJson(jsonDocument, jsonString);

  // パースエラーがあれば処理中断
  if (parseError) {
    Serial.printf("❌ JSON parsing failed: %s\\n", parseError.c_str());
    return extractedData;  // 無効なデータを返す
  }

  // 各フィールドが存在すれば、構造体にデータを設定
  // キーが存在するかチェックすることで、一部のデータが欠けていても対応可能
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

  // データが有効であることをフラグで示す
  extractedData.hasValidData = true;

  return extractedData;
}

/**
 * @brief 現在のセンサーデータを新しいデータで更新
 * @param newSensorData 新しいセンサーデータ
 */
void updateCurrentSensorData(const SensorDataPacket &newSensorData) {
  // グローバル変数のセンサーデータを、新しく受信したデータで上書き
  currentSensorReading = newSensorData;
}

/**
 * @brief MQTTブローカー接続を監視し、切断時には自動的に再接続する
 */
void maintainMQTTBrokerConnection() {
  // MQTT接続が切れているかチェック
  if (!mqttCommunicationClient.connected()) {
    // 切断されていれば再接続を試みる
    Serial.println("⚠️ MQTT connection lost. Reconnecting...");
    establishMQTTBrokerConnection();
  }
}

/**
 * @brief 受信したMQTTメッセージを処理する
 * @details MQTTクライアントのloop()メソッドを呼び出し、メッセージ受信を処理します
 */
void processIncomingMQTTMessages() {
  // MQTTクライアントのループ処理を実行
  // このメソッドを定期的に呼び出すことで、新しいメッセージがないかチェックし、
  // あればhandleIncomingMQTTMessageコールバック関数を自動的に呼び出します
  mqttCommunicationClient.loop();
}

/**
 * @brief NTPサーバーから時刻情報を更新
 */
void updateSystemNetworkTime() {
  // NTPクライアントの更新処理を実行
  // このメソッドは内部的に設定された間隔に基づいて更新処理を行います
  // （毎回サーバーにアクセスするわけではない）
  timeClient.update();
}

// -----------------------------------------------------------------
// ユーティリティ関数
// -----------------------------------------------------------------

/**
 * @brief 接続状態メッセージを画面に表示
 * @param statusMessage 表示するメッセージ
 */
void showConnectionStatusMessage(const char *statusMessage) {
  // 画面をクリア
  clearDisplayScreenWithColor(BLACK);

  // テキスト位置を設定
  M5.Display.setCursor(TITLE_POSITION_X, TITLE_POSITION_Y);

  // 指定されたメッセージを表示
  M5.Display.println(statusMessage);
}

/**
 * @brief 画面を指定した色でクリアする
 * @param backgroundColor 背景色（16ビット色）
 */
void clearDisplayScreenWithColor(uint16_t backgroundColor) {
  // 指定した色で画面全体を塗りつぶす
  // fillScreen()メソッドは画面全体を単一の色で塗りつぶします
  M5.Display.fillScreen(backgroundColor);
}

/**
 * @brief MQTTサブスクリプションのデバッグ情報をシリアルに出力
 * @details 接続トラブル時のデバッグに役立つ情報を表示します
 */
void printMQTTSubscriptionDebugInfo() {
  Serial.println("--- MQTT Subscription Status ---");
  Serial.printf("Broker: %s:%d\\n", MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  Serial.printf("Topic: %s\\n", MQTT_TOPIC_NAME);
  Serial.printf("Connected: %s\\n", mqttCommunicationClient.connected() ? "Yes" : "No");
  Serial.printf("Client State Code: %d\\n", mqttCommunicationClient.state());
  Serial.println("------------------------------");
}

/**
 * @brief センサーデータまたはエラーメッセージを表示（レガシーメソッド）
 * @details 新しいロジックで置き換えられているため、現在は使用されていません
 */
void displaySensorDataOrErrorMessage() { /* Redundant with new logic, can be removed if desired */
}
