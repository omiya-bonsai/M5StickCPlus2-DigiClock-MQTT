#ifndef CONFIG_H
#define CONFIG_H

// ================================================================================
// M5StickCPlus2 センサーモニター 設定ファイル (テンプレート)
// ================================================================================
//
// 【使い方】
// 1. このファイルをコピーして、ファイル名を "config.h" に変更します。
// 2. "config.h" ファイル内の、ご自身の環境に合わせた設定値（*****の部分）を編集します。
// ※ "config.h" ファイルは .gitignore によりGitの管理対象から除外されるため、
//    パスワードなどの個人情報がGitHubにアップロードされるのを防ぎます。
//
// ================================================================================


// ========== ★要編集★ ネットワーク設定 ==========
const char* WIFI_NETWORK_NAME = "Your_WiFi_SSID";          // ***** 接続するWiFiのSSID（ネットワーク名）を入力 *****
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";  // ***** WiFiのパスワードを入力 *****

// ========== ★要編集★ MQTT設定 ==========
const char* MQTT_BROKER_ADDRESS = "192.168.1.100";     // ***** MQTTブローカ（サーバ）のIPアドレスを入力 *****
const char* MQTT_TOPIC_NAME = "sensor_data";           // 購読するトピック名（必要に応じて変更）
const int MQTT_BROKER_PORT = 1883;                     // MQTTブローカのポート番号（標準は1883）
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";  // MQTT接続時のクライアントID接頭辞

// ========== 時刻同期設定 ==========
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";               // NTPサーバのアドレス
const long JAPAN_TIME_OFFSET_SECONDS = 32400;                   // 日本時間のオフセット（+9時間を秒換算）
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000;  // 時刻更新間隔（1分）

// ========== 表示更新設定 ==========
const unsigned long MAIN_LOOP_DELAY_MILLISECONDS = 100;  // メインループの待機時間

// ========== 画面表示位置の設定 ==========
const int VERTICAL_OFFSET = 5;
const int TITLE_POSITION_X = 5;
const int TITLE_POSITION_Y = 2 + VERTICAL_OFFSET;
const int TIME_DISPLAY_X = 140;
const int TIME_DISPLAY_Y = 2 + VERTICAL_OFFSET;
const int CONNECTION_STATUS_X = 190;
const int CONNECTION_STATUS_Y = 2 + VERTICAL_OFFSET;
const int LARGE_LABEL_X = 15;
const int LARGE_LABEL_Y = 30 + VERTICAL_OFFSET;
const int LARGE_VALUE_Y = 50 + VERTICAL_OFFSET;
const int DISPLAY_RIGHT_MARGIN = 15;
const int NO_DATA_MESSAGE_X = 40;
const int NO_DATA_MESSAGE_Y = 55 + VERTICAL_OFFSET;

// ========== 再試行・タイムアウト設定 ==========
const int MAXIMUM_NTP_RETRY_ATTEMPTS = 10;
const unsigned long MQTT_RECONNECTION_DELAY_MILLISECONDS = 5000;
const unsigned long CONNECTION_SUCCESS_DISPLAY_TIME = 2000;

// ========== JSON解析設定 ==========
const size_t JSON_PARSING_MEMORY_SIZE = 2048;

// ========== 交互表示のための設定 ==========
const unsigned long INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS = 3000;

#endif  // CONFIG_H
