#ifndef CONFIG_H
#define CONFIG_H

// ================================================================================
// M5StickCPlus2 センサーモニター 設定ファイル
// ================================================================================

// ========== ★要編集★ ネットワーク設定 ==========
const char* WIFI_NETWORK_NAME = "Your_WiFi_SSID";
const char* WIFI_NETWORK_PASSWORD = "Your_WiFi_Password";

// ========== ★要編集★ MQTT設定 ==========
const char* MQTT_BROKER_ADDRESS = "192.168.1.100";
const char* MQTT_TOPIC_NAME = "sensor_data";
const int   MQTT_BROKER_PORT = 1883;
const char* MQTT_CLIENT_ID_PREFIX = "M5StickCPlus2-";

// ========== 時刻同期設定 ==========
const char* TIME_SERVER_ADDRESS = "pool.ntp.org";
const long  JAPAN_TIME_OFFSET_SECONDS = 32400;
const unsigned long TIME_UPDATE_INTERVAL_MILLISECONDS = 60000;

// ========== 表示更新設定 ==========
const unsigned long MAIN_LOOP_DELAY_MILLISECONDS = 100;

// ========== 画面表示位置の設定 ==========
const int VERTICAL_OFFSET        = 5;
const int TITLE_POSITION_X       = 5;
const int TITLE_POSITION_Y       = 2 + VERTICAL_OFFSET;
const int TIME_DISPLAY_X         = 140;
const int TIME_DISPLAY_Y         = 2 + VERTICAL_OFFSET;
const int CONNECTION_STATUS_X    = 190;
const int CONNECTION_STATUS_Y    = 2 + VERTICAL_OFFSET;
const int LARGE_LABEL_X          = 15;
const int LARGE_LABEL_Y          = 30 + VERTICAL_OFFSET;
const int LARGE_VALUE_Y          = 50 + VERTICAL_OFFSET;
const int DISPLAY_RIGHT_MARGIN   = 15;
const int NO_DATA_MESSAGE_X      = 40;
const int NO_DATA_MESSAGE_Y      = 55 + VERTICAL_OFFSET;

// ========== 再試行・タイムアウト設定 ==========
const int   MAXIMUM_NTP_RETRY_ATTEMPTS = 10;
const unsigned long MQTT_RECONNECTION_DELAY_MILLISECONDS = 5000;
const unsigned long CONNECTION_SUCCESS_DISPLAY_TIME = 2000;

// ========== JSON解析設定 ==========
const size_t JSON_PARSING_MEMORY_SIZE = 2048;

// ========== 交互表示のための設定 ==========
const unsigned long INTERACTIVE_DISPLAY_INTERVAL_MILLISECONDS = 3000;

// ========== NEW: 重力ベースの自動回転設定 ==========
// 有効/無効
const bool ENABLE_GRAVITY_AUTO_ROTATE = true;

// 判定周期（ミリ秒）
const unsigned long ORIENTATION_CHECK_INTERVAL_MS = 200;

// XY平面方向の重力（g）成分がこの閾値未満なら判定しない（机に水平時の誤判定防止）
const float ORIENTATION_TILT_THRESHOLD_G = 0.20f;

// ヒステリシス余裕（g）
const float ORIENTATION_HYSTERESIS_G = 0.05f;

// 既定の回転（メインボタンが右の時の回転）と反転時
const int DISPLAY_ROTATION_NORMAL  = 1;  // 通常（既存スケッチと同じ）
const int DISPLAY_ROTATION_FLIPPED = 3;  // 180度回転

// X軸の符号判定を反転したい場合は true に（環境で逆に判定される場合の切替）
const bool ORIENTATION_INVERT_X = false;

#endif  // CONFIG_H
