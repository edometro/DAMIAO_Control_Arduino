/**
 * @file 3_Advanced_ModeSwitching.ino
 * @brief モーターの制御モードを動的に切り替えるサンプルコードです。
 * @details
 * このサンプルでは、1つのモーターを使い、10秒ごとに以下の3つのモードを順番に切り替えます。
 * これにより、各制御モードがどのような動作をするかを実際に確認できます。
 *
 * 1. POS_VEL (位置・速度制御モード):
 *    - 指定した位置(180°)まで、指定した速度で移動します。
 *
 * 2. VEL (速度制御モード):
 *    - 指定した速度(0.5 rad/s)で回転し続けます。
 *
 * 3. MIT (MITモード / インピーダンス制御モード):
 *    - モーターが外力に対して柔軟に反応するようになります。
 *    - このモードでは、モーターを手で簡単に回すことができます。
 *    - 位置(pos), 速度(vel), KP, KD, トルク(tau)をパラメータとして指定します。
 *      ここでは、目標位置0, 速度0, KP=5.0, KD=2.0, トルク0として、バネダンパの挙動をさせています。
 *
 * @note
 * モードを切り替える際には、一度モーターを disable し、再度 enable するのが安全です。
 * このサンプルでは、モード切替時に自動でゼロ点をリセットしています。
 */

// ▼▼▼ Select Board ▼▼▼ 使うボードだけコメントアウトを外す
#define USE_BOARD_ARDUINO_R4
// #define USE_BOARD_PICO
// #define USE_BOARD_ESP32

#if defined(USE_BOARD_ARDUINO_R4)
#include <Arduino_CAN.h>
// R4はCAN_TX/CAN_RXピンが固定のため、ピン設定は不要

#elif defined(USE_BOARD_ESP32)
#include <ESP32_TWAI.h>  // https://github.com/eyr1n/ESP32_TWAI
// 使用するマイコンに合わせてピン番号を変更してください
const gpio_num_t CAN_TX_PIN = 22;
const gpio_num_t CAN_RX_PIN = 21;

#elif defined(USE_BOARD_PICO)
#include <RP2040PIO_CAN.h>  //https://github.com/eyr1n/RP2040PIO_CAN
// 使用するマイコンに合わせてピン番号を変更してください
const uint32_t CAN_TX_PIN = 0;  // 連続してなくてもいい
const uint32_t CAN_RX_PIN = 1;  // GP1とGP3みたいな組み合わせでも動く

#else
#error "ボードが選択されていません。ファイルの先頭で USE_BOARD_... のどれか1つを有効にしてください。"
#endif

//CANライブラリよりも下で呼び出す api/HardwareCAN.hが無いって言われる
#include <DAMIAO_Control.h>  // DAMIAOモーター制御ライブラリ
#include <DMUtils.h>         // ユーティリティ関数置き場

using namespace damiao;

// =============================================
// ユーザー設定項目 (User Settings)
// =============================================
const uint32_t MASTER_ID = 0x00;  // モーターのMasterID
const uint32_t SLAVE_ID = 0x09;   // モーターのSlaveID
// =============================================

// モーターオブジェクトの作成
Motor motor1(MASTER_ID, SLAVE_ID);

// 現在の制御モードを管理するための変数
static Mode currentState = Mode::POS_VEL;
// 最後にモードを切り替えた時刻
static unsigned long lastSwitchMillis = 0;
// モードを切り替える間隔 (ミリ秒)
const unsigned long SWITCH_INTERVAL_MS = 10000UL;

void printFeedback(Motor& motor) {
  Status status = motor.getStatus();
  Mode mode = motor.getMode();
  float pos = motor.getPosition();
  float vel = motor.getVelocity();
  float tau = motor.getTorque();

  Serial.print("[FB] ID:");
  Serial.print(motor.getSlaveId());
  Serial.print(" Status:");
  Serial.print(statusToString(status));
  Serial.print(" Mode:");
  Serial.print(modeToString(mode));
  Serial.print(" Pos:");
  Serial.print(pos, 2);
  Serial.print(" Vel:");
  Serial.print(vel, 2);
  Serial.print(" Tau:");
  Serial.println(tau, 2);
}

// 次の制御モードに切り替える関数
void switchToNextState(Motor& motor) {
  Serial.println("\n--------------------");
  Serial.print("Switching mode from ");
  Serial.print(modeToString(motor.getMode()));
  Serial.print(" to ");

  motor.disable();  // 安全のため、モード切替前に一度モーターを無効化
  delay(100);

  // モードを順番に切り替える
  switch (currentState) {
    case Mode::POS_VEL:
      currentState = Mode::VEL;
      Serial.println("Executing VEL command: Rotate at 0.5 rad/s.");
      break;
    case Mode::VEL:
      currentState = Mode::MIT;
      break;
    case Mode::MIT:
      currentState = Mode::POS_VEL;
      Serial.println("Executing POS_VEL command: Go to 180 deg.");
      break;
    default:  // その他のモードの場合はPOS_VELに戻す
      currentState = Mode::POS_VEL;
      Serial.println("Executing POS_VEL command: Go to 180 deg.");
      break;
  }
  Serial.println(modeToString(currentState));
  Serial.println("--------------------");
  motor.setControlMode(currentState);  // 新しいモードを設定
  motor.setZeroPosition();             // ゼロ点をリセット
  motor.enable();                      // モーターを再度有効化
}


void setup() {
  Serial.begin(115200);
  // シリアルモニタが起動するまで最大5秒待機
  while (!Serial && millis() < 5000);
  Serial.println("\n--- Single Motor MIT Control Example ---");

  // 1. CAN通信の初期化
  bool can_ok = false;
#if defined(USE_BOARD_ARDUINO_R4)
  can_ok = CAN.begin(CanBitRate::BR_1000k);

#elif defined(USE_BOARD_ESP32)
  can_ok = CAN.begin(CanBitRate::BR_1000k, CAN_TX_PIN, CAN_RX_PIN);

#elif defined(USE_BOARD_PICO)
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#endif

  if (can_ok) {
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed!");
    while (1);  // CAN通信が開始できない場合はここで停止
  }
  motor1.setCAN(&CAN);  // モーターにCANインスタンスを渡す
  motor1.disable();     // 念のためモーターを無効化

  // 2. モーターの各種パラメータを取得
  motor1.initialize();

  // 最初のモードを設定して有効化
  Serial.print("Starting with mode: ");
  Serial.println(modeToString(currentState));
  motor1.setControlMode(currentState);
  motor1.setZeroPosition();
  motor1.enable();
  lastSwitchMillis = millis();  // 最初の切り替え時刻を記録
}

void loop() {
  motor1.update();

  // 一定時間ごとにモードを切り替える
  unsigned long now = millis();
  if (now - lastSwitchMillis >= SWITCH_INTERVAL_MS) {
    lastSwitchMillis = now;
    switchToNextState(motor1);
  }

  // 現在のモードに応じた制御コマンドを送信
  switch (currentState) {
    case Mode::POS_VEL:
      // 【位置・速度制御】目標位置 3.14 rad (180°), 速度制限 1.0 rad/s
      motor1.sendPosition(M_PI, 1.0f);
      break;

    case Mode::VEL:
      // 【速度制御】目標速度 0.5 rad/s
      motor1.sendVelocity(0.5f);
      break;

    case Mode::MIT:
      // 【インピーダンス制御】目標位置 0, 速度 0, KP 1.0, KD 1.0, トルク 0
      // この設定では、モーターはバネダンパの挙動を示します。
      motor1.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
      break;

    default:
      // 何もしない
      break;
  }

  // 定期的にフィードバックを表示
  static unsigned long lastFeedbackMillis = 0;
  const unsigned long FEEDBACK_INTERVAL_MS = 100UL;
  if (now - lastFeedbackMillis >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackMillis = now;
    printFeedback(motor1);
  }
  delay(1);
}
