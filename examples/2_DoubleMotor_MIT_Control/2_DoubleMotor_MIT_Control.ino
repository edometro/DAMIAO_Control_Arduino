/**
 * @file 2_DoubleMotor_MIT_Control.ino
 * @brief DAMIAOモーターを2つ、MITモードで動作させるサンプルコードです。
 * @details
 * このサンプルでは、MotorManagerクラスを使って複数のモーターを管理する方法を示します。
 * 2つのモーターが、MIT制御されます。
 * 1. CAN通信の初期化
 * 2. 2つのモーターを管理するMotorManagerを作成
 * 3. モーターの各種パラメータを取得
 * 4. 両方のモーターを「MITモード」に設定
 * 5. 両方のモーターの現在位置をゼロ点として設定
 * 6. 両方のモーターを有効化（起動）
 * 7. MITモードの指令を送信（目標位置0速度0でバネダンパの挙動）
 * 8. 100msごとにモーターの状態（位置、速度、トルクなど）をシリアルモニタに表示
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
const uint32_t MASTER_ID = 0x00;         // モーターのMasterID
const uint32_t MOTOR_1_SLAVE_ID = 0x09;  // 1台目のモーターのSlaveID
const uint32_t MOTOR_2_SLAVE_ID = 0x0a;  // 2台目のモーターのSlaveID
// =============================================

// モーターオブジェクトの作成
Motor motor1(MASTER_ID, MOTOR_1_SLAVE_ID);
Motor motor2(MASTER_ID, MOTOR_2_SLAVE_ID);

// 複数のモーターを管理するMotorManagerを作成
// <>内の数字は管理するモーターの数
MotorManager<2> motorManager(&CAN, &motor1, &motor2);  // カンマ区切りで複数のモーターを渡す

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

  // 2. MotorManagerを使ってモーターを一括でセットアップ
  Serial.println("Setting up motors via MotorManager...");
  motorManager.disableAll();  // 念のため全モーターを無効化

  // 3. 全モーターの各種パラメータを取得
  motorManager.initializeAll();

  // 4. 全モーターの制御モードを設定
  // forループでも可能
  for (size_t i = 0; i < motorManager.size(); ++i) {
    motorManager.getMotor(i).setControlMode(Mode::MIT);
  }

  // 5. 全モーターの現在位置をゼロ点に設定
  // Motorオブジェクトを直接使っても可能
  motor1.setZeroPosition();
  motor2.setZeroPosition();

  // 6. 全モーターを有効化
  motorManager.enableAll();

  Serial.println("\nSetup complete. Motors are running.");
}

void loop() {
  // CANバス上のメッセージを処理し、モーターの内部状態（位置、速度など）を一括更新
  motorManager.update();

  // 7. MITモードで指令を送信（目標位置0速度0でバネダンパの挙動）
  motor1.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);
  motor2.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);

  // 8. 定期的にモーターの状態をシリアルモニタに表示
  const unsigned long FEEDBACK_INTERVAL_MS = 100UL;  // 100msごとに表示
  static unsigned long lastFeedbackMillis = 0;
  unsigned long now = millis();
  if (now - lastFeedbackMillis >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackMillis = now;
    printFeedback(motor1);
    printFeedback(motor2);
  }
  delay(1);
}
