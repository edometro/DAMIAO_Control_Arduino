/**
 * @file 1_SingleMotor_MIT_Control.ino
 * @brief DAMIAOモーターを1つ、MITモードで動作させる基本的なサンプルコードです。
 * @details
 * MITモードは、目標位置、目標速度、Pゲイン(Kp)、Dゲイン(Kd)、フィードフォワードトルクを
 * 組み合わせてモーターを制御するモードです。
 * 出力トルク = Kp * (目標位置 - 現在位置) + Kd * (目標速度 - 現在速度) + フィードフォワードトルク
 *
 * このサンプルでは、以下の手順でモーターを制御します。
 * 1. CAN通信の初期化
 * 2. モーターの各種パラメータを取得
 * 3. モーターの制御モードを「MITモード」に設定
 * 4. 現在位置をゼロ点として設定
 * 5. モーターを有効化（起動）
 * 6. MITモードの指令を送信（目標位置0速度0でバネダンパの挙動）
 * 7. 100msごとにモーターの状態（位置、速度、トルクなど）をシリアルモニタに表示
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

// モーターのフィードバック情報をシリアルモニタに表示する関数
void printFeedback(Motor& motor) {
  Status status = motor.getStatus();
  Mode mode = motor.getMode();
  float pos = motor.getPosition();
  float vel = motor.getVelocity();
  float tau = motor.getTorque();

  Serial.print("[FB] ID:");
  Serial.print(motor.getSlaveId());
  Serial.print(" Status:");
  Serial.print(statusToString(status));  // ステータスを文字列に変換して表示
  Serial.print(" Mode:");
  Serial.print(modeToString(mode));  // モードを文字列に変換して表示
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
  motor1.setCAN(&CAN);  // モーターにCANインスタンスを渡す
  motor1.disable();     // 念のためモーターを無効化

  // 2. モーターの各種パラメータを取得
  motor1.initialize();

  // 3. モーターの制御モードを設定
  Serial.print("Setting control mode to MIT...");
  if (motor1.setControlMode(Mode::MIT)) {
    Serial.println("Success.");
  } else {
    Serial.println("Failed.");
  }

  // 4. 現在位置をゼロ点として設定
  Serial.print("Setting current position as zero...");
  if (motor1.setZeroPosition()) {
    Serial.println("Success.");
  } else {
    Serial.println("Failed.");
  }

  // 5. モーターを有効化（起動）
  Serial.print("Enabling motor...");
  if (motor1.enable()) {
    Serial.println("Success.");
  } else {
    Serial.println("Failed.");
  }

  Serial.println("\nSetup complete. Motor is running.");
}

void loop() {
  // CANバス上のメッセージを処理し、モーターの内部状態（位置、速度など）を更新
  motor1.update();

  // 6. MITモードで指令を送信（目標位置0速度0でバネダンパの挙動）
  motor1.sendMIT(0.0f, 0.0f, 1.0f, 1.0f, 0.0f);

  // 7. 定期的にモーターの状態をシリアルモニタに表示
  const unsigned long FEEDBACK_INTERVAL_MS = 100UL;  // 100msごとに表示
  static unsigned long lastFeedbackMillis = 0;
  unsigned long now = millis();
  if (now - lastFeedbackMillis >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackMillis = now;
    printFeedback(motor1);
  }
  delay(1);
}
