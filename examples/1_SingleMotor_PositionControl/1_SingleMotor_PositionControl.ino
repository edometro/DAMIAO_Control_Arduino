/**
 * @file 1_SingleMotor_PositionControl.ino
 * @brief DAMIAOモーターを1つ、位置制御モードで動作させる基本的なサンプルコードです。
 * @details
 * このサンプルでは、以下の手順でモーターを制御します。
 * 1. CAN通信の初期化
 * 2. モーターの位置・速度・トルクの計算用パラメータを取得
 * 3. モーターの制御モードを「位置・速度制御モード」に設定
 * 4. 現在位置をゼロ点として設定
 * 5. モーターを有効化（起動）
 * 6. 指定した目標位置（ラジアン単位）までモーターを回転させる
 * 7. 500msごとにモーターの状態（位置、速度、トルクなど）をシリアルモニタに表示
 */
#include <RP2040PIO_CAN.h>  // RP2040用のCAN通信ライブラリ

#include <DAMIAO.h>   // DAMIAOモーター制御ライブラリ
#include <DMUtils.h>  // 便利なユーティリティ関数

using namespace damiao;

// =============================================
// ユーザー設定項目 (User Settings)
// =============================================

// --- CAN通信設定 ---
// 使用するマイコンに合わせてピン番号を変更してください
const uint8_t CAN_TX_PIN = 0;  // CANトランシーバーのTXピン
const uint8_t CAN_RX_PIN = 1;  // CANトランシーバーのRXピン

// --- モーター設定 ---
const uint32_t MASTER_ID = 0x00;  // モーターのMasterID
const uint32_t SLAVE_ID = 0x09;   // モーターのSlaveID

// --- 制御目標値 ---
const float TARGET_POSITION_RAD = M_PI;  // 目標位置 (ラジアン単位, M_PI = 180°)
const float VELOCITY_LIMIT_RPS = 1.0f;   // 速度制限 (rad/s)

// =============================================

// モーターオブジェクトの作成
Motor motor1(MASTER_ID, SLAVE_ID);

// 最後にフィードバックを表示した時刻
static unsigned long lastFeedbackMillis = 0;
const unsigned long FEEDBACK_INTERVAL_MS = 500UL;  // 500msごとに表示

// モーターのフィードバック情報をシリアルモニタに表示する関数
void printFeedback() {
  // motor1.update() を loop内で呼び出すことで、これらの値は最新に保たれます
  Status status = motor1.getStatus();
  Mode mode = motor1.getMode();
  float pos = motor1.getPosition();
  float vel = motor1.getVelocity();
  float tau = motor1.getTorque();

  Serial.print("[FB] ID:");
  Serial.print(motor1.getSlaveId());
  Serial.print(" Status:");
  Serial.print(statusToString(status));  // ステータスを文字列に変換して表示
  Serial.print(" Mode:");
  Serial.print(modeToString(mode));  // モードを文字列に変換して表示
  Serial.print(" Pos:");
  Serial.print(pos, 2);  // 小数点以下2桁まで表示
  Serial.print(" Vel:");
  Serial.print(vel, 2);
  Serial.print(" Tau:");
  Serial.println(tau, 2);
}

void setup() {
  Serial.begin(115200);
  // シリアルモニタが起動するまで最大5秒待機
  while (!Serial && millis() < 5000);
  Serial.println("\n--- Single Motor Position Control Example ---");

  // 1. CAN通信の初期化
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  if (CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed!");
    while (1);  // CAN通信が開始できない場合はここで停止
  }
  motor1.setCAN(&CAN);  // モーターにCANインスタンスを渡す
  delay(100);

  // 2. モーターの位置・速度・トルクの計算用パラメータを取得
  motor1.initialize();

  // 3. モーターの制御モードを設定
  Serial.print("Setting control mode to POS_VEL...");
  if (motor1.setControlMode(Mode::POS_VEL)) {
    Serial.println("Success.");
  } else {
    Serial.println("Failed.");
  }
  delay(100);

  // 4. 現在位置をゼロ点として設定
  Serial.print("Setting current position as zero...");
  if (motor1.setZeroPosition()) {
    Serial.println("Success.");
  } else {
    Serial.println("Failed.");
  }
  delay(100);

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
  // CANバス上のメッセージを処理し、モーターの内部状態（位置、速度など）を更新します。
  // この関数はできるだけ頻繁に呼び出してください。
  motor1.update();

  // 6. 指定した目標位置にモーターを回転させる
  motor1.sendPosition(TARGET_POSITION_RAD, VELOCITY_LIMIT_RPS);

  // 7. 定期的にモーターの状態をシリアルモニタに表示
  unsigned long now = millis();
  if (now - lastFeedbackMillis >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackMillis = now;
    printFeedback();
  }

  delay(1);  // CPU負荷を軽減するための短い待機
}
