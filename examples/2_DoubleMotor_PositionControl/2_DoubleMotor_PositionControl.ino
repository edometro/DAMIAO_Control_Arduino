/**
 * @file 2_DoubleMotor_PositionControl.ino
 * @brief DAMIAOモーターを2つ、位置制御モードで動作させるサンプルコードです。
 * @details
 * このサンプルでは、MotorManagerクラスを使って複数のモーターを効率的に管理する方法を示します。
 * 2つのモーターが、それぞれ反対方向にサイン波を描くように動きます。
 * 1. CAN通信の初期化
 * 2. 2つのモーターを管理するMotorManagerを作成
 * 3. 両方のモーターを「位置・速度制御モード」に設定
 * 4. 両方のモーターの現在位置をゼロ点として設定
 * 5. 両方のモーターを有効化（起動）
 * 6. loop内で時間経過に合わせてサイン波の目標位置を計算し、各モーターに指令を送信
 * 7. 500msごとに両方のモーターの状態をシリアルモニタに表示
 */
#include <RP2040PIO_CAN.h>  // RP2040用のCAN通信ライブラリ

#include <DAMIAO.h>   // DAMIAOモーター制御ライブラリ
#include <DMUtils.h>  // 便利なユーティリティ関数

using namespace damiao;

// =============================================
// ユーザー設定項目 (User Settings)
// =============================================

// --- CAN通信設定 ---
const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;

// --- モーター設定 ---
const uint32_t MASTER_ID = 0x00;
const uint32_t MOTOR_1_SLAVE_ID = 0x09;  // 1台目のモーターのSlaveID
const uint32_t MOTOR_2_SLAVE_ID = 0x0a;  // 2台目のモーターのSlaveID

// --- 制御目標値 ---
const float AMPLITUDE_RAD = M_PI / 2.0f;  // 動作の振幅 (ラジアン単位, PI/2 = 90°)
const float FREQUENCY_HZ = 0.2f;          // 動作の周波数 (Hz)
const float VELOCITY_LIMIT_RPS = 2.0f;    // 速度制限 (rad/s)

// =============================================

// モーターオブジェクトの作成
Motor motor1(MASTER_ID, MOTOR_1_SLAVE_ID);
Motor motor2(MASTER_ID, MOTOR_2_SLAVE_ID);

// 2つのモーターを管理するMotorManagerを作成
// <>内の数字は管理するモーターの数
MotorManager<2> motorManager(&CAN, &motor1, &motor2);  // カンマ区切りで複数のモーターを渡す

// 最後にフィードバックを表示した時刻
static unsigned long lastFeedbackMillis = 0;
const unsigned long FEEDBACK_INTERVAL_MS = 500UL;

// モーターのフィードバック情報をシリアルモニタに表示する関数
void printFeedback(Motor& motor) {
  Status status = motor.getStatus();
  Mode mode = motor.getMode();
  float pos = motor.getPosition();
  float vel = motor.getVelocity();
  float tau = motor.getTorque();

  Serial.print("[FB] ID:");
  Serial.print(motor.getSlaveId());
  Serial.print(" Pos:");
  Serial.print(pos, 2);
  Serial.print(" Vel:");
  Serial.print(vel, 2);
  Serial.print(" Tau:");
  Serial.println(tau, 2);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n--- Double Motor Position Control Example ---");

  // 1. CAN通信の初期化
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  if (CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed!");
    while (1);  // CAN通信が開始できない場合はここで停止
  }
  delay(100);

  // 2. MotorManagerを使ってモーターを一括でセットアップ
  Serial.println("Setting up motors via MotorManager...");

  // 全モーターのパラメータを初期化
  motorManager.initializeAll();

  // 全モーターを位置・速度制御モードに設定
  // forループでも可能
  for (size_t i = 0; i < motorManager.size(); ++i) {
    motorManager.getMotor(i).setControlMode(Mode::POS_VEL);
  }

  // 全モーターの現在位置をゼロ点に設定
  // Motorオブジェクトを直接使っても可能
  motor1.setZeroPosition();
  motor2.setZeroPosition();

  // 全モーターを有効化
  motorManager.enableAll();

  Serial.println("\nSetup complete. Motors are running.");
}

void loop() {
  // 6. CANメッセージを処理し、全モーターの状態を更新
  motorManager.update();

  // 7. 時間に応じてサイン波の目標位置を計算
  unsigned long currentTime = millis();
  float angle = 2.0f * M_PI * FREQUENCY_HZ * (currentTime / 1000.0f);
  float targetPos1 = AMPLITUDE_RAD * sin(angle);
  float targetPos2 = -AMPLITUDE_RAD * sin(angle);  // 逆方向に動かす

  // 各モーターに目標位置を送信
  motor1.sendPosition(targetPos1, VELOCITY_LIMIT_RPS);
  motor2.sendPosition(targetPos2, VELOCITY_LIMIT_RPS);

  // 8. 定期的にモーターの状態を表示
  unsigned long now = millis();
  if (now - lastFeedbackMillis >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackMillis = now;
    Serial.println("--- Feedback ---");
    // MotorManagerを使って、管理下の全モーターの情報を表示
    for (size_t i = 0; i < motorManager.size(); ++i) {
      printFeedback(motorManager.getMotor(i));
    }
  }

  delay(1);
}
