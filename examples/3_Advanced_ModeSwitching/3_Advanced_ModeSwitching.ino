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
#include <RP2040PIO_CAN.h>

#include <DAMIAO.h>
#include <DMUtils.h>

using namespace damiao;

// =============================================
// ユーザー設定項目 (User Settings)
// =============================================
const uint8_t CAN_TX_PIN = 0;
const uint8_t CAN_RX_PIN = 1;

const uint32_t MASTER_ID = 0x00;
const uint32_t MOTOR_SLAVE_ID = 0x09;
// =============================================

// モーターオブジェクト
Motor motor1(MASTER_ID, MOTOR_SLAVE_ID);

// 現在の制御モードを管理するための変数
static Mode currentState = Mode::POS_VEL;

// 最後にモードを切り替えた時刻
static unsigned long lastSwitchMillis = 0;
// モードを切り替える間隔 (ミリ秒)
const unsigned long SWITCH_INTERVAL_MS = 10000UL;

// 最後にフィードバックを表示した時刻
static unsigned long lastFeedbackMillis = 0;
const unsigned long FEEDBACK_INTERVAL_MS = 500UL;


// モーターのフィードバック情報を表示する関数
void printFeedback() {
  Status status = motor1.getStatus();
  Mode mode = motor1.getMode();
  float pos = motor1.getPosition();
  float vel = motor1.getVelocity();
  float tau = motor1.getTorque();

  Serial.print("[FB] ID:");
  Serial.print(motor1.getSlaveId());
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
void switchToNextState() {
  Serial.println("\n--------------------");
  Serial.print("Switching mode from ");
  Serial.print(modeToString(motor1.getMode()));
  Serial.print(" to ");

  // 安全のため、モード切替前に一度モーターを無効化
  motor1.disable();
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

  // 新しいモードを設定
  motor1.setControlMode(currentState);

  // ゼロ点をリセット
  motor1.setZeroPosition();

  // モーターを再度有効化
  motor1.enable();
}


void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n--- Advanced Mode Switching Example ---");

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  if (!CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
  motor1.setCAN(&CAN);
  delay(100);

  // 最初のモードを設定して有効化
  Serial.print("Starting with mode: ");
  Serial.println(modeToString(currentState));
  motor1.setControlMode(currentState);
  delay(100);
  motor1.setZeroPosition();
  delay(100);
  motor1.enable();

  // 最初の切り替え時刻を記録
  lastSwitchMillis = millis();
}

void loop() {
  motor1.update();

  // 一定時間ごとにモードを切り替える
  unsigned long now = millis();
  if (now - lastSwitchMillis >= SWITCH_INTERVAL_MS) {
    lastSwitchMillis = now;
    switchToNextState();
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
      // 【インピーダンス制御】目標位置 0, 速度 0, KP 5.0, KD 2.0, トルク 0
      // この設定では、モーターはバネダンパの挙動を示します。
      motor1.sendMIT(0.0f, 0.0f, 5.0f, 2.0f, 0.0f);
      break;

    default:
      // 何もしない
      break;
  }

  // 定期的にフィードバックを表示
  if (now - lastFeedbackMillis >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackMillis = now;
    printFeedback();
  }

  delay(10);  // 少し長めのdelayでシリアル出力を見やすくする
}
