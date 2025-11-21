/**
 * @file 4_Utility_ReadWriteParams.ino
 * @brief モーターの内部パラメータを読み書きするユーティリティ的なサンプルです。
 * @details
 * このライブラリでは、`motor.param`オブジェクトを通して、モーターの様々な設定値にアクセスできます。
 * 例えば、PIDゲイン、各種制限値（電流、電圧、速度）、IDなどをプログラムから動的に変更することが可能です。
 *
 * このサンプルでは、いくつかの代表的なパラメータの読み取り、書き込み、そして再度の読み取りを行い、
 * 値が正しく変更されたことを確認する方法を示します。
 *
 * - R/W (Read/Write): 読み書き可能なパラメータ
 * - R/O (Read Only)  : 読み取り専用のパラメータ
 *
 * @note
 * パラメータの変更は、`.set()` を呼び出した時点ではモーターのRAM上でのみ変更されます。
 * 電源を落としても設定を保持したい場合は、最後に `motor.saveParams()` を呼び出す必要があります。
 * ただし、フラッシュメモリの寿命を縮める可能性があるため、頻繁に呼び出すことは避けてください。
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

Motor motor1(MASTER_ID, MOTOR_SLAVE_ID);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);
  Serial.println("\n--- Utility: Read/Write Parameters Example ---");

  // CAN通信の初期化
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  if (!CAN.begin(CanBitRate::BR_1000k)) {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
  motor1.setCAN(&CAN);
  delay(100);

  Serial.println("Reading and Writing parameters...");

  // ----------------------------------------------------------------
  // 1. [R/W] float型パラメータ: UV_Value (低電圧保護のしきい値)
  // ----------------------------------------------------------------
  Serial.println("\n--- 1. UV_Value (float, R/W) ---");
  float original_uv = motor1.param.UV_Value.get();
  Serial.print("Original UV_Value: ");
  Serial.println(original_uv, 2);

  float new_uv = 12.5f;
  Serial.print("Setting UV_Value to: ");
  Serial.println(new_uv, 2);
  if (motor1.param.UV_Value.set(new_uv)) {
    Serial.println("-> Set command successful.");
  } else {
    Serial.println("-> Set command failed.");
  }

  float read_new_uv = motor1.param.UV_Value.get();
  Serial.print("Read back UV_Value: ");
  Serial.println(read_new_uv, 2);

  // ----------------------------------------------------------------
  // 2. [R/W] uint32_t型パラメータ: CTRL_MODE (制御モード)
  // ----------------------------------------------------------------
  Serial.println("\n--- 2. CTRL_MODE (uint32_t, R/W) ---");
  uint32_t original_ctrl_mode = motor1.param.CTRL_MODE.get();
  Serial.print("Original CTRL_MODE: 0x0");
  Serial.println(original_ctrl_mode, HEX);

  // Mode::VEL に相当する値 (0x02) を設定
  uint32_t new_ctrl_mode = 0x02;
  Serial.print("Setting CTRL_MODE to: 0x0");
  Serial.println(new_ctrl_mode, HEX);
  if (motor1.param.CTRL_MODE.set(new_ctrl_mode)) {
    Serial.println("-> Set command successful.");
  } else {
    Serial.println("-> Set command failed.");
  }

  uint32_t read_new_ctrl_mode = motor1.param.CTRL_MODE.get();
  Serial.print("Read back CTRL_MODE: 0x0");
  Serial.println(read_new_ctrl_mode, HEX);

  // ----------------------------------------------------------------
  // 3. [R/O] float型パラメータ: Rs (モーターの相抵抗)
  // ----------------------------------------------------------------
  Serial.println("\n--- 3. Rs (float, R/O) ---");
  float motor_resistance = motor1.param.Rs.get();
  Serial.print("Motor Resistance (Rs): ");
  Serial.println(motor_resistance, 4);

  // 以下の行のコメントを外すと、コンパイルエラーが発生します。
  // なぜなら、`.param.Rs` は読み取り専用(Read Only)であり、.set()メソッドを持たないためです。
  // motor1.param.Rs.set(1.23f);

  // ----------------------------------------------------------------
  // 4. [R/O] uint32_t型パラメータ: sw_ver (ソフトウェアバージョン)
  // ----------------------------------------------------------------
  Serial.println("\n--- 4. sw_ver (uint32_t, R/O) ---");
  uint32_t sw_version = motor1.param.sw_ver.get();
  Serial.print("Software Version (sw_ver): ");
  Serial.println(sw_version);

  // ----------------------------------------------------------------
  // 5. 設定の保存 (重要)
  // ----------------------------------------------------------------
  // 注意: `.set()` による変更はモーターのRAM上のみで有効です。電源を切ると失われます。
  //       電源を切っても設定を保持したい場合は、以下の `motor1.saveParams()` を
  //       コメント解除して呼び出し、フラッシュに書き込んでください。
  //       ただしフラッシュの書き込み回数には制限があり、寿命を縮める可能性があるため、
  //       ループの中に置いてしまうなどして頻繁に実行しないよう注意してください。
  //       CAN IDの変更も行えますが、変更後は新しいIDで通信を行う必要があります。
  Serial.println("\n--- 5. Saving Configuration ---");
  Serial.println("To permanently save the changes to the motor's flash memory,");
  Serial.println("uncomment the 'motor1.saveParams()' line below.");
  // Serial.print("Saving configuration to motor flash...");
  // if (motor1.saveParams()) {
  //   Serial.println(" -> Success!");
  // } else {
  //   Serial.println(" -> Failed!");
  // }

  Serial.println("\n--- Example Finished ---");
}

void loop() {
  // このサンプルではloop内で行うことは特にありません。
  // `motor.update()` を呼び出すと、CAN通信を読み続けることができます。
  // motor1.update();
  delay(1000);
}
