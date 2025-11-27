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

  Serial.println("Reading and Writing parameters...");

  // ----------------------------------------------------------------
  // 1. [R/W] float型パラメータ: UV_Value (低電圧保護のしきい値) 半開区間(10.0,fmax]
  // ----------------------------------------------------------------
  Serial.println("\n--- 1. UV_Value (float, R/W) ---");
  float original_uv = motor1.param.UV_Value.get();
  Serial.print("Original UV_Value: ");
  Serial.println(original_uv, 2);

  float new_uv = 10.1f;
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
  // 2. [R/W] uint32_t型パラメータ: CTRL_MODE (制御モード) MIT = 1, POS_VEL = 2, VEL = 3,
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
}
