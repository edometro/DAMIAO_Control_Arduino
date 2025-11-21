#pragma once

#include <optional>
#include <cstring>

#if !__has_include(<api/HardwareCAN.h>)
#error "DMMotor.h: include a HardwareCAN provider (e.g. <Arduino_CAN.h>, <ESP32_TWAI.h>, <RP2040PIO_CAN.h>) before this file."
#endif

#include <api/HardwareCAN.h>

#include "DMParams.h"

namespace damiao {

enum class Mode : uint8_t {
  UNREAD = 0,
  MIT = 1,
  POS_VEL = 2,
  VEL = 3,
  // POS_FORCE=4,
};

enum class Status : uint8_t {
  DISABLED = 0x00,
  ENABLED = 0x01,
  SENSOR_ERROR = 0x05,
  PARAM_ERROR = 0x06,
  OVER_VOLTAGE = 0x08,
  UNDER_VOLTAGE = 0x09,
  OVER_CURRENT = 0x0A,
  MOS_OVER_TEMP = 0x0B,
  COIL_OVER_TEMP = 0x0C,
  COMM_LOST = 0x0D,
  OVER_LOAD = 0x0E,
};

class Motor {
public:
  Motor(uint32_t masterId, uint32_t slaveId, arduino::HardwareCAN* can = nullptr)
    : masterId_(masterId), slaveId_(slaveId), can_(can), param(*this) {}

  DMParams param;  // Add public param member

  void setCAN(arduino::HardwareCAN* can) {
    can_ = can;
  }

  void readFeedback(const CanMsg& msg) {
    auto uintToFloat = [](uint16_t x, float min_val, float max_val, int bits) {
      float span = max_val - min_val;
      float normalized = static_cast<float>(x) / static_cast<float>((1 << bits) - 1);
      return normalized * span + min_val;
    };

    if (msg.getStandardId() != masterId_) { return; }
    uint8_t motor_id = static_cast<uint8_t>(msg.data[0] & 0x0F);
    if (motor_id != (slaveId_ & 0x0F)) { return; }
    feedback_.status = static_cast<Status>((msg.data[0] >> 4) & 0x0F);
    uint16_t position_raw = static_cast<uint16_t>(msg.data[1] << 8 | msg.data[2]);
    uint16_t velocity_raw = static_cast<uint16_t>((msg.data[3] << 4) | (msg.data[4] >> 4));
    uint16_t tau_raw = static_cast<uint16_t>(((msg.data[4] & 0x0F) << 8) | msg.data[5]);
    feedback_.temp_mos = static_cast<int8_t>(msg.data[6]);
    feedback_.temp_rotor = static_cast<int8_t>(msg.data[7]);
    feedback_.position = uintToFloat(position_raw, -getPMAX(), getPMAX(), 16);
    feedback_.velocity = uintToFloat(velocity_raw, -getVMAX(), getVMAX(), 12);
    feedback_.torque = uintToFloat(tau_raw, -getTMAX(), getTMAX(), 12);
  }

  void update() {
    if (can_ == nullptr) return;
    while (can_->available()) {
      CanMsg msg = can_->read();
      readFeedback(msg);
    }
  }

  bool enable() {
    return controlCommand(Command_Code::ENABLE);
  }
  bool disable() {
    return controlCommand(Command_Code::DISABLE);
  }
  bool setZeroPosition() {
    return controlCommand(Command_Code::SET_ZERO);
  }
  bool setControlMode(Mode mode) {
    if (mode != Mode::UNREAD) return param.CTRL_MODE.set(static_cast<uint32_t>(mode));
    return false;
  }
  Mode getMode() {
    if (currentMode_ == Mode::UNREAD) {
      uint32_t mode_val = param.CTRL_MODE.get();
      if (mode_val != static_cast<uint32_t>(Mode::UNREAD)) {
        currentMode_ = static_cast<Mode>(mode_val);
      }
    }
    return currentMode_;
  }
  float getPMAX() {
    if (map_.pmax != 0.0f) return map_.pmax;
    return param.PMAX.get();
  }
  float getVMAX() {
    if (map_.vmax != 0.0f) return map_.vmax;
    return param.VMAX.get();
  }
  float getTMAX() {
    if (map_.tmax != 0.0f) return map_.tmax;
    return param.TMAX.get();
  }
  Status getStatus() const {
    return feedback_.status;
  }
  uint8_t getStatusNum() const {
    return static_cast<uint8_t>(feedback_.status);
  }
  // Output shaft position [rad]
  float getPosition() const {
    return feedback_.position;
  }
  // Output shaft velocity [rad/s]
  float getVelocity() const {
    return feedback_.velocity;
  }
  // Output shaft torque [Nm]
  float getTorque() const {
    return feedback_.torque;
  }
  // ESC MOS-FET Temperature [°C]
  int8_t getMosTemp() const {
    return feedback_.temp_mos;
  }
  // Rotor Temperature [°C]
  int8_t getRotorTemp() const {
    return feedback_.temp_rotor;
  }
  uint32_t getSlaveId() const {
    return slaveId_;
  }
  uint32_t getMasterId() const {
    return masterId_;
  }
  // Get motor feedback mapping ranges
  void initialize() {
    getPMAX();
    getVMAX();
    getTMAX();
  }

  std::optional<uint32_t> readParamUint32(RID id, std::optional<uint32_t> timeout_ms = std::nullopt) {
    uint32_t raw_val;
    if (!readParamRaw(id, raw_val, timeout_ms)) {
      return std::nullopt;
    }
    updateCachedParam(id, raw_val);
    return raw_val;
  }

  std::optional<float> readParamFloat(RID id, std::optional<uint32_t> timeout_ms = std::nullopt) {
    auto raw_val = readParamUint32(id, timeout_ms);
    if (!raw_val) {
      return std::nullopt;
    }
    float float_val;
    std::memcpy(&float_val, &(*raw_val), sizeof(float));
    return float_val;
  }

  bool writeParamUint32(RID id, uint32_t value, std::optional<uint32_t> timeout_ms = std::nullopt) {
    if (!sendParamRaw(id, value, timeout_ms)) {
      return false;
    }
    updateCachedParam(id, value);
    return true;
  }

  bool writeParamFloat(RID id, float value, std::optional<uint32_t> timeout_ms = std::nullopt) {
    uint32_t uint_value;
    std::memcpy(&uint_value, &value, sizeof(uint32_t));
    return writeParamUint32(id, uint_value, timeout_ms);
  }

  //  電源OFFでも値が消えない フラッシュの寿命を縮めるので注意して使う
  bool saveParams(std::optional<uint32_t> timeout_ms = std::nullopt) {
    if (can_ == nullptr) return false;
    CanMsg tx = makeParamTx(Parameter_Command_Id::SAVE, Parameter_Command_Code::SAVE);
    if (can_->write(tx) < 0) return false;

    CanMsg rx;
    if (!receiveParam(Parameter_Command_Code::SAVE, rx, timeout_ms)) return false;
    if (rx.data[3] != 1) return false;
    return true;
  }

  bool sendMIT(float position_rad, float velocity_rad_s, float kp, float kd, float torque_nm) {
    auto floatToUint = [](float x, float min_val, float max_val, int bits) {
      float span = max_val - min_val;
      float clamped_x = constrain(x, min_val, max_val);
      float normalized = (clamped_x - min_val) / span;
      return static_cast<uint16_t>(normalized * ((1 << bits) - 1));
    };

    uint16_t position_raw = floatToUint(position_rad, -getPMAX(), getPMAX(), 16);
    uint16_t velocity_raw = floatToUint(velocity_rad_s, -getVMAX(), getVMAX(), 12);
    uint16_t kp_raw = floatToUint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_raw = floatToUint(kd, 0.0f, 5.0f, 12);
    uint16_t torque_raw = floatToUint(torque_nm, -getTMAX(), getTMAX(), 12);
    CanMsg tx = {};
    tx.id = slaveId_ + static_cast<uint32_t>(Control_Mode_Offset::MIT_MODE);
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(position_raw >> 8);
    tx.data[1] = static_cast<uint8_t>(position_raw & 0xFF);
    tx.data[2] = static_cast<uint8_t>(velocity_raw >> 4);
    tx.data[3] = static_cast<uint8_t>((velocity_raw & 0x0F) << 4) | (kp_raw >> 8);
    tx.data[4] = static_cast<uint8_t>(kp_raw & 0xFF);
    tx.data[5] = static_cast<uint8_t>(kd_raw >> 4);
    tx.data[6] = static_cast<uint8_t>((kd_raw & 0x0F) << 4) | (torque_raw >> 8);
    tx.data[7] = static_cast<uint8_t>(torque_raw & 0xFF);
    return can_->write(tx) >= 0;
  }

  bool sendPosition(float position_rad, float velocity_rad_s_limit) {
    uint32_t position_raw, velocity_raw;
    memcpy(&position_raw, &position_rad, sizeof(position_raw));
    memcpy(&velocity_raw, &velocity_rad_s_limit, sizeof(velocity_raw));
    CanMsg tx = {};
    tx.id = slaveId_ + static_cast<uint32_t>(Control_Mode_Offset::POS_VEL_MODE);
    tx.data_length = 8;
    tx.data[0] = static_cast<uint8_t>(position_raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((position_raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>((position_raw >> 16) & 0xFF);
    tx.data[3] = static_cast<uint8_t>((position_raw >> 24) & 0xFF);
    tx.data[4] = static_cast<uint8_t>(velocity_raw & 0xFF);
    tx.data[5] = static_cast<uint8_t>((velocity_raw >> 8) & 0xFF);
    tx.data[6] = static_cast<uint8_t>((velocity_raw >> 16) & 0xFF);
    tx.data[7] = static_cast<uint8_t>((velocity_raw >> 24) & 0xFF);
    return can_->write(tx) >= 0;
  }

  bool sendVelocity(float velocity_rad_s) {
    uint32_t velocity_raw;
    memcpy(&velocity_raw, &velocity_rad_s, sizeof(velocity_raw));
    CanMsg tx = {};
    tx.id = slaveId_ + static_cast<uint32_t>(Control_Mode_Offset::VEL_MODE);
    tx.data_length = 4;
    tx.data[0] = static_cast<uint8_t>(velocity_raw & 0xFF);
    tx.data[1] = static_cast<uint8_t>((velocity_raw >> 8) & 0xFF);
    tx.data[2] = static_cast<uint8_t>((velocity_raw >> 16) & 0xFF);
    tx.data[3] = static_cast<uint8_t>((velocity_raw >> 24) & 0xFF);
    return can_->write(tx) >= 0;
  }

private:
  static const uint32_t DEFAULT_PARAM_TIMEOUT_MS = 30;

  enum class Control_Mode_Offset : uint32_t {
    MIT_MODE = 0x000,
    POS_VEL_MODE = 0x100,
    VEL_MODE = 0x200,
    // POS_FORCE_MODE = 0x300,
  };

  enum class Command_Code : uint8_t {
    ENABLE = 0xFC,
    DISABLE = 0xFD,
    SET_ZERO = 0xFE,
  };

  enum class Parameter_Command_Id : uint32_t {
    READ = 0x7FF,
    WRITE = 0x7FF,
    SAVE = 0x7FF,
  };

  enum class Parameter_Command_Code : uint8_t {
    READ = 0x33,
    WRITE = 0x55,
    SAVE = 0xAA,
  };

  struct Feedback {
    Status status = Status::DISABLED;
    float position = 0.0f;  //rad
    float velocity = 0.0f;  //rad/s
    float torque = 0.0f;    //Nm
    int8_t temp_mos = 0;    //°C
    int8_t temp_rotor = 0;  //°C
  };

  struct MappingRange {
    float pmax = 0.0f, vmax = 0.0f, tmax = 0.0f;
  };

  Feedback feedback_;
  MappingRange map_;
  arduino::HardwareCAN* can_;
  uint32_t masterId_;
  uint32_t slaveId_;
  Mode currentMode_ = Mode::UNREAD;

  bool controlCommand(Command_Code cmd) {
    CanMsg tx = {};
    tx.id = slaveId_;
    tx.data_length = 8;
    for (uint8_t i = 0; i < 7; i++) { tx.data[i] = 0xFF; }
    tx.data[7] = static_cast<uint8_t>(cmd);
    return can_->write(tx) >= 0;
  }

  void updateCachedParam(RID rid, uint32_t raw_val) {
    switch (rid) {
      case RID::PMAX:
        {
          float val;
          std::memcpy(&val, &raw_val, sizeof(val));
          map_.pmax = val;
          break;
        }
      case RID::VMAX:
        {
          float val;
          std::memcpy(&val, &raw_val, sizeof(val));
          map_.vmax = val;
          break;
        }
      case RID::TMAX:
        {
          float val;
          std::memcpy(&val, &raw_val, sizeof(val));
          map_.tmax = val;
          break;
        }
      case RID::MST_ID:
        {
          masterId_ = raw_val;
          break;
        }
      case RID::ESC_ID:
        {
          slaveId_ = raw_val;
          break;
        }
      case RID::CTRL_MODE:
        {
          currentMode_ = static_cast<Mode>(raw_val);
          break;
        }
      default:
        break;
    }
  }

  bool readParamRaw(RID rid, uint32_t& out, std::optional<uint32_t> timeout_ms = std::nullopt) {
    if (can_ == nullptr) return false;
    CanMsg tx = makeParamTx(Parameter_Command_Id::READ, Parameter_Command_Code::READ, rid);
    if (can_->write(tx) < 0) return false;

    CanMsg rx;
    if (!receiveParam(Parameter_Command_Code::READ, rx, timeout_ms, rid)) return false;

    out = static_cast<uint32_t>(rx.data[4]) | (static_cast<uint32_t>(rx.data[5]) << 8) | (static_cast<uint32_t>(rx.data[6]) << 16) | (static_cast<uint32_t>(rx.data[7]) << 24);
    return true;
  }

  bool sendParamRaw(RID rid, uint32_t txval, std::optional<uint32_t> timeout_ms = std::nullopt) {
    if (can_ == nullptr) return false;
    uint8_t payload[4] = {
      static_cast<uint8_t>(txval & 0xFF),
      static_cast<uint8_t>((txval >> 8) & 0xFF),
      static_cast<uint8_t>((txval >> 16) & 0xFF),
      static_cast<uint8_t>((txval >> 24) & 0xFF),
    };
    CanMsg tx = makeParamTx(Parameter_Command_Id::WRITE, Parameter_Command_Code::WRITE, rid, payload);
    if (can_->write(tx) < 0) return false;

    CanMsg rx;
    if (!receiveParam(Parameter_Command_Code::WRITE, rx, timeout_ms, rid)) return false;  // データシートのWrite後に返ってくるdata[2]は間違ってる 0x55が正しい

    uint32_t rx_val = static_cast<uint32_t>(rx.data[4]) | (static_cast<uint32_t>(rx.data[5]) << 8) | (static_cast<uint32_t>(rx.data[6]) << 16) | (static_cast<uint32_t>(rx.data[7]) << 24);
    return rx_val == txval;
  }

  CanMsg makeParamTx(Parameter_Command_Id id, Parameter_Command_Code code, std::optional<RID> rid = std::nullopt, const uint8_t* payload = nullptr) {
    CanMsg tx{};
    tx.id = static_cast<uint32_t>(id);
    tx.data_length = (code == Parameter_Command_Code::SAVE) ? 4 : 8;
    tx.data[0] = static_cast<uint8_t>(slaveId_ & 0xFF);         // id_low
    tx.data[1] = static_cast<uint8_t>((slaveId_ >> 8) & 0xFF);  // id_high
    tx.data[2] = static_cast<uint8_t>(code);                    // command_code
    // RIDが空のときはSAVE時 data[3]は無視されるので0xFFを入れておく
    tx.data[3] = rid.has_value() ? static_cast<uint8_t>(*rid) : static_cast<uint8_t>(0xFF);
    if (tx.data_length == 8) {
      if (payload) {
        std::memcpy(&tx.data[4], payload, 4);
      } else {
        for (uint8_t i = 4; i < 8; i++) { tx.data[i] = 0xFF; }  // readParamではdata[4]-data[7]は無視
      }
    }
    return tx;
  }

  bool receiveParam(Parameter_Command_Code expected_code, CanMsg& out_rx, std::optional<uint32_t> timeout_ms_opt = std::nullopt, std::optional<RID> expected_rid = std::nullopt) {
    uint32_t timeout_ms = timeout_ms_opt.value_or(DEFAULT_PARAM_TIMEOUT_MS);  // Use the new default constant
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      if (can_->available()) {
        CanMsg rx = can_->read();
        if (rx.getStandardId() != masterId_) continue;
        uint32_t rx_slave = static_cast<uint32_t>(rx.data[0]) | (static_cast<uint32_t>(rx.data[1]) << 8);
        if (rx_slave != slaveId_) continue;
        if (rx.data[2] != static_cast<uint8_t>(expected_code)) continue;
        const uint8_t min_len = (expected_code == Parameter_Command_Code::SAVE) ? 4 : 8;
        if (rx.data_length < min_len) continue;
        if (expected_code != Parameter_Command_Code::SAVE && expected_rid.has_value() && rx.data[3] != static_cast<uint8_t>(*expected_rid)) continue;
        out_rx = rx;
        return true;
      }
    }
    return false;
  }
};

}  // namespace damiao

#include "DMParams_inline.h"
