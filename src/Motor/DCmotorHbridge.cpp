#include "DCmotorHbridge.hpp"
#include "stmepic.hpp"
#include "status.hpp"
#include <cmath>
#include <cstdint>

using namespace stmepic::motor;
using namespace stmepic;

template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

DCMotorHBridge::DCMotorHBridge(TIM_HandleTypeDef &_htim, uint32_t _timer_channel, GpioPin &_direction_pin)
: htim(_htim), timer_channel(_timer_channel), direction_pin(_direction_pin) {
  this->max_velocity = 1.0f;
  this->min_velocity = 0.0f;
  this->gear_ratio   = 1.0f;
  this->reverse      = false;
  this->is_enabled   = false;

  this->current_position_cmd = 0.0f;
  this->current_velocity_cmd = 0.0f;
  this->current_torque_cmd   = 0.0f;
}

void DCMotorHBridge::set_velocity(float velocity) {


  if(std::abs(velocity) > this->max_velocity)
    velocity = sgn(velocity) * this->max_velocity;
  else if(std::abs(velocity) < this->min_velocity) {
    __HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), 0);
    current_velocity_cmd = 0.0f;
    return;
  }

  current_velocity_cmd = velocity;

  bool direction = !reverse;
  if(velocity > 0)
    direction_pin.write(direction);
  else {
    direction_pin.write(!direction);
    velocity = -velocity;
  }

  if(velocity == 0) {
    __HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), 0);
    return;
  }

  // calculate % of duty cycle
  float duty = velocity / this->max_velocity;
  if(duty > 1.0f)
    duty = 1.0f;

  // sets defined duty cycle
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim);
  uint32_t pulse  = (uint32_t)(duty * (float)period);
  __HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), pulse);
}

void DCMotorHBridge::set_torque(float torque) {

  if(std::abs(torque) > this->max_velocity) {
    torque = sgn(torque) * this->max_velocity;
  }

  current_torque_cmd = torque;

  bool direction = !reverse;
  if(torque > 0) {
    direction_pin.write(direction);
  } else {
    direction_pin.write(!direction);
    torque = -torque;
  }

  if(this->max_velocity <= 0.0f)
    return;

  float duty = torque / this->max_velocity;

  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim);
  uint32_t pulse  = (uint32_t)(duty * (float)period);
  __HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), pulse);
}

// not implemented
void DCMotorHBridge::set_position(float position) {
  current_position_cmd = position;
}

float DCMotorHBridge::get_velocity() const {
  return current_velocity_cmd;
}
float DCMotorHBridge::get_torque() const {
  return current_torque_cmd;
}
float DCMotorHBridge::get_gear_ratio() const {
  return gear_ratio;
}
float DCMotorHBridge::get_position() const {
  return current_position_cmd;
}
float DCMotorHBridge::get_absolute_position() const {
  return 0.0f;
}

void DCMotorHBridge::set_enable(bool enable) {
  if(enable)
    device_start();
  else
    device_stop();
}

void DCMotorHBridge::set_gear_ratio(float gear_ratio) {
  this->gear_ratio = gear_ratio;
}
void DCMotorHBridge::set_max_velocity(float max_velocity) {
  this->max_velocity = max_velocity;
}
void DCMotorHBridge::set_min_velocity(float min_velocity) {
  this->min_velocity = min_velocity;
}
void DCMotorHBridge::set_reverse(bool reverse) {
  this->reverse = reverse;
}

bool DCMotorHBridge::device_ok() {
  return true;
}
Result<bool> DCMotorHBridge::device_is_connected() {
  return Result<bool>::OK(true);
}
Status DCMotorHBridge::device_get_status() {
  return Status::OK();
}
Status DCMotorHBridge::device_reset() {
  return Status::OK();
}

Status DCMotorHBridge::device_start() {
  HAL_TIM_PWM_Start(&htim, timer_channel);
  is_enabled = true;
  return Status::OK();
}
Status DCMotorHBridge::device_stop() {
  __HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), 0);
  HAL_TIM_PWM_Stop(&htim, timer_channel);
  is_enabled = false;
  return Status::OK();
}
Status DCMotorHBridge::device_set_settings(const DeviceSettings &settings) {
  (void)settings;
  return Status::OK();
}