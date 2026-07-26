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

DCMotorHBridge::DCMotorHBridge(TIM_HandleTypeDef &_htim, uint32_t _timer_channel, GpioPin &_direction_pin):
htim(_htim), timer_channel(_timer_channel), direction_pin(_direction_pin), gear_ratio(1.0f), reverse(false), is_enabled(false){
  
  DCMotorPWMSettings default_DC_settings;
  default_DC_settings.max_velocity_setting = 100.0f; // zależne od silnika
  default_DC_settings.min_velocity_setting = 0.0f; // zależne od silnika
  default_DC_settings.max_torque_setting = 20.0f; // zależne od silnika
  default_DC_settings.min_torque_setting = 0.0f; // zależne od silnika
  
  default_DC_settings.min_pulse_width_us = 0.0f;
  default_DC_settings.max_pulse_width_us = 2500.0f;
  default_DC_settings.pwm_frequency = 1000.0f;
  (void)device_set_settings(default_DC_settings);
}

Status DCMotorHBridge::device_set_settings(const DeviceSettings &_settings) {
  const auto *dc_settings = dynamic_cast<const DCMotorPWMSettings *>(&_settings);
  if (!dc_settings) return Status::TypeError("Invalid settings type");

  settings = *dc_settings;
  return Status::OK();
}

void DCMotorHBridge::set_velocity(float velocity) {
	if (!is_enabled) return;
	  
	  //jeżeli jest przekładnia
	  velocity *= gear_ratio;
	  
	  //sprawdzenie przekroczenia założonej prędkości
	  if(std::abs(velocity) > settings.max_velocity_setting)
		velocity = sgn(velocity) * settings.max_velocity_setting;
	  else if(std::abs(velocity) < settings.min_velocity_setting) {
		__HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), 0);
		current_velocity_cmd = 0.0f;
		return;
	  }
	
	  //ustawienie prędkości
	  current_velocity_cmd = velocity;
	  bool direction = !reverse;
	  
	  //ustawienie kierunku
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

	  // wyliczenie PWM
	  float duty_velocity = velocity / settings.max_velocity_setting;
	  if(duty_velocity > 1.0f)
		duty_velocity = 1.0f;

	  // ustawia PWM
	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim);
	  uint32_t pulse  = (uint32_t)(duty_velocity * (float)period);
	  __HAL_TIM_SET_COMPARE(&htim, static_cast<uint32_t>(timer_channel), pulse);
}

void DCMotorHBridge::set_torque(float torque) {

	if (!is_enabled) return;
	
	  torque *= gear_ratio;
	
	  if(std::abs(torque) > settings.max_torque_setting) {
		torque = sgn(torque) * settings.max_torque_setting;
	  }

	  current_torque_cmd = torque;

	  bool direction = !reverse;
	  if(torque > 0) {
		direction_pin.write(direction);
	  } else {
		direction_pin.write(!direction);
		torque = -torque;
	  }

	  if(settings.max_torque_setting <= 0.0f)
		return;

	  float duty_torque = torque / settings.max_torque_setting;

	  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim);
	  uint32_t pulse  = (uint32_t)(duty_torque * (float)period);
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
  settings.max_velocity_setting = max_velocity;
}
void DCMotorHBridge::set_min_velocity(float min_velocity) {
  settings.min_velocity_setting = min_velocity;
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