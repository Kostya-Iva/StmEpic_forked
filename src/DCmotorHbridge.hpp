#pragma once

#include "motor.hpp"
#include "gpio.hpp"
#include "stmepic.hpp"

namespace stmepic::motor {

struct DCMotorPWMSettings : public DeviceSettings {
  float max_velocity_setting; 
  float min_velocity_setting;
  float max_torque_setting; 
  float min_torque_setting; 

  float current_velocity_setting;
  float current_torque_setting;

  float min_pulse_width_us; // Minimum pulse width in micro seconds [us]
  float max_pulse_width_us; // Maximum pulse width in micro seconds [us]
  float pwm_frequency;      // Frequency of the PWM signal in Hz [Hz]
};

class DCMotorHBridge : public MotorBase {

public:
  DCMotorHBridge(TIM_HandleTypeDef &htim, uint32_t timer_channel, GpioPin &direction_pin);

  void set_position(float position) override;
  
  void set_velocity(float velocity) override;
  void set_torque(float torque) override;

  void set_enable(bool enable) override;

  void set_gear_ratio(float gear_ratio) override;
  void set_max_velocity(float max_velocity) override;
  void set_min_velocity(float min_velocity) override;
  void set_reverse(bool reverse) override;

  [[nodiscard]] float get_position() const override;
  [[nodiscard]] float get_velocity() const override;
  [[nodiscard]] float get_torque() const override;
  [[nodiscard]] float get_absolute_position() const override;
  [[nodiscard]] float get_gear_ratio() const override;

  [[nodiscard]] bool device_ok() override;
  [[nodiscard]] Result<bool> device_is_connected() override;

  [[nodiscard]] Status device_get_status() override;
  [[nodiscard]] Status device_reset() override;
  [[nodiscard]] Status device_start() override;
  [[nodiscard]] Status device_stop() override;
  [[nodiscard]] Status device_set_settings(const DeviceSettings &settings) override;

private:
  TIM_HandleTypeDef &htim;
  unsigned int timer_channel;
  GpioPin &direction_pin;

  float max_velocity;
  float min_velocity;
  float gear_ratio;
  bool reverse;
  bool is_enabled;

  float current_position_cmd;
  float current_velocity_cmd;
  float current_torque_cmd;
  DCMotorPWMSettings settings;
};

}