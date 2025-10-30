#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - motor_can1: '@motor_can1'
  - motor_can2: '@motor_can2'
  - task_stack_depth: 2048
  - pid_param_1:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
  - wheel_radius = 0.0f
  - wheel_to_center = 0.0f
  - gravity_height = 0.0f
  - wheel_resistance = 0.0f
  - error_compensation = 0.0f
template_args:
  - ChassisType: Helm<RMMotorContainer>
  - MotorType: RMMotorContainer
required_hardware:
  - dr16
  - motor
  - can
  - bmi088
depends:
  - qdu-future/CMD
  - qdu-future/BMI088
  - qdu-future/Motor
  - xrobot-org/MadgwickAHRS
=== END MANIFEST === */
// clang-format on

#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Motor.hpp"
#include "Omni.hpp"
#include "app_framework.hpp"
#include "pid.hpp"

template <typename ChassisType, typename MotorType>
class Chassis : public LibXR::Application {
 public:
  Chassis(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          CMD &cmd, Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
          uint32_t task_stack_depth,
          LibXR::PID<float>::Param pid_velocity_x_ = {.k = .0f,
                                                      .p = .0f,
                                                      .i = .0f,
                                                      .d = .0f,
                                                      .i_limit = .0f,
                                                      .out_limit = .0f,
                                                      .cycle = false},
          LibXR::PID<float>::Param pid_velocity_y_ = {.k = .0f,
                                                      .p = .0f,
                                                      .i = .0f,
                                                      .d = .0f,
                                                      .i_limit = .0f,
                                                      .out_limit = .0f,
                                                      .cycle = false},
          LibXR::PID<float>::Param pid_omega_ = {.k = .0f,
                                                 .p = .0f,
                                                 .i = .0f,
                                                 .d = .0f,
                                                 .i_limit = .0f,
                                                 .out_limit = .0f,
                                                 .cycle = false},
          LibXR::PID<float>::Param pid_wheel_angle_0_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_wheel_angle_1_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_wheel_angle_2_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_wheel_angle_3_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_steer_angle_0_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_steer_angle_1_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_steer_angle_2_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          LibXR::PID<float>::Param pid_steer_angle_3_ = {.k = .0f,
                                                         .p = .0f,
                                                         .i = .0f,
                                                         .d = .0f,
                                                         .i_limit = .0f,
                                                         .out_limit = .0f,
                                                         .cycle = false},
          float wheel_radius = 0.0f, float wheel_to_center = 0.0f, float gravity_height = 0.0f,
          float wheel_resistance = 0.0f, float error_compensation = 0.0f)
      : chassis_(hw, app, cmd, motor_can1, motor_can2, task_stack_depth,
                 pid_velocity_x_, pid_velocity_y_, pid_omega_,
                 pid_wheel_angle_0_, pid_wheel_angle_1_, pid_wheel_angle_2_,
                 pid_wheel_angle_3_, pid_steer_angle_0_, pid_steer_angle_1_,
                 pid_steer_angle_2_, pid_steer_angle_3_, wheel_radius, wheel_to_center,
                 gravity_height, wheel_resistance, error_compensation) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  void OnMonitor() override {}

 private:
  ChassisType chassis_;
};
