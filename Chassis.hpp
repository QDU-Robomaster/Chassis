#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args:
  - cmd: '@cmd'
  - motor_can1: '@motor_can1'
  - motor_can2: '@motor_can2'
  - task_stack_depth: 2048
  - pid_param:
      k: 1.0
      p: 0.0
      i: 0.0
      d: 0.0
      i_limit: 0.0
      out_limit: 0.0
      cycle: false
template_args:
  - ChassisType: Helm<RMMotorContainer>
  - MotorType: RMMotorContainer
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on

#include "Motor.hpp"
#include "app_framework.hpp"
#include "pid.hpp"
#include "CMD.hpp"
#include "Helm.hpp"
#include "Mecanum.hpp"
#include "Omni.hpp"


template <typename ChassisType, typename MotorType>
class Chassis : public LibXR::Application {
 public:
  Chassis(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          CMD &cmd, Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
          uint32_t task_stack_depth, LibXR::PID<float>::Param pid_param)
      : chassis_(hw, app, cmd, motor_can1, motor_can2, task_stack_depth, pid_param) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  void OnMonitor() override {}

 private:
  ChassisType chassis_;
};
