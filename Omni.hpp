#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: No description provided
constructor_args: []
template_args: []
required_hardware: []
depends: []
=== END MANIFEST === */
// clang-format on
#include "CMD.hpp"
#include "Chassis.hpp"
#include "RMMotorContainer.hpp"
#include "app_framework.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "thread.hpp"

template <typename MotorType>
class Omni {
 public:
  Omni(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
       Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
       uint32_t task_stack_depth, LibXR::PID<float>::Param pid_param)
      : motor_can1_(&motor_can1),
        motor_can2_(&motor_can2),
        cmd_(&cmd),
        pid_param_(pid_param) {
    thread_.Create(this, ThreadFunction, "OmniThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
  }

  static void ThreadFunction(Omni *omni) {
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");

    cmd_suber.StartWaiting();
    while (true) {
      if (cmd_suber.Available()) {
        omni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      omni->SelfResolution();
      omni->KinematicsInverseResolution();
      omni->SteerMotorKinematicsNearestTransposition();
      omni->OutputToDynamics();
    }
  }

  void Update() {}

  void SelfResolution() {
    now_vx_ = 0.0f;
    now_vy_ = 0.0f;

    for (int i = 0; i < 4; i++) {
      now_vx_ += 0.0f;
      now_vy_ += 0.0f;
    }
  }

  void KinematicsInverseResolution() {}
  void SteerMotorKinematicsNearestTransposition() {}
  void OutputToDynamics() {}

 private:
  const float wheel_radius_ = 0.0f;

  const float wheel_to_center_ = 0.0f;

  const float gravity_height_ = 0.0f;

  float target_wheel_omega_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float target_wheel_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float static_wheel_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float dynamic_wheel_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float wheel_resistance_omega_threshold_ = 0.0f;

  float wheel_speed_limit_factor_ = 0.5f;

  float now_vx_ = 0.0f;
  float now_vy_ = 0.0f;

  float now_omega_ = 0.0f;

  float angle_pitch_ = 0.0f;
  float angle_roll_ = 0.0f;

  float slope_direction_x_ = 0.0f;
  float slope_direction_y_ = 0.0f;
  float slope_direction_z_ = 0.0f;

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;

  float target_omega_ = 0.0f;

  Motor<MotorType> *motor_can1_;
  Motor<MotorType> *motor_can2_;
  CMD *cmd_;
  LibXR::PID<float>::Param pid_param_;
  LibXR::Thread thread_;

  CMD::ChassisCMD cmd_data_;
};
