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

#include <sys/_types.h>

#include <cmath>
#include <cstdint>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "thread.hpp"


#define MOTOR_MAX_OMEGA 52 /* 电机输出轴最大角速度 */

template <typename ChassisType>
class Chassis;

class MyOmni {
 public:
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reductionratio = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
  };
  struct MotorData {
    float output_current[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float rotorspeed_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float target_motor_omega_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float current_motor_omega_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  };

  /*
  5  8
  7  1
  */
  MyOmni(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
         RMMotor *motor_wheel_0, RMMotor *motor_wheel_1, RMMotor *motor_wheel_2,
         RMMotor *motor_wheel_3, RMMotor *motor_steer_0, RMMotor *motor_steer_1,
         RMMotor *motor_steer_2, RMMotor *motor_steer_3, CMD *cmd,
         uint32_t task_stack_depth, ChassisParam chassis_param,
         LibXR::PID<float>::Param pid_velocity_x,
         LibXR::PID<float>::Param pid_velocity_y,
         LibXR::PID<float>::Param pid_omega,
         LibXR::PID<float>::Param pid_motor_omega_0,
         LibXR::PID<float>::Param pid_motor_omega_1,
         LibXR::PID<float>::Param pid_motor_omega_2,
         LibXR::PID<float>::Param pid_motor_omega_3,
         LibXR::PID<float>::Param pid_steer_angle_0,
         LibXR::PID<float>::Param pid_steer_angle_1,
         LibXR::PID<float>::Param pid_steer_angle_2,
         LibXR::PID<float>::Param pid_steer_angle_3)
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_motor_omega_{pid_motor_omega_0, pid_motor_omega_1,
                         pid_motor_omega_2, pid_motor_omega_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3},
        cmd_(cmd),
        topic_motor_data_("motor_data", sizeof(motor_data_)) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(motor_steer_0);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);

    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, MyOmni *myomni, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          myomni->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  static void ThreadFunction(MyOmni *myomni) {
    myomni->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<PowerControlData> powercontrol_data_suber(
        "powercontrol_data");

    cmd_suber.StartWaiting();
    powercontrol_data_suber.StartWaiting();

    myomni->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        myomni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (powercontrol_data_suber.Available()) {
        auto power_data = powercontrol_data_suber.GetData();

        LibXR::Memory::FastCopy(myomni->new_output_current_,
                                power_data.new_output_current,
                                sizeof(myomni->new_output_current_));
        myomni->is_power_limited_ = power_data.enable;

        powercontrol_data_suber.StartWaiting();
      }
      myomni->mutex_.Lock();
      myomni->Update();
      myomni->UpdateCMD();
      myomni->SelfResolution();
      myomni->InverseKinematicsSolution();
      myomni->DynamicInverseSolution();
      myomni->topic_motor_data_.Publish(myomni->motor_data_);
      myomni->mutex_.Unlock();
      myomni->OutputToDynamics();
      myomni->thread_.Sleep(2);
    }
  }

  // 更新电机反馈数据
  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;

    motor_wheel_0_->Update();
    motor_wheel_1_->Update();
    motor_wheel_2_->Update();
    motor_wheel_3_->Update();
    /*给功率控制的数据*/
    motor_data_.rotorspeed_rpm[0] = motor_wheel_0_->GetRPM();
    motor_data_.rotorspeed_rpm[1] = motor_wheel_1_->GetRPM();
    motor_data_.rotorspeed_rpm[2] = motor_wheel_2_->GetRPM();
    motor_data_.rotorspeed_rpm[3] = motor_wheel_3_->GetRPM();

    motor_data_.current_motor_omega_[0] =
        motor_wheel_0_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_[1] =
        motor_wheel_1_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_[2] =
        motor_wheel_2_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_[3] =
        motor_wheel_3_->GetOmega() / PARAM.reductionratio;

    motor_data_.target_motor_omega_[0] = target_motor_omega_[0];
    motor_data_.target_motor_omega_[1] = target_motor_omega_[1];
    motor_data_.target_motor_omega_[2] = target_motor_omega_[2];
    motor_data_.target_motor_omega_[3] = target_motor_omega_[3];
  }

  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = mode;
    mutex_.Unlock();
  }

  void UpdateCMD() {
    const float SQRT2 = 1.41421356237f;

    target_vy_ =
        cmd_data_.x * MOTOR_MAX_OMEGA * this->PARAM.wheel_radius * SQRT2;
    target_vx_ =
        -cmd_data_.y * MOTOR_MAX_OMEGA * this->PARAM.wheel_radius * SQRT2;
    target_omega_ =
        -cmd_data_.z * MOTOR_MAX_OMEGA * this->PARAM.wheel_to_center * SQRT2;
  }

  void LostCtrl() {
    motor_wheel_0_->Relax();
    motor_wheel_1_->Relax();
    motor_wheel_2_->Relax();
    motor_wheel_3_->Relax();
  }

  void SelfResolution() {
    const float SQRT2 = 1.41421356237f;

    now_vx_ = (-motor_wheel_0_->GetOmega() / PARAM.reductionratio -
               motor_wheel_1_->GetOmega() / PARAM.reductionratio +
               motor_wheel_2_->GetOmega() / PARAM.reductionratio +
               motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio -
               motor_wheel_1_->GetOmega() / PARAM.reductionratio -
               motor_wheel_2_->GetOmega() / PARAM.reductionratio +
               motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_1_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_2_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
                 SQRT2 * PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  void InverseKinematicsSolution() {
    const float SQRT1 = 0.70710678118f;
    target_motor_omega_[0] = (-target_vx_ * SQRT1 + target_vy_ * SQRT1 +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[1] = (-target_vx_ * SQRT1 - target_vy_ * SQRT1 +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[2] = (target_vx_ * SQRT1 - target_vy_ * SQRT1 +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[3] = (target_vx_ * SQRT1 + target_vy_ * SQRT1 +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
  }

  void DynamicInverseSolution() {
    float force_x = pid_velocity_x_.Calculate(target_vx_, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, now_vy_, dt_);
    float torque = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    target_motor_force_[0] =
        (-force_x / 4 / PARAM.wheel_radius + force_y / 4 / PARAM.wheel_radius +
         torque * PARAM.wheel_to_center / 4 / PARAM.wheel_radius);
    target_motor_force_[1] =
        (-force_x / 4 / PARAM.wheel_radius - force_y / 4 / PARAM.wheel_radius +
         torque * PARAM.wheel_to_center / 4 / PARAM.wheel_radius);
    target_motor_force_[2] =
        (force_x / 4 / PARAM.wheel_radius - force_y / 4 / PARAM.wheel_radius +
         torque * PARAM.wheel_to_center / 4 / PARAM.wheel_radius);
    target_motor_force_[3] =
        (force_x / 4 / PARAM.wheel_radius + force_y / 4 / PARAM.wheel_radius +
         torque * PARAM.wheel_to_center / 4 / PARAM.wheel_radius);
  }

  void OutputToDynamics() {
    target_motor_current_[0] = pid_motor_omega_[0].Calculate(
        target_motor_omega_[0],
        motor_wheel_0_->GetOmega() / PARAM.reductionratio, dt_);

    target_motor_current_[1] = pid_motor_omega_[1].Calculate(
        target_motor_omega_[1],
        motor_wheel_1_->GetOmega() / PARAM.reductionratio, dt_);

    target_motor_current_[2] = pid_motor_omega_[2].Calculate(
        target_motor_omega_[2],
        motor_wheel_2_->GetOmega() / PARAM.reductionratio, dt_);

    target_motor_current_[3] = pid_motor_omega_[3].Calculate(
        target_motor_omega_[3],
        motor_wheel_3_->GetOmega() / PARAM.reductionratio, dt_);

    for (int i = 0; i < 4; i++) {
      output_[i] = (target_motor_current_[i] +
                    target_motor_force_[i] * PARAM.wheel_radius);

      motor_data_.output_current[i] =
          output_[i] *
          (motor_wheel_0_->GetLSB() / PARAM.reductionratio /
           motor_wheel_0_->KGetTorque() / motor_wheel_0_->GetCurrentMAX());
    }

    if (is_power_limited_) {
      for (int i = 0; i < 4; i++) {
        output_[i] =
            new_output_current_[i] /
            (motor_wheel_0_->GetLSB() / PARAM.reductionratio /
             motor_wheel_0_->KGetTorque() / motor_wheel_0_->GetCurrentMAX());
      }
    }

    motor_wheel_0_->TorqueControl(output_[0], PARAM.reductionratio);
    motor_wheel_1_->TorqueControl(output_[1], PARAM.reductionratio);
    motor_wheel_2_->TorqueControl(output_[2], PARAM.reductionratio);
    motor_wheel_3_->TorqueControl(output_[3], PARAM.reductionratio);
  }

 private:
  const ChassisParam PARAM;

  float target_motor_omega_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_force_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  float output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float new_output_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  bool is_power_limited_ = false;

  float now_vx_ = 0.0f;
  float now_vy_ = 0.0f;
  float now_omega_ = 0.0f;

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  RMMotor *motor_wheel_0_;
  RMMotor *motor_wheel_1_;
  RMMotor *motor_wheel_2_;
  RMMotor *motor_wheel_3_;

  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_motor_omega_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_angle_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  CMD *cmd_;
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;  // 互斥量
  LibXR::Topic topic_motor_data_;

  MotorData motor_data_;

  CMD::ChassisCMD cmd_data_;
  uint32_t chassis_event_ = 0;

  LibXR::RamFS::File cmd_file_;
};
