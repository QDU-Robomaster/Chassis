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
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
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
        cmd_(cmd) {
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

    // auto last_time = LibXR::Timebase::GetMilliseconds();
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");

    cmd_suber.StartWaiting();

    myomni->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        myomni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      myomni->mutex_.Lock();
      myomni->Update();
      myomni->UpdateCMD();
      myomni->SelfResolution();
      myomni->InverseKinematicsSolution();
      myomni->DynamicInverseSolution();
      myomni->mutex_.Unlock();
      myomni->OutputToDynamics();
      // myomni->OutputPowerLimit(80.0f);
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
    speed_rpm_[0] = motor_wheel_0_->GetRPM();
    speed_rpm_[1] = motor_wheel_1_->GetRPM();
    speed_rpm_[2] = motor_wheel_2_->GetRPM();
    speed_rpm_[3] = motor_wheel_3_->GetRPM();
  }

  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = mode;
    mutex_.Unlock();
  }

  void UpdateCMD() {
    const float SQRT2 = 1.41421356237f;

    target_vy_ =
        cmd_data_.y * MOTOR_MAX_OMEGA * this->PARAM.wheel_radius * SQRT2;
    target_vx_ =
        cmd_data_.x * MOTOR_MAX_OMEGA * this->PARAM.wheel_radius * SQRT2;
    target_omega_ =
        -cmd_data_.z * MOTOR_MAX_OMEGA * this->PARAM.wheel_to_center * SQRT2;
  }

  void LostCtrl() {
    motor_wheel_0_->Relax();
    motor_wheel_1_->Relax();
    motor_wheel_2_->Relax();
    motor_wheel_3_->Relax();
  }

  // 正解算 得到底盘速度
  void SelfResolution() {
    const float SQRT2 = 1.41421356237f;

    now_vx_ = (-motor_wheel_0_->GetOmega() - motor_wheel_1_->GetOmega() +
               motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_wheel_0_->GetOmega() - motor_wheel_1_->GetOmega() -
               motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_wheel_0_->GetOmega() + motor_wheel_1_->GetOmega() +
                  motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
                 SQRT2 * PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  // 底盘逆解算
  void InverseKinematicsSolution() {
    target_motor_omega_[0] =
        (-target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[1] =
        (-target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[2] =
        (target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[3] =
        (target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
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

  void OutputPowerLimit(float power_limit) {
    if (power_limit < 0.0f) {
      return;
    }
    chassis_power_ = 0.0f;
    text_1_ = 0.0f;
    text_2_ = 0.0f;
    text_3_ = 0.0f;
    for (int i = 0; i < 4; i++) {
      text_1_ += torque_coeff_ * output_current_[i] * speed_rpm_[i];
      text_2_ += speed_2_coeff_ * speed_rpm_[i] * speed_rpm_[i];
      text_3_ += out_2_coeff_ * output_current_[i] * output_current_[i];

      motor_power_[i] = torque_coeff_ * output_current_[i] * speed_rpm_[i] +
                        speed_2_coeff_ * speed_rpm_[i] * speed_rpm_[i] +
                        out_2_coeff_ * output_current_[i] * output_current_[i] +
                        const_coeff_;
      chassis_power_ += motor_power_[i];
    }

    if (chassis_power_ > power_limit) {
      float limit_ratio = power_limit / chassis_power_;
      for (int i = 0; i < 4; i++) {
        output_[i] *= limit_ratio;
      }
    }
  }

  void OutputToDynamics() {
    target_motor_current_[0] = pid_motor_omega_[0].Calculate(
        target_motor_omega_[0], motor_wheel_0_->GetOmega(), dt_);

    target_motor_current_[1] = pid_motor_omega_[1].Calculate(
        target_motor_omega_[1], motor_wheel_1_->GetOmega(), dt_);

    target_motor_current_[2] = pid_motor_omega_[2].Calculate(
        target_motor_omega_[2], motor_wheel_2_->GetOmega(), dt_);

    target_motor_current_[3] = pid_motor_omega_[3].Calculate(
        target_motor_omega_[3], motor_wheel_3_->GetOmega(), dt_);

    for (int i = 0; i < 4; i++) {
      output_[i] = (target_motor_current_[i] +
                    target_motor_force_[i] * PARAM.wheel_radius);

      output_current_[i] = output_[i] * 2730;
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
  float motor_power_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float chassis_power_ = 0.0f;

  float output_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float output_current_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float speed_rpm_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

  float text_1_;
  float text_2_;
  float text_3_;

  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  //   LibXR::PID<float> pid_motor_omega_[4]{0.0f, 0.0f, 0.0f, 0.0f};

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

  float torque_coeff_ = 1.996000e-06f;
  float speed_2_coeff_ = 9.0772349874e-07f;
  float out_2_coeff_ = 3.1993429496e-01f;
  float const_coeff_ = 2.3226238653f;

  CMD::ChassisCMD cmd_data_;
  uint32_t chassis_event_ = 0;

  LibXR::RamFS::File cmd_file_;
};
