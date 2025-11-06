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
#include <algorithm>
#include <cmath>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"

#define MOTOR_MAX_ROTATIONAL_SPEED 9600 /* 电机最大转速 */

template <typename MotorType>
class Mecanum {
 public:
  Mecanum(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          CMD &cmd, Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
          uint32_t task_stack_depth, float wheel_radius, float wheel_to_center,
          float gravity_height, float wheel_resistance,
          float error_compensation, LibXR::PID<float>::Param pid_velocity_x,
          LibXR::PID<float>::Param pid_velocity_y,
          LibXR::PID<float>::Param pid_omega,
          LibXR::PID<float>::Param pid_wheel_omega_0,
          LibXR::PID<float>::Param pid_wheel_omega_1,
          LibXR::PID<float>::Param pid_wheel_omega_2,
          LibXR::PID<float>::Param pid_wheel_omega_3,
          LibXR::PID<float>::Param pid_steer_angle_0,
          LibXR::PID<float>::Param pid_steer_angle_1,
          LibXR::PID<float>::Param pid_steer_angle_2,
          LibXR::PID<float>::Param pid_steer_angle_3)
      : r_wheel_(wheel_radius),
        r_center_(wheel_to_center),
        g_height_(gravity_height),
        wheel_resistance_(wheel_resistance),
        error_compensation_(error_compensation),
        motor_can1_(&motor_can1),
        motor_can2_(&motor_can2),
        cmd_(&cmd),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_omega_{pid_wheel_omega_0, pid_wheel_omega_1,
                         pid_wheel_omega_2, pid_wheel_omega_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3} {
    UNUSED(hw);
    UNUSED(app);

    thread_.Create(this, ThreadFunction, "MecanumChassisThread",
                   task_stack_depth, LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(Mecanum *mecanum) {
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");

    cmd_suber.StartWaiting();
    while (true) {
      if (cmd_suber.Available()) {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      mecanum->mutex_.Lock();
      mecanum->Update();
      mecanum->UpdateSetpointFromCMD();
      mecanum->SelfResolution();
      mecanum->InverseKinematicsSolution();
      mecanum->DynamicInverseSolution();
      mecanum->mutex_.Unlock();
      mecanum->OutputToDynamics();

      mecanum->thread_.Sleep(2);

      // auto last_time = LibXR::Timebase::GetMilliseconds();
      // mecanum->thread_.SleepUntil(last_time, 2.0f);
    }
  }

  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    motor_can1_->Update();
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = mode;
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CDM获取底盘控制指令
   */
  void UpdateSetpointFromCMD() {
    target_vx_ = cmd_data_.x;
    target_vy_ = cmd_data_.y;
    target_omega_ = cmd_data_.z;
  }

  /**
   * @brief 麦轮底盘正运动学解算
   * @details 根据四个麦轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    now_vx_ = (-motor_can1_->GetSpeed(0) - motor_can1_->GetSpeed(1) +
               motor_can1_->GetSpeed(2) + motor_can1_->GetSpeed(3)) *
              r_wheel_ / 4.0f;
    now_vy_ = (motor_can1_->GetSpeed(0) - motor_can1_->GetSpeed(1) -
               motor_can1_->GetSpeed(2) + motor_can1_->GetSpeed(3)) *
              r_wheel_ / 4.0f;
    now_omega_ = (motor_can1_->GetSpeed(0) + motor_can1_->GetSpeed(1) +
                  motor_can1_->GetSpeed(2) + motor_can1_->GetSpeed(3)) *
                 r_wheel_ / (4.0f * r_center_);
  }

  /**
   * @brief 麦轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个麦轮的目标角速度
   */
  void InverseKinematicsSolution() {
    target_motor_omega_[0] = (-target_vx_ + target_vy_ + target_omega_);
    target_motor_omega_[1] = (-target_vx_ - target_vy_ + target_omega_);
    target_motor_omega_[2] = (target_vx_ - target_vy_ + target_omega_);
    target_motor_omega_[3] = (target_vx_ + target_vy_ + target_omega_);
  }

  /**
   * @brief 麦轮底盘逆动力学解算
   * @details
   * 通过运动学正解算出底盘现在的运动状态，并与目标状态进行PID控制，获得目标前馈力矩
   */
  void DynamicInverseSolution() {
    float force_x = pid_velocity_x_.Calculate(
        target_vx_ * MOTOR_MAX_ROTATIONAL_SPEED, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(
        target_vy_ * MOTOR_MAX_ROTATIONAL_SPEED, now_vy_, dt_);
    float torque = pid_omega_.Calculate(target_omega_ * MOTOR_MAX_ROTATIONAL_SPEED,
                                        now_omega_, dt_);

    target_motor_force_[0] = (-force_x / 4 * r_wheel_ + force_y / 4 * r_wheel_ +
                              torque * r_center_ / 4 / r_wheel_);
    target_motor_force_[1] = (-force_x / 4 * r_wheel_ - force_y / 4 * r_wheel_ +
                              torque * r_center_ / 4 / r_wheel_);
    target_motor_force_[2] = (force_x / 4 * r_wheel_ - force_y / 4 * r_wheel_ +
                              torque * r_center_ / 4 / r_wheel_);
    target_motor_force_[3] = (force_x / 4 * r_wheel_ + force_y / 4 * r_wheel_ +
                              torque * r_center_ / 4 / r_wheel_);
  }

  /**
   * @brief 麦轮底盘动力学输出
   * @details
   * 限幅并输出四个麦轮的电流控制指令
   */

  void OutputToDynamics() {
    target_motor_current_[0] = pid_wheel_omega_[0].Calculate(
        target_motor_omega_[0] * MOTOR_MAX_ROTATIONAL_SPEED,
        motor_can1_->GetSpeed(0), dt_);
    target_motor_current_[1] = pid_wheel_omega_[1].Calculate(
        target_motor_omega_[1] * MOTOR_MAX_ROTATIONAL_SPEED,
        motor_can1_->GetSpeed(1), dt_);
    target_motor_current_[2] = pid_wheel_omega_[2].Calculate(
        target_motor_omega_[2] * MOTOR_MAX_ROTATIONAL_SPEED,
        motor_can1_->GetSpeed(2), dt_);
    target_motor_current_[3] = pid_wheel_omega_[3].Calculate(
        target_motor_omega_[3] * MOTOR_MAX_ROTATIONAL_SPEED,
        motor_can1_->GetSpeed(3), dt_);

    for (int i = 0; i < 4; i++) {
      output_[i] = std::clamp(target_motor_current_[i] + target_motor_force_[i],
                              -max_current_, max_current_);
    }

    motor_can1_->SetCurrent(0, output_[0]);
    motor_can1_->SetCurrent(1, output_[1]);
    motor_can1_->SetCurrent(2, output_[2]);
    motor_can1_->SetCurrent(3, output_[3]);
  }

 private:
  float r_wheel_ = 0.0f;

  float r_center_ = 0.0f;

  float g_height_ = 0.0f;

  float target_motor_omega_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_force_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_current_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float output_[4]{0.0f, 0.0f, 0.0f, 0.0f};

  /*若轮子出现明显阻力使用标定法获得此参数*/
  float wheel_resistance_ = 0.0f;

  float error_compensation_ = 0.5f;

  float now_vx_ = 0.0f;
  float now_vy_ = 0.0f;
  float now_omega_ = 0.0f;

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;

  float max_current_ = 1.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor<MotorType> *motor_can1_;
  Motor<MotorType> *motor_can2_;
  CMD *cmd_;

  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_omega_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::PID<float> pid_steer_angle_[4]{0.0f, 0.0f, 0.0f, 0.0f};

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  CMD::ChassisCMD cmd_data_;
  uint32_t chassis_event_ = 0;
};
