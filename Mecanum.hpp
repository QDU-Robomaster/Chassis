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
#include <cmath>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

template <typename MotorType>
class Mecanum {
 public:
  Mecanum(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          CMD &cmd, Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
          uint32_t task_stack_depth, LibXR::PID<float>::Param pid_velocity_x,
          LibXR::PID<float>::Param pid_velocity_y,
          LibXR::PID<float>::Param pid_omega,
          LibXR::PID<float>::Param pid_wheel_angle_0,
          LibXR::PID<float>::Param pid_wheel_angle_1,
          LibXR::PID<float>::Param pid_wheel_angle_2,
          LibXR::PID<float>::Param pid_wheel_angle_3,
          LibXR::PID<float>::Param pid_steer_angle_0,
          LibXR::PID<float>::Param pid_steer_angle_1,
          LibXR::PID<float>::Param pid_steer_angle_2,
          LibXR::PID<float>::Param pid_steer_angle_3)
      : motor_can1_(&motor_can1),
        motor_can2_(&motor_can2),
        cmd_(&cmd),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_angle_0_(pid_wheel_angle_0),
        pid_wheel_angle_1_(pid_wheel_angle_1),
        pid_wheel_angle_2_(pid_wheel_angle_2),
        pid_wheel_angle_3_(pid_wheel_angle_3),
        pid_steer_angle_0_(pid_steer_angle_0),
        pid_steer_angle_1_(pid_steer_angle_1),
        pid_steer_angle_2_(pid_steer_angle_2),
        pid_steer_angle_3_(pid_steer_angle_3) {
    thread_.Create(this, ThreadFunction, "MecanumChassisThread",
                   task_stack_depth, LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(Mecanum *mecanum) {
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");

    auto now_ = LibXR::Timebase::GetMicroseconds();
    mecanum->dt_ = (now_ - mecanum->last_online_time_);
    mecanum->last_online_time_ = now_;

    cmd_suber.StartWaiting();
    while (true) {
      if (cmd_suber.Available()) {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      mecanum->target_vx_ = mecanum->cmd_data_.x;
      mecanum->target_vy_ = mecanum->cmd_data_.y;
      mecanum->target_omega_ = mecanum->cmd_data_.z;

      mecanum->SelfResolution();
      mecanum->KinematicsInverseResolution();
      mecanum->OutputToDynamics();
    }
  }

  void Update() {
    for (int i = 0; i < 4; i++) {
      motor_can1_->Update(i);
    }
    for (int i = 0; i < 4; i++) {
      motor_can2_->Update(i);
    }
  }

  /**
   * @brief 麦轮底盘正运动学解算
   * @details 根据四个麦轮的角速度，解算出底盘当前的运动状态
   *
   *          麦轮运动学模型：
   *          - 滚子方向 n = (cosθ, sinθ)，其中 θ 为轮组方位角
   *          - 麦轮线速度方向 m = (cosγ, sinγ)，其中 γ = θ - π/4
   *          - n 与 m 之间夹角为 45°
   *
   *          从电机角速度反推底盘速度：
   *          Wheel_Omega = ((vx - ω*ry)*cosθ + (vy + ω*rx)*sinθ) / cos(γ-θ) / s
   *
   *          其中 cos(γ-θ) = cos(π/4) = √2/2
   *
   *          通过四个轮子的速度建立超定方程组求解 vx, vy, ω
   */
  void SelfResolution() {
    const float cos_roller_angle_ = 0.70710678118f;  // cos(45°) = √2/2

    now_vx_ = 0.0f;
    now_vy_ = 0.0f;
    now_omega_ = 0.0f;

    for (int i = 0; i < 4; i++) {
      float v_projected =
          motor_can1_->GetSpeed(i) * wheel_radius_ * cos_roller_angle_;
      float cos_theta = std::cos(wheel_azimuth_[i]);
      float sin_theta = std::sin(wheel_azimuth_[i]);

      now_vx_ += v_projected * cos_theta / 4.0f;
      now_vy_ += v_projected * sin_theta / 4.0f;
      now_omega_ += (-v_projected / wheel_to_center_[i]) / 4.0f;
    }
  }

  /**
   * @brief 麦轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个麦轮的目标角速度
   *
   *          麦轮逆运动学公式：
   *          Wheel_Omega = ((vx - ω*ry)*cosθ + (vy + ω*rx)*sinθ) / cos(45°) /
   * wheel_radius
   *
   *          其中：
   *          - (rx, ry) 是轮子相对底盘中心的位置（极坐标转直角坐标）
   *          - θ 是轮组方位角
   *          - cos(45°) = √2/2 ≈ 0.707
   */
  void KinematicsInverseResolution() {
    const float cos_roller_angle_ = 0.70710678118f;  // cos(45°) = √2/2

    for (int i = 0; i < 4; i++) {
      float r = wheel_to_center_[i];
      float theta = wheel_azimuth_[i];
      float cos_theta = std::cos(theta);
      float sin_theta = std::sin(theta);

      float rx = r * cos_theta;
      float ry = r * sin_theta;

      float wheel_velocity = ((target_vx_ - target_omega_ * ry) * cos_theta +
                              (target_vy_ + target_omega_ * rx) * sin_theta) /
                             cos_roller_angle_;

      target_wheel_omega_[i] = wheel_velocity / wheel_radius_;
    }
  }

  /**
   * @brief 麦轮底盘动力学控制输出
   * @details 三层控制架构（完全仿照Helm舵轮底盘）:
   *
   *          【第1层】底盘级PID控制:
   *            - pid_velocity_x: X方向速度 → force_x (所需力)
   *            - pid_velocity_y: Y方向速度 → force_y (所需力)
   *            - pid_omega: 角速度 → torque (所需扭矩)
   *
   *          【动力学逆解算】麦轮力分配:
   *            假设轮子半径为 s，底盘半长为 a，半宽为 b
   *
   *            Motor0 = (-Fx + Fy + T/(a+b)) / 4 * s
   *            Motor1 = (-Fx - Fy + T/(a+b)) / 4 * s
   *            Motor2 = (Fx - Fy + T/(a+b)) / 4 * s
   *            Motor3 = (Fx + Fy + T/(a+b)) / 4 * s
   *
   *          【第2层】电机角速度PID控制:
   *            - pid_wheel_angle_X: 目标角速度 → 电流输出
   *            - 添加动摩擦阻力前馈补偿
   *
   *          【第3层】电流限幅与输出:
   *            - 限制最大电流，输出到电机
   */
  void OutputToDynamics() {
    float current_vx = now_vx_;
    float current_vy = now_vy_;
    float current_omega = now_omega_;

    float force_x = pid_velocity_x_.Calculate(target_vx_, current_vx, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, current_vy, dt_);
    float torque = pid_omega_.Calculate(target_omega_, current_omega, dt_);

    float half_wheelbase = (wheel_to_center_[0] + wheel_to_center_[1] +
                            wheel_to_center_[2] + wheel_to_center_[3]) /
                           4.0f;

    float torque_factor = torque / half_wheelbase;

    float wheel_force[4];
    wheel_force[0] =
        (-force_x + force_y + torque_factor) / 4.0f * wheel_radius_;  // 左前
    wheel_force[1] =
        (-force_x - force_y + torque_factor) / 4.0f * wheel_radius_;  // 左后
    wheel_force[2] =
        (force_x - force_y + torque_factor) / 4.0f * wheel_radius_;  // 右后
    wheel_force[3] =
        (force_x + force_y + torque_factor) / 4.0f * wheel_radius_;  // 右前

    for (int i = 0; i < 4; i++) {
      float current_wheel_omega = motor_can1_->GetSpeed(i);

      target_wheel_current_[i] =
          wheel_force[i] + wheel_speed_limit_factor_ *
                               (target_wheel_omega_[i] - current_wheel_omega);

      if (target_wheel_omega_[i] > wheel_resistance_omega_threshold_) {
        target_wheel_current_[i] += dynamic_wheel_current_[i];
      } else if (target_wheel_omega_[i] < -wheel_resistance_omega_threshold_) {
        target_wheel_current_[i] -= dynamic_wheel_current_[i];
      } else {
        target_wheel_current_[i] += current_wheel_omega /
                                    wheel_resistance_omega_threshold_ *
                                    dynamic_wheel_current_[i];
      }
    }

    for (int i = 0; i < 4; i++) {
      float current_wheel_omega = motor_can1_->GetSpeed(i);

      float wheel_current = 0.0f;
      switch (i) {
        case 0:
          wheel_current = pid_wheel_angle_0_.Calculate(
              target_wheel_omega_[i], current_wheel_omega, dt_);
          break;
        case 1:
          wheel_current = pid_wheel_angle_1_.Calculate(
              target_wheel_omega_[i], current_wheel_omega, dt_);
          break;
        case 2:
          wheel_current = pid_wheel_angle_2_.Calculate(
              target_wheel_omega_[i], current_wheel_omega, dt_);
          break;
        case 3:
          wheel_current = pid_wheel_angle_3_.Calculate(
              target_wheel_omega_[i], current_wheel_omega, dt_);
          break;
      }

      wheel_current += target_wheel_current_[i];

      const float kMaxCurrent = 1.0f;
      wheel_current = std::clamp(wheel_current, -kMaxCurrent, kMaxCurrent);

      motor_can1_->SetCurrent(i, wheel_current);
    }
  }

 private:
  const float wheel_radius_ = 0.0f;

  const float wheel_to_center_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  const float wheel_azimuth_[4] = {
      M_PI / 4.0f,
      3.0f * M_PI / 4.0f,
      5.0f * M_PI / 4.0f,
      7.0f * M_PI / 4.0f,
  };

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

  LibXR::MicrosecondTimestamp::Duration dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor<MotorType> *motor_can1_;
  Motor<MotorType> *motor_can2_;
  CMD *cmd_;

  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_angle_0_;
  LibXR::PID<float> pid_wheel_angle_1_;
  LibXR::PID<float> pid_wheel_angle_2_;
  LibXR::PID<float> pid_wheel_angle_3_;
  LibXR::PID<float> pid_steer_angle_0_;
  LibXR::PID<float> pid_steer_angle_1_;
  LibXR::PID<float> pid_steer_angle_2_;
  LibXR::PID<float> pid_steer_angle_3_;

  LibXR::Thread thread_;

  CMD::ChassisCMD cmd_data_;
};
