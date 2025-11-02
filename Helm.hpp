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
#include "pid.hpp"

/**
 * @brief 舵轮底盘控制类
 * @tparam MotorType 电机容器类型
 */
template <typename MotorType>
class Helm {
 public:
  /**
   * @brief 构造函数
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令引用
   * @param motor_can1 轮向电机容器（驱动轮旋转）
   * @param motor_can2 舵向电机容器（控制轮子转向）
   * @param task_stack_depth 线程栈深度
   * @param pid_velocity_x 底盘X方向速度PID参数
   * @param pid_velocity_y 底盘Y方向速度PID参数
   * @param pid_omega 底盘角速度PID参数
   * @param pid_wheel_angle_0 轮向电机0角速度PID参数
   * @param pid_wheel_angle_1 轮向电机1角速度PID参数
   * @param pid_wheel_angle_2 轮向电机2角速度PID参数
   * @param pid_wheel_angle_3 轮向电机3角速度PID参数
   * @param pid_steer_angle_0 舵向电机0角度PID参数
   * @param pid_steer_angle_1 舵向电机1角度PID参数
   * @param pid_steer_angle_2 舵向电机2角度PID参数
   * @param pid_steer_angle_3 舵向电机3角度PID参数
   */
  Helm(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
       Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
       uint32_t task_stack_depth, float wheel_radius, float wheel_to_center,
       float gravity_height, float wheel_resistance, float error_compensation,
       LibXR::PID<float>::Param pid_velocity_x,
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
      : wheel_radius_(wheel_radius),
        wheel_to_center_(wheel_to_center),
        gravity_height_(gravity_height),
        wheel_resistance_(wheel_resistance),
        error_compensation_(error_compensation),
        motor_can1_(&motor_can1),
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
    thread_.Create(this, ThreadFunction, "HelmChassisThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 底盘控制线程函数
   * @param helm Helm 对象指针
   * @details 循环执行底盘控制流程：
   *          1. 接收控制命令
   *          2. 正运动学解算（获取当前状态）
   *          3. 逆运动学解算（计算目标值）
   *          4. 舵向最近转置优化
   *          5. 动力学控制输出
   */
  static void ThreadFunction(Helm *helm) {
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");

    cmd_suber.StartWaiting();
    while (true) {
      if (cmd_suber.Available()) {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      helm->target_vx_ = helm->cmd_data_.x;
      helm->target_vy_ = helm->cmd_data_.y;
      helm->target_omega_ = helm->cmd_data_.z;

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateSetpointFromCMD();
      helm->SelfResolution();
      helm->KinematicsInverseResolution();
      helm->mutex_.Unlock();
      helm->OutputToDynamics();

      helm->thread_.SleepUntil(helm->last_online_time_, 2.0f);
    }
  }

  /**
   * @brief 更新电机状态
   * @details 更新所有轮向和舵向电机的反馈数据
   */
  void Update() {
    auto now_ = LibXR::Timebase::GetMilliseconds();
    this->dt_ = (now_ - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now_;

    for (int i = 0; i < 4; i++) {
      motor_can1_->Update(i);
    }
    for (int i = 0; i < 4; i++) {
      motor_can2_->Update(i);
    }
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
   * @brief 底盘正运动学解算
   * @details 根据各轮子的速度和舵向角度，解算出底盘当前的:
   *          - X方向速度 (now_vx_)
   *          - Y方向速度 (now_vy_)
   *          - 旋转角速度 (now_omega_)
   */
  void SelfResolution() {
    now_vx_ = 0.0f;
    now_vy_ = 0.0f;
    now_omega_ = 0.0f;

    for (int i = 0; i < 4; i++) {
      now_vx_ += (motor_can1_->GetSpeed(i) *
                  std::cos(motor_can1_->GetSpeed(i)) * wheel_radius_) /
                 4.0f;
      now_vy_ += (motor_can1_->GetSpeed(i) *
                  std::cos(motor_can1_->GetSpeed(i)) * wheel_radius_) /
                 4.0f;
      now_omega_ += (motor_can2_->GetAngle(i) *
                     std::sin(motor_can2_->GetAngle(i) - wheel_azimuth_[i]) *
                     wheel_radius_ / wheel_to_center_) /
                    4.0f;
    }
  }

  /**
   * @brief 底盘逆运动学解算
   * @details 根据目标底盘速度(vx, vy, omega)，计算每个轮子的:
   *          - 目标角速度 (target_wheel_omega_)
   *          - 目标舵向角度 (target_steer_angle_)
   *
   *          计算公式:
   *          - tmp_vx = target_vx - target_omega * R * sin(θ)
   *          - tmp_vy = target_vy + target_omega * R * cos(θ)
   *          - target_wheel_omega = sqrt(vx² + vy²) / wheel_radius
   *          - target_steer_angle = atan2(vy, vx)
   */
  void KinematicsInverseResolution() {
    float tmp_vx, tmp_vy, tmp_velocity_modulus;
    for (int i = 0; i < 4; ++i) {
      tmp_vx = target_vx_ -
               target_omega_ * wheel_to_center_ * std::sin(wheel_azimuth_[i]);
      tmp_vy = target_vy_ +
               target_omega_ * wheel_to_center_ * std::cos(wheel_azimuth_[i]);
      tmp_velocity_modulus =
          std::sqrt(tmp_vx * tmp_vx + tmp_vy * tmp_vy) / wheel_radius_;

      target_wheel_omega_[i] = tmp_velocity_modulus / wheel_radius_;

      if (tmp_velocity_modulus == 0.0f) {
        target_steer_angle_[i] = motor_can2_->GetAngle(i);
      } else {
        target_steer_angle_[i] = std::atan2(tmp_vy, tmp_vx);
      }
    }

    SteerMotorKinematicsNearestTransposition();
  }

  /**
   * @brief 舵向电机最近转置优化
   * @details 优化舵向电机的转动策略:
   *          - 如果目标角度与当前角度差值 ∈ [-90°, 90°]，直接转向
   *          - 如果差值 > 90°，则反向转动并将轮速反向
   *
   *          目的: 减少舵向电机转动幅度，提高响应速度
   */
  void SteerMotorKinematicsNearestTransposition() {
    for (int i = 0; i < 4; i++) {
      float tmp_delta_angle = Math_Modulus_Normalization(
          target_steer_angle_[i] - motor_can2_->GetAngle(i), 2.0f * M_PI);

      if (-M_PI / 2.0f <= tmp_delta_angle && tmp_delta_angle <= M_PI / 2.0f) {
        target_steer_angle_[i] = tmp_delta_angle + motor_can2_->GetAngle(i);
      } else {
        target_steer_angle_[i] =
            Math_Modulus_Normalization(tmp_delta_angle + M_PI, 2.0f * M_PI) +
            motor_can2_->GetAngle(i);
        target_wheel_omega_[i] *= -1.0f;
      }
    }
  }

  /**
   * @brief 模数归一化函数
   * @param x 输入值
   * @param modulus 模数（周期）
   * @return 归一化后的值，范围 [-modulus/2, modulus/2]
   * @details 将角度归一化到 [-π, π] 或其他周期范围内
   */
  float Math_Modulus_Normalization(float x, float modulus) {
    float tmp;

    tmp = fmod(x + modulus / 2.0f, modulus);

    if (tmp < 0.0f) {
      tmp += modulus;
    }

    return (tmp - modulus / 2.0f);
  }

  /**
   * @brief 动力学控制输出函数
   * @details 三层控制架构:
   *
   *          【第1层】底盘级PID控制:
   *            - pid_velocity_x: X方向速度 → force_x (所需力)
   *            - pid_velocity_y: Y方向速度 → force_y (所需力)
   *            - pid_omega: 角速度 → torque_omega (所需扭矩)
   *
   *          【动力学逆解算】:
   *            - 将底盘所需力和扭矩分配到各轮子
   *            - 计算每个轮子的摩擦力和目标电流
   *            - 添加动摩擦阻力前馈补偿
   *
   *          【第2层】电机级PID控制:
   *            - 舵向电机: 角度PID → 角速度PID → 电流输出
   *            - 轮向电机: 直接电流控制（来自动力学逆解算）
   */
  void OutputToDynamics() {
    // ========== 第1层：底盘级PID控制 ==========
    float current_vx = now_vx_;
    float current_vy = now_vy_;
    float current_omega = now_omega_;

    // 底盘速度PID计算
    float force_x = pid_velocity_x_.Calculate(target_vx_, current_vx, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, current_vy, dt_);
    float torque_omega =
        pid_omega_.Calculate(target_omega_, current_omega, dt_);

    // ========== 动力学逆解算 ==========
    // 计算每个轮组的摩擦力
    float tmp_force[4];
    for (int i = 0; i < 4; i++) {
      float current_steer_angle = motor_can2_->GetAngle(i);

      tmp_force[i] = force_x * std::cos(current_steer_angle) +
                     force_y * std::sin(current_steer_angle) -
                     torque_omega / wheel_to_center_ *
                         std::sin(wheel_azimuth_[i] - current_steer_angle);
    }

    for (int i = 0; i < 4; i++) {
      float current_wheel_omega = motor_can1_->GetSpeed(i);

      target_wheel_current_[i] =
          tmp_force[i] * wheel_radius_ +
          error_compensation_ * (target_wheel_omega_[i] - current_wheel_omega);

      if (target_wheel_omega_[i] > wheel_resistance_) {
        target_wheel_current_[i] += dynamic_wheel_current_[i];
      } else if (target_wheel_omega_[i] < -wheel_resistance_) {
        target_wheel_current_[i] -= dynamic_wheel_current_[i];
      } else {
        target_wheel_current_[i] +=
            current_wheel_omega / wheel_resistance_ * dynamic_wheel_current_[i];
      }
    }

    for (int i = 0; i < 4; i++) {
      float current_steer_angle = motor_can2_->GetAngle(i);

      float steer_angle_output = 0.0f;
      switch (i) {
        case 0:
          steer_angle_output = pid_steer_angle_0_.Calculate(
              target_steer_angle_[i], current_steer_angle, dt_);
          break;
        case 1:
          steer_angle_output = pid_steer_angle_1_.Calculate(
              target_steer_angle_[i], current_steer_angle, dt_);
          break;
        case 2:
          steer_angle_output = pid_steer_angle_2_.Calculate(
              target_steer_angle_[i], current_steer_angle, dt_);
          break;
        case 3:
          steer_angle_output = pid_steer_angle_3_.Calculate(
              target_steer_angle_[i], current_steer_angle, dt_);
          break;
      }

      float target_steer_omega = steer_angle_output;
      float current_steer_omega = motor_can2_->GetSpeed(i);

      float steer_current = 0.0f;
      switch (i) {
        case 0:
          steer_current = pid_wheel_angle_0_.Calculate(
              target_steer_omega, current_steer_omega, dt_);
          break;
        case 1:
          steer_current = pid_wheel_angle_1_.Calculate(
              target_steer_omega, current_steer_omega, dt_);
          break;
        case 2:
          steer_current = pid_wheel_angle_2_.Calculate(
              target_steer_omega, current_steer_omega, dt_);
          break;
        case 3:
          steer_current = pid_wheel_angle_3_.Calculate(
              target_steer_omega, current_steer_omega, dt_);
          break;
      }

      motor_can2_->SetCurrent(i, steer_current);

      const float kMaxCurrent = 1.0f;
      float wheel_current =
          std::clamp(target_wheel_current_[i], -kMaxCurrent, kMaxCurrent);

      motor_can1_->SetCurrent(i, wheel_current);

      while (target_vx_ == 0.0f && target_vy_ == 0.0f &&
             target_omega_ == 0.0f) {
        motor_can2_->SetCurrent(i, 0.0f);
        motor_can1_->SetCurrent(i, 0.0f);
      }
    }
  }

 private:
  const float wheel_radius_ = 0.0f;

  const float wheel_to_center_ = 0.0f;

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

  float target_steer_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /*若轮子出现明显阻力使用标定法获得此参数*/
  float wheel_resistance_ = 0.0f;

  float error_compensation_ = 0.5f;

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

  float  dt_ = 0;
  LibXR::MillisecondTimestamp last_online_time_ = 0;

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
  LibXR::Mutex mutex_;

  CMD::ChassisCMD cmd_data_;
};
