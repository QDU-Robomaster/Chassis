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
#include <cstdint>

#include "CMD.hpp"
#include "Motor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"

template <typename ChassisType, typename MotorType>
class Chassis;

template <typename MotorType>
class Omni {
 public:
  Omni(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
       Motor<MotorType> &motor_can1, Motor<MotorType> &motor_can2,
       uint32_t task_stack_depth, float wheel_radius, float wheel_to_center,
       float gravity_height, float wheel_resistance, float error_compensation,
       LibXR::PID<float>::Param pid_velocity_x,
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
        pid_wheel_omega_{pid_wheel_omega_0, pid_wheel_omega_1,
                         pid_wheel_omega_2, pid_wheel_omega_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3} {
    UNUSED(hw);
    UNUSED(app);

    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  static void ThreadFunction(Omni *omni) {
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");

    cmd_suber.StartWaiting();
    while (true) {
      if (cmd_suber.Available()) {
        omni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      omni->mutex_.Lock();
      omni->Update();
      omni->UpdateSetpointFromCMD();
      omni->SelfResolution();
      omni->KinematicsInverseResolution();
      omni->mutex_.Unlock();
      omni->OutputToDynamics();

      omni->thread_.SleepUntil(omni->last_online_time_, 2.0f);
    }
  }

  /**
   * @brief 更新电机状态
   * @details 更新所有轮向电机的反馈数据
   */
  void Update() {
    auto now = LibXR::Timebase::GetMilliseconds();
    this->dt_ = (now - this->last_online_time_).ToSecondf();
    this->last_online_time_ = now;
    for (int i = 0; i < 4; i++) {
      motor_can1_->Update(i);
    }
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
   * @brief 全向轮底盘正运动学解算
   * @details 根据四个全向轮的角速度，解算出底盘当前的运动状态
   *
   *          全向轮运动学模型：
   *          - 轮组方向向量 n = (cosθ, sinθ)，其中 θ 为轮组方位角
   *          - 全向轮滚子方向与轮组方向一致（不同于麦轮）
   *
   *          从电机角速度反推底盘速度：
   *          Wheel_Omega = ((vx - ω*ry)*cosθ + (vy + ω*rx)*sinθ) / s
   *
   *          其中 s 为轮子半径
   *
   *          通过四个轮子的速度建立超定方程组求解 vx, vy, ω
   */
  void SelfResolution() {
    now_vx_ = 0.0f;
    now_vy_ = 0.0f;
    now_omega_ = 0.0f;

    for (int i = 0; i < 4; i++) {
      float wheel_velocity = motor_can1_->GetSpeed(i) * wheel_radius_;
      float theta = wheel_azimuth_[i];
      float cos_theta = std::cos(theta);
      float sin_theta = std::sin(theta);
      float r = wheel_to_center_;

      now_vx_ += wheel_velocity * cos_theta / 4.0f;
      now_vy_ += wheel_velocity * sin_theta / 4.0f;
      now_omega_ += (-wheel_velocity / r) / 4.0f;
    }
  }

  /**
   * @brief 全向轮底盘逆运动学解算
   * @details 根据目标底盘速度(vx, vy, omega)，计算每个轮子的目标角速度
   *
   *          全向轮逆运动学公式：
   *          Wheel_Omega = ((vx - ω*ry)*cosθ + (vy + ω*rx)*sinθ) / wheel_radius
   *
   *          其中：
   *          - (rx, ry) 是轮子相对底盘中心的位置（极坐标转直角坐标）
   *          - θ 是轮组方位角
   *          - 全向轮不需要投影补偿（不同于麦轮的cos(45°)）
   */
  void KinematicsInverseResolution() {
    for (int i = 0; i < 4; i++) {
      float r = wheel_to_center_;
      float theta = wheel_azimuth_[i];
      float cos_theta = std::cos(theta);
      float sin_theta = std::sin(theta);

      float rx = r * cos_theta;
      float ry = r * sin_theta;

      float wheel_velocity = (target_vx_ - target_omega_ * ry) * cos_theta +
                             (target_vy_ + target_omega_ * rx) * sin_theta;

      target_wheel_omega_[i] = wheel_velocity / wheel_radius_;
    }
  }

  /**
   * @brief 全向轮底盘动力学控制输出
   * @details 三层控制架构（完全仿照Helm舵轮底盘）:
   *
   *          【第1层】底盘级PID控制:
   *            - pid_velocity_x: X方向速度 → force_x (所需力)
   *            - pid_velocity_y: Y方向速度 → force_y (所需力)
   *            - pid_omega: 角速度 → torque (所需扭矩)
   *
   *          【动力学逆解算】全向轮力分配:
   *            假设轮子半径为 s，底盘半径为 r
   *
   *            Motor0 = (-√2*Fx + √2*Fy + T/r) / 4 * s
   *            Motor1 = (-√2*Fx - √2*Fy + T/r) / 4 * s
   *            Motor2 = (√2*Fx - √2*Fy + T/r) / 4 * s
   *            Motor3 = (√2*Fx + √2*Fy + T/r) / 4 * s
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

    const float SQRT2 = 1.41421356237f;

    float torque_factor = torque / wheel_to_center_;

    float wheel_force[4];
    wheel_force[0] = (-SQRT2 * force_x + SQRT2 * force_y + torque_factor) /
                     4.0f * wheel_radius_;
    wheel_force[1] = (-SQRT2 * force_x - SQRT2 * force_y + torque_factor) /
                     4.0f * wheel_radius_;
    wheel_force[2] = (SQRT2 * force_x - SQRT2 * force_y + torque_factor) /
                     4.0f * wheel_radius_;
    wheel_force[3] = (SQRT2 * force_x + SQRT2 * force_y + torque_factor) /
                     4.0f * wheel_radius_;

    for (int i = 0; i < 4; i++) {
      float current_wheel_omega = motor_can1_->GetSpeed(i);

      target_wheel_current_[i] =
          wheel_force[i] +
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
      float current_wheel_omega = motor_can1_->GetSpeed(i);

      float wheel_current = 0.0f;
      wheel_current = pid_wheel_omega_[i].Calculate(target_wheel_omega_[i], current_wheel_omega, dt_);

      wheel_current += target_wheel_current_[i];

      const float MAX_CURRENT = 1.0f;
      wheel_current = std::clamp(wheel_current, -MAX_CURRENT, MAX_CURRENT);

      motor_can1_->SetCurrent(i, wheel_current);

      while (target_vx_ == 0.0f && target_vy_ == 0.0f &&
             target_omega_ == 0.0f) {
        motor_can1_->SetCurrent(i, 0.0f);
      }
    }
  }

 private:
  float wheel_radius_ = 0.0f;

  float wheel_to_center_ = 0.0f;

  float wheel_azimuth_[4] = {
      M_PI / 4.0f,
      3.0f * M_PI / 4.0f,
      5.0f * M_PI / 4.0f,
      7.0f * M_PI / 4.0f,
  };

  float gravity_height_ = 0.0f;

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

  float dt_ = 0;
  LibXR::MillisecondTimestamp last_online_time_ = 0;

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
