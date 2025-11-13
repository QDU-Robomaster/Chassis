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

#define MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT \
  52 /* 电机输出轴最大角速度 */

template <typename ChassisType, typename MotorType>
class Chassis;

template <typename MotorType>
class Omni {
 public:
  /**
   * @brief 构造函数，初始化全向轮底盘控制对象
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令引用
   * @param motor_wheel_0 第0个驱动轮电机指针
   * @param motor_wheel_1 第1个驱动轮电机指针
   * @param motor_wheel_2 第2个驱动轮电机指针
   * @param motor_wheel_3 第3个驱动轮电机指针
   * @param motor_steer_0 第0个舵向电机指针（本底盘未使用）
   * @param motor_steer_1 第1个舵向电机指针（本底盘未使用）
   * @param motor_steer_2 第2个舵向电机指针（本底盘未使用）
   * @param motor_steer_3 第3个舵向电机指针（本底盘未使用）
   * @param task_stack_depth 控制线程栈深度
   * @param wheel_radius 轮子半径
   * @param wheel_to_center 轮子到底盘中心距离
   * @param gravity_height 重心高度
   * @param wheel_resistance 轮子阻力
   * @param error_compensation 误差补偿系数
   * @param pid_velocity_x X方向速度PID参数
   * @param pid_velocity_y Y方向速度PID参数
   * @param pid_omega 角速度PID参数
   * @param pid_wheel_omega_0 轮子0角速度PID参数
   * @param pid_wheel_omega_1 轮子1角速度PID参数
   * @param pid_wheel_omega_2 轮子2角速度PID参数
   * @param pid_wheel_omega_3 轮子3角速度PID参数
   * @param pid_steer_angle_0 舵机0角度PID参数（本底盘未使用）
   * @param pid_steer_angle_1 舵机1角度PID参数（本底盘未使用）
   * @param pid_steer_angle_2 舵机2角度PID参数（本底盘未使用）
   * @param pid_steer_angle_3 舵机3角度PID参数（本底盘未使用）
   */
  Omni(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app, CMD &cmd,
       typename MotorType::RMMotor *motor_wheel_0,
       typename MotorType::RMMotor *motor_wheel_1,
       typename MotorType::RMMotor *motor_wheel_2,
       typename MotorType::RMMotor *motor_wheel_3,
       typename MotorType::RMMotor *motor_steer_0,
       typename MotorType::RMMotor *motor_steer_1,
       typename MotorType::RMMotor *motor_steer_2,
       typename MotorType::RMMotor *motor_steer_3, uint32_t task_stack_depth,
       float wheel_radius, float wheel_to_center, float gravity_height,
       float reductionratio, float wheel_resistance, float error_compensation,
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
      : r_wheel_(wheel_radius),
        r_center_(wheel_to_center),
        g_height_(gravity_height),
        reductionratio_(reductionratio),
        wheel_resistance_(wheel_resistance),
        error_compensation_(error_compensation),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
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
    UNUSED(motor_steer_0);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
  }

  /**
   * @brief 全向轮底盘控制线程函数
   * @param omni Omni对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
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
      omni->InverseKinematicsSolution();
      omni->DynamicInverseSolution();
      omni->mutex_.Unlock();
      omni->OutputToDynamics();

      auto last_time = LibXR::Timebase::GetMilliseconds();
      omni->thread_.SleepUntil(last_time, 2.0f);
    }
  }

  /**
   * @brief 更新电机状态
   * @details 获取当前时间戳并更新所有驱动轮电机的状态
   */
  void Update() {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    motor_wheel_0_->Update();
    motor_wheel_1_->Update();
    motor_wheel_2_->Update();
    motor_wheel_3_->Update();
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
   * @details 从CMD获取底盘控制指令，并转换为目标速度
   */
  void UpdateSetpointFromCMD() {
    const float SQRT2 = 1.41421356237f;
    target_vx_ = cmd_data_.y * MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT *
                 r_wheel_ * SQRT2;
    target_vy_ = cmd_data_.x * MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT *
                 r_wheel_ * SQRT2;
    target_omega_ = -cmd_data_.z * MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT *
                    r_center_ * SQRT2;
  }

  /**
   * @brief 全向轮底盘正运动学解算
   * @details 根据四个全向轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    const float SQRT2 = 1.41421356237f;
    now_vx_ = (-motor_wheel_0_->GetOmega() - motor_wheel_1_->GetOmega() +
               motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
              SQRT2 * r_wheel_ / 4.0f;
    now_vy_ = (motor_wheel_0_->GetOmega() - motor_wheel_1_->GetOmega() -
               motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
              SQRT2 * r_wheel_ / 4.0f;
    now_omega_ = (motor_wheel_0_->GetOmega() + motor_wheel_1_->GetOmega() +
                  motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
                 SQRT2 * r_wheel_ / (4.0f * r_center_);
  }

  /**
   * @brief 全向轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个全向轮的目标角速度
   */
  void InverseKinematicsSolution() {
    const float SQRT1 = 0.70710678118f;

    target_motor_omega_[0] =
        (-SQRT1 * now_vx_ + SQRT1 * now_vx_ + now_omega_ * r_center_) /
        r_wheel_;
    target_motor_omega_[1] =
        (-SQRT1 * target_vx_ - SQRT1 * target_vy_ + now_omega_ * r_center_) /
        r_wheel_;
    target_motor_omega_[2] =
        (SQRT1 * target_vx_ - SQRT1 * target_vy_ + now_omega_ * r_center_) /
        r_wheel_;
    target_motor_omega_[3] =
        (SQRT1 * target_vx_ + SQRT1 * target_vy_ + now_omega_ * r_center_) /
        r_wheel_;
  }

  /**
   * @brief 全向轮底盘逆动力学解算
   * @details
   * 通过运动学正解算出底盘现在的运动状态，并与目标状态进行PID控制，获得目标前馈力矩
   */
  void DynamicInverseSolution() {
    const float SQRT2 = 1.41421356237f;

    float force_x = pid_velocity_x_.Calculate(target_vx_, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, now_vy_, dt_);
    float torque = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    target_motor_force_[0] =
        (-SQRT2 * force_x / 4 / r_wheel_ + SQRT2 * force_y / 4 / r_wheel_ +
         torque * r_center_ / 4 / r_wheel_);
    target_motor_force_[1] =
        (-SQRT2 * force_x / 4 / r_wheel_ - SQRT2 * force_y / 4 / r_wheel_ +
         torque * r_center_ / 4 / r_wheel_);
    target_motor_force_[2] =
        (SQRT2 * force_x / 4 / r_wheel_ - SQRT2 * force_y / 4 / r_wheel_ +
         torque * r_center_ / 4 / r_wheel_);
    target_motor_force_[3] =
        (SQRT2 * force_x / 4 / r_wheel_ + SQRT2 * force_y / 4 / r_wheel_ +
         torque * r_center_ / 4 / r_wheel_);
  }

  /**
   * @brief 全向轮底盘动力学输出
   * @details 限幅并输出四个全向轮的电流控制指令
   */
  void OutputToDynamics() {
    target_motor_current_[0] = pid_wheel_omega_[0].Calculate(
        target_motor_omega_[0], motor_wheel_0_->GetOmega(), dt_);
    target_motor_current_[1] = pid_wheel_omega_[1].Calculate(
        target_motor_omega_[1], motor_wheel_1_->GetOmega(), dt_);
    target_motor_current_[2] = pid_wheel_omega_[2].Calculate(
        target_motor_omega_[2], motor_wheel_2_->GetOmega(), dt_);
    target_motor_current_[3] = pid_wheel_omega_[3].Calculate(
        target_motor_omega_[3], motor_wheel_3_->GetOmega(), dt_);

    for (int i = 0; i < 4; i++) {
      output_[i] = (target_motor_current_[i] + target_motor_force_[i]*r_wheel_);
    }

    motor_wheel_0_->TorqueControl(output_[0], reductionratio_);
    motor_wheel_1_->TorqueControl(output_[1], reductionratio_);
    motor_wheel_2_->TorqueControl(output_[2], reductionratio_);
    motor_wheel_3_->TorqueControl(output_[3], reductionratio_);
  }

 private:
  float r_wheel_ = 0.0f;

  float r_center_ = 0.0f;

  float g_height_ = 0.0f;

  float reductionratio_ = 0.0f;

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

  typename MotorType::RMMotor *motor_wheel_0_;
  typename MotorType::RMMotor *motor_wheel_1_;
  typename MotorType::RMMotor *motor_wheel_2_;
  typename MotorType::RMMotor *motor_wheel_3_;
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
