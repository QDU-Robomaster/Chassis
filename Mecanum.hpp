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
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"

#define MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT \
  52 /* 电机输出轴最大角速度 */

template <typename ChassisType>
class Chassis;
class Mecanum {
 public:
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reductionratio = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
  };

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
   * @param chassis_param 全向轮底盘参数
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
  Mecanum(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
          RMMotor *motor_wheel_0, RMMotor *motor_wheel_1,
          RMMotor *motor_wheel_2, RMMotor *motor_wheel_3,
          RMMotor *motor_steer_0, RMMotor *motor_steer_1,
          RMMotor *motor_steer_2, RMMotor *motor_steer_3, CMD *cmd,
          uint32_t task_stack_depth, ChassisParam chassis_param,
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
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_omega_{pid_wheel_omega_0, pid_wheel_omega_1,
                         pid_wheel_omega_2, pid_wheel_omega_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3},
        cmd_(cmd) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(motor_steer_0);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    thread_.Create(this, ThreadFunction, "MecanumChassisThread",
                   task_stack_depth, LibXR::Thread::Priority::MEDIUM);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum *mecanum, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 全向轮底盘控制线程函数
   * @param omni Omni对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Mecanum *mecanum) {
    mecanum->mutex_.Lock();
    auto last_time = LibXR::Timebase::GetMilliseconds();
    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    cmd_suber.StartWaiting();

    mecanum->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      mecanum->mutex_.Lock();
      mecanum->Update();
      mecanum->UpdateSetpointFromCMD();
      // TODO:添加FOLLOW和ROTOR模式
      mecanum->SelfResolution();
      mecanum->InverseKinematicsSolution();
      mecanum->DynamicInverseSolution();
      mecanum->mutex_.Unlock();
      mecanum->OutputToDynamics();

      mecanum->thread_.SleepUntil(last_time, 2);
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
    target_vx_ = cmd_data_.y * MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT *
                 this->PARAM.wheel_radius;
    target_vy_ = cmd_data_.x * MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT *
                 this->PARAM.wheel_radius;
    target_omega_ = -cmd_data_.z * MAXIMUM_ANGULAR_SPEED_OF_MOTOR_OUTPUT_SHAFT *
                    this->PARAM.wheel_to_center;
  }

  /**
   * @brief 全向轮底盘正运动学解算
   * @details 根据四个全向轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    now_vx_ = (-motor_wheel_0_->GetOmega() - motor_wheel_1_->GetOmega() +
               motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
              PARAM.wheel_radius / 4.0f;
    now_vy_ = (motor_wheel_0_->GetOmega() - motor_wheel_1_->GetOmega() -
               motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
              PARAM.wheel_radius / 4.0f;
    now_omega_ = (motor_wheel_0_->GetOmega() + motor_wheel_1_->GetOmega() +
                  motor_wheel_2_->GetOmega() + motor_wheel_3_->GetOmega()) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 全向轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个全向轮的目标角速度
   */
  void InverseKinematicsSolution() {
    target_motor_omega_[0] =
        (-now_vx_ + now_vx_ + now_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[1] =
        (-target_vx_ - target_vy_ + now_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[2] =
        (target_vx_ - target_vy_ + now_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[3] =
        (target_vx_ + target_vy_ + now_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
  }

  /**
   * @brief 全向轮底盘逆动力学解算
   * @details
   * 通过运动学正解算出底盘现在的运动状态，并与目标状态进行PID控制，获得目标前馈力矩
   */
  void DynamicInverseSolution() {
    // TODO:添加功率控制和打滑检测
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

  /**
   * @brief 全向轮底盘动力学输出
   * @details 限幅并输出四个全向轮的电流控制指令
   */
  void OutputToDynamics() {
    // TODO:判断电机返回值是否正常
    target_motor_current_[0] = pid_wheel_omega_[0].Calculate(
        target_motor_omega_[0], motor_wheel_0_->GetOmega(), dt_);
    target_motor_current_[1] = pid_wheel_omega_[1].Calculate(
        target_motor_omega_[1], motor_wheel_1_->GetOmega(), dt_);
    target_motor_current_[2] = pid_wheel_omega_[2].Calculate(
        target_motor_omega_[2], motor_wheel_2_->GetOmega(), dt_);
    target_motor_current_[3] = pid_wheel_omega_[3].Calculate(
        target_motor_omega_[3], motor_wheel_3_->GetOmega(), dt_);

    for (int i = 0; i < 4; i++) {
      output_[i] = (target_motor_current_[i] +
                    target_motor_force_[i] * PARAM.wheel_radius);
    }

    motor_wheel_0_->TorqueControl(output_[0], PARAM.reductionratio);
    motor_wheel_1_->TorqueControl(output_[1], PARAM.reductionratio);
    motor_wheel_2_->TorqueControl(output_[2], PARAM.reductionratio);
    motor_wheel_3_->TorqueControl(output_[3], PARAM.reductionratio);
  }

  void LostCtrl() {
    motor_wheel_0_->Relax();
    motor_wheel_1_->Relax();
    motor_wheel_2_->Relax();
    motor_wheel_3_->Relax();
  }

 private:
  const ChassisParam PARAM;

  float target_motor_omega_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_force_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float target_motor_current_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float output_[4]{0.0f, 0.0f, 0.0f, 0.0f};

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

  LibXR::PID<float> pid_wheel_omega_[4] = {
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
  LibXR::Mutex mutex_;

  CMD::ChassisCMD cmd_data_;
  uint32_t chassis_event_ = 0;
};
