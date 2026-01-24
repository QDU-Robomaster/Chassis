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
#include "Chassis.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"

#define MECANUM_MOTOR_MAX_OMEGA 50 /* 电机输出轴最大角速度 */

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

  enum class Chassismode : uint8_t {
    RELAX,
    ROTOR,
    FOLLOW_GIMBAL_INTERSECT,
    FOLLOW_GIMBAL_CROSS,
    INDEPENDENT,
  };
  /**
   * @brief 构造函数，初始化麦轮底盘控制对象
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
   * @param chassis_param 麦轮底盘参数
   * @param pid_follow 跟随控制PID参数
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
          PowerControl *power_control,uint32_t task_stack_depth,
          ChassisParam chassis_param,
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
          LibXR::PID<float>::Param pid_steer_angle_3,
          LibXR::PID<float>::Param pid_steer_speed_0,
          LibXR::PID<float>::Param pid_steer_speed_1,
          LibXR::PID<float>::Param pid_steer_speed_2,
          LibXR::PID<float>::Param pid_steer_speed_3)
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
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1,
                         pid_steer_speed_2, pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(motor_steer_0);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    UNUSED(pid_steer_speed_0);
    UNUSED(pid_steer_speed_1);
    UNUSED(pid_steer_speed_2);
    UNUSED(pid_steer_speed_3);
    UNUSED(pid_steer_angle_0);
    UNUSED(pid_steer_angle_1);
    UNUSED(pid_steer_angle_2);
    UNUSED(pid_steer_angle_3);
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
   * @brief 麦轮底盘控制线程函数
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
      mecanum->UpdateCMD();
      mecanum->SelfResolution();
      mecanum->InverseKinematicsSolution();
      mecanum->DynamicInverseSolution();
      mecanum->CalculateMotorCurrent();
      mecanum->PowerControlUpdate();
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
   * @brief 设置底盘模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = mode;
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    pid_wheel_omega_[0].Reset();
    pid_wheel_omega_[1].Reset();
    pid_wheel_omega_[2].Reset();
    pid_wheel_omega_[3].Reset();
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CMD获取底盘控制指令，并转换为目标速度
   */
  void UpdateCMD() {
    const float SQRT2 = 1.41421356237f;

    switch (chassis_event_) {
      case static_cast<uint32_t>(Chassismode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        target_omega_ = 0.0f;
        break;

      case static_cast<uint32_t>(Chassismode::ROTOR):
      case static_cast<uint32_t>(Chassismode::FOLLOW_GIMBAL_INTERSECT):
      case static_cast<uint32_t>(Chassismode::FOLLOW_GIMBAL_CROSS):
        break;
      case static_cast<uint32_t>(Chassismode::INDEPENDENT):
        target_vx_ = cmd_data_.x * MECANUM_MOTOR_MAX_OMEGA * PARAM.wheel_radius * SQRT2;
        target_vy_ = cmd_data_.y * MECANUM_MOTOR_MAX_OMEGA * PARAM.wheel_radius * SQRT2;
        target_omega_ = -cmd_data_.z * MECANUM_MOTOR_MAX_OMEGA * PARAM.wheel_to_center * SQRT2;
        break;
      default:
        break;
    }
  }

  /**
   * @brief 麦轮底盘正运动学解算
   * @details 根据四个麦轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    now_vx_ = (-motor_wheel_0_->GetOmega() / PARAM.reductionratio -
               motor_wheel_1_->GetOmega() / PARAM.reductionratio +
               motor_wheel_2_->GetOmega() / PARAM.reductionratio +
               motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
              PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio -
               motor_wheel_1_->GetOmega() / PARAM.reductionratio -
               motor_wheel_2_->GetOmega() / PARAM.reductionratio +
               motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
              PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_1_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_2_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 麦轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个麦轮的目标角速度
   */
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

  /**
   * @brief 麦轮底盘逆动力学解算
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
   * @brief 计算 PID 输出电流
   */
  void CalculateMotorCurrent() {
    target_motor_current_[0] = pid_wheel_omega_[0].Calculate(
        target_motor_omega_[0],
        motor_wheel_0_->GetOmega() / PARAM.reductionratio, dt_);
    target_motor_current_[1] = pid_wheel_omega_[1].Calculate(
        target_motor_omega_[1],
        motor_wheel_1_->GetOmega() / PARAM.reductionratio, dt_);
    target_motor_current_[2] = pid_wheel_omega_[2].Calculate(
        target_motor_omega_[2],
        motor_wheel_2_->GetOmega() / PARAM.reductionratio, dt_);
    target_motor_current_[3] = pid_wheel_omega_[3].Calculate(
        target_motor_omega_[3],
        motor_wheel_3_->GetOmega() / PARAM.reductionratio, dt_);

    /* 计算原始输出 (PID + 前馈力矩) */
    for (int i = 0; i < 4; i++) {
      output_[i] = target_motor_current_[i] +
                   target_motor_force_[i] * PARAM.wheel_radius;
    }
  }

  /**
   * @brief 功率控制更新
   */
  void PowerControlUpdate() {
    /* 获取电机转速 */
    motor_data_.rotorspeed_rpm_3508[0] = motor_wheel_0_->GetRPM();
    motor_data_.rotorspeed_rpm_3508[1] = motor_wheel_1_->GetRPM();
    motor_data_.rotorspeed_rpm_3508[2] = motor_wheel_2_->GetRPM();
    motor_data_.rotorspeed_rpm_3508[3] = motor_wheel_3_->GetRPM();

    /* 获取当前轮速 */
    motor_data_.current_motor_omega_3508[0] =
        motor_wheel_0_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_3508[1] =
        motor_wheel_1_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_3508[2] =
        motor_wheel_2_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_3508[3] =
        motor_wheel_3_->GetOmega() / PARAM.reductionratio;

    /* 设置目标角速度和输出电流 */
    for (int i = 0; i < 4; i++) {
      motor_data_.target_motor_omega_3508[i] = target_motor_omega_[i];
      motor_data_.output_current_3508[i] =
          output_[i] *
          (motor_wheel_0_->GetLSB() / PARAM.reductionratio /
           motor_wheel_0_->KGetTorque() / motor_wheel_0_->GetCurrentMAX());
    }

    /* 设置3508电机数据 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508,
                                     motor_data_.target_motor_omega_3508,
                                     motor_data_.current_motor_omega_3508);
    power_control_->CalculatePowerControlParam();
    power_control_->OutputLimit(60);
    power_control_data_ = power_control_->GetPowerControlData();
  }

  /**
   * @brief 麦轮底盘动力学输出
   * @details 限幅并输出四个麦轮的电流控制指令
   */
  void OutputToDynamics() {
    /*如果超功率了output根据功率的数值来计算*/
    if (power_control_data_.is_power_limited) {
      for (int i = 0; i < 4; i++) {
        output_[i] =
            power_control_data_.new_output_current_3508[i] /
            (motor_wheel_0_->GetLSB() / PARAM.reductionratio /
             motor_wheel_0_->KGetTorque() / motor_wheel_0_->GetCurrentMAX());
      }
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

  LibXR::PID<float> pid_steer_speed_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  CMD *cmd_;

  MotorData motor_data_={};
  PowerControl *power_control_;
  PowerControlData power_control_data_;

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  CMD::ChassisCMD cmd_data_;
  uint32_t chassis_event_ = 0;
};
