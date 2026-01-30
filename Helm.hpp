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
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "dsp/fast_math_functions.h"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO \
  52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */
#define GM6020_NM_TO_LSB_RATIO \
  7370.0f                            /* 6020转子扭矩转化为电机控制单位的比例 */
#define HELM_CHASSIS_MAX_POWER 60.0f /* 底盘最大功率 */

template <typename ChassisType>
class Chassis;
class Helm {
 public:
  struct ChassisParam {
    float wheel_radius = 0.0f;
    float wheel_to_center = 0.0f;
    float gravity_height = 0.0f;
    float reduction_ratio = 0.0f;
    float wheel_resistance = 0.0f;
    float error_compensation = 0.0f;
    float gravity = 0.0f;
  };
  enum class ChassisMode : uint8_t {
    RELAX,
    INDEPENDENT,
    ROTOR,
    FOLLOW,
  };

  /**
   * @brief 构造函数，初始化舵轮底盘控制对象
   * @param hw 硬件容器引用
   * @param app 应用管理器引用
   * @param cmd 控制命令引用
   * @param motor_wheel_0 第0个驱动轮电机指针
   * @param motor_wheel_1 第1个驱动轮电机指针
   * @param motor_wheel_2 第2个驱动轮电机指针
   * @param motor_wheel_3 第3个驱动轮电机指针
   * @param motor_steer_0 第0个舵向电机指针
   * @param motor_steer_1 第1个舵向电机指针
   * @param motor_steer_2 第2个舵向电机指针
   * @param motor_steer_3 第3个舵向电机指针
   * @param task_stack_depth 控制线程栈深度
   * @param chassis_param 舵轮底盘参数
   * @param pid_follow 跟随云台角度PID参数
   * @param pid_velocity_x X方向速度PID参数
   * @param pid_velocity_y Y方向速度PID参数
   * @param pid_omega 角速度PID参数
   * @param pid_wheel_omega_0 轮子0角速度PID参数
   * @param pid_wheel_omega_1 轮子1角速度PID参数
   * @param pid_wheel_omega_2 轮子2角速度PID参数
   * @param pid_wheel_omega_3 轮子3角速度PID参数
   * @param pid_steer_angle_0 舵机0角度PID参数
   * @param pid_steer_angle_1 舵机1角度PID参数
   * @param pid_steer_angle_2 舵机2角度PID参数
   * @param pid_steer_angle_3 舵机3角度PID参数
   */
  Helm(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
       Motor *motor_wheel_0, Motor *motor_wheel_1, Motor *motor_wheel_2,
       Motor *motor_wheel_3, Motor *motor_steer_0, Motor *motor_steer_1,
       Motor *motor_steer_2, Motor *motor_steer_3, CMD *cmd,
       PowerControl *power_control, uint32_t task_stack_depth,
       ChassisParam chassis_param, LibXR::PID<float>::Param pid_follow,
       LibXR::PID<float>::Param pid_velocity_x,
       LibXR::PID<float>::Param pid_velocity_y,
       LibXR::PID<float>::Param pid_omega,
       LibXR::PID<float>::Param pid_wheel_speed_0,
       LibXR::PID<float>::Param pid_wheel_speed_1,
       LibXR::PID<float>::Param pid_wheel_speed_2,
       LibXR::PID<float>::Param pid_wheel_speed_3,
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
        motor_steer_0_(motor_steer_0),   /*wheel3   ▲ y  wheel2*/
        motor_steer_1_(motor_steer_1),   /*    ↑    │     ↑    */
        motor_steer_2_(motor_steer_2),   /*         │          */
        motor_steer_3_(motor_steer_3),   /* ――――――――│―――――――▶x */
        pid_follow_(pid_follow),         /*         │          */
        pid_velocity_x_(pid_velocity_x), /*    ↑    │     ↑    */
        pid_velocity_y_(pid_velocity_y), /*wheel0   │    wheel1*/
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1,
                         pid_wheel_speed_2, pid_wheel_speed_3},
        pid_steer_angle_{
            pid_steer_angle_0,
            pid_steer_angle_1,
            pid_steer_angle_2,
            pid_steer_angle_3,
        },
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1,
                         pid_steer_speed_2, pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control) {
    UNUSED(hw);
    UNUSED(app);

    for (int i = 0; i < 4; i++) {
      motor_wheel_cmd_[i].mode = Motor::ControlMode::MODE_CURRENT;
      motor_wheel_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_wheel_cmd_[i].torque = 0.0f;
      motor_wheel_cmd_[i].position = 0.0f;
      motor_wheel_cmd_[i].velocity = 0.0f;
      motor_wheel_cmd_[i].kp = 0.0f;
      motor_wheel_cmd_[i].kd = 0.0f;
    }
    for (int i = 0; i < 4; i++) {
      motor_steer_cmd_[i].mode = Motor::ControlMode::MODE_CURRENT;
      motor_steer_cmd_[i].reduction_ratio = 1.0f;
      motor_steer_cmd_[i].torque = 0.0f;
      motor_steer_cmd_[i].position = 0.0f;
      motor_steer_cmd_[i].velocity = 0.0f;
      motor_steer_cmd_[i].kp = 0.0f;
      motor_steer_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "HelmChassisThread", task_stack_depth,
                   LibXR::Thread::Priority::MEDIUM);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm *helm, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 舵轮底盘控制线程函数
   * @param helm Helm对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Helm *helm) {
    helm->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    helm->mutex_.Unlock();
    while (true) {
      auto last_time = LibXR::Timebase::GetMilliseconds();
      if (cmd_suber.Available()) {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available()) {
        helm->current_yaw_ = LibXR::CycleValue<float>(yawmotor_angle_suber.GetData()-helm->yawmotor_zero_);
        yawmotor_angle_suber.StartWaiting();
      }

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateCMD();
      helm->Helmcontrol();
      helm->PowerControlUpdate();
      helm->mutex_.Unlock();
      helm->Output();

      helm->thread_.SleepUntil(last_time,2);
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

    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Update();
      motor_steer_[i]->Update();
      motor_wheel_feedback_[i] = motor_wheel_[i]->GetFeedback();
      motor_steer_feedback_[i] = motor_steer_[i]->GetFeedback();
    }
  }

  /**
   * @brief 失去控制处理
   *
   */
  void LostCtrl() {
    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Relax();
      motor_steer_[i]->Relax();
    }
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++) {
      pid_wheel_speed_[i].Reset();
      pid_steer_angle_[i].Reset();
      pid_steer_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CMD获取底盘控制指令,速控底盘
   */
  void UpdateCMD() {
    target_vx_ = cmd_data_.x;
    target_vy_ = cmd_data_.y;
    target_omega_ = cmd_data_.z;
  }

  /**
   * @brief 功率控制更新
   *
   */
  void PowerControlUpdate() {
    for (int i = 0; i < 4; i++) {
      motor_data_.rotorspeed_rpm_3508[i] = motor_wheel_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_wheel_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;

      motor_data_.rotorspeed_rpm_6020[i] = motor_steer_feedback_[i].velocity;
      motor_data_.output_current_6020[i] =
          motor_steer_feedback_[i].torque * GM6020_NM_TO_LSB_RATIO;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020);

    power_control_->CalculatePowerControlParam();

    for (int i = 0; i < 4; i++) {
      motor_data_.output_current_3508[i] =
          std::clamp(power_control_data_.new_output_current_3508[i] * 16384.0f,
                     -16384.0f, 16384.0f);
      motor_data_.output_current_6020[i] =
          std::clamp(power_control_data_.new_output_current_6020[i] * 16384.0f,
                     -16384.0f, 16384.0f);
    }
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);
    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020);

    power_control_->OutputLimit(HELM_CHASSIS_MAX_POWER);
    power_control_data_ = power_control_->GetPowerControlData();
  }

  /**
   * @brief 速控底盘控制
   * @details 多模式控制底盘
   */

  void Helmcontrol() {
    const float SQRT2 = 1.41421356237f;

    // 计算 vx,xy
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::INDEPENDENT): {
        target_vx_ = cmd_data_.x;
        target_vy_ = cmd_data_.y;
      } break;
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR): {
        float beta = -current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = cos_beta * cmd_data_.x - sin_beta * cmd_data_.y;
        target_vy_ = sin_beta * cmd_data_.x + cos_beta * cmd_data_.y;
      } break;

      default:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
    }

    /* 计算 wz */
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;
      case (ChassisMode::INDEPENDENT):
        /* 独立模式每个轮子的方向相同，wz当作轮子转向角速度 */
        target_omega_ = cmd_data_.z;
        break;
      case (ChassisMode::FOLLOW):
        target_omega_ = -pid_follow_.Calculate(0.0f, current_yaw_, dt_);
        break;
      case (ChassisMode::ROTOR):
        /* 陀螺模式底盘以一定速度旋转 */
        target_omega_ = -wz_dir_mult_;
        break;
      default:
        target_omega_ = 0.0f;
        break;
    }

    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        for (int i = 0; i < 4; i++) {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0;
        }
        break;
      case (ChassisMode::INDEPENDENT):
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR): {
        float x = 0, y = 0, wheel_pos = 0;
        for (int i = 0; i < 4; i++) {
          wheel_pos = -static_cast<float>(i) * static_cast<float>(M_PI_2) +
                      static_cast<float>(M_PI) / 4.0f * 3.0f;
          x = -sinf(wheel_pos) * target_omega_ + target_vx_;
          y = -cosf(wheel_pos) * target_omega_ + target_vy_;

          target_angle_[i] = M_PI_2 - atan2f(y, x);
          target_speed_[i] = motor_max_speed_ * sqrtf(x * x + y * y) * SQRT2;
        }
      } break;
      default:
        for (int i = 0; i < 4; i++) {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0;
        }
        break;
    }

    /**
     * @brief 判断3508电机是否需要反转
     *
     */
    for (int i = 0; i < 4; i++) {
      if (fabs(
              LibXR::CycleValue(motor_steer_feedback_[i].abs_angle - zero_[i]) -
              target_angle_[i]) > M_PI_2) {
        motor_reverse_[i] = true;
      } else {
        motor_reverse_[i] = false;
      }
    }

    /**
     * @brief PID计算输出
     *
     */
    for (int i = 0; i < 4; i++) {
      if (motor_reverse_[i]) {
        wheel_out_[i] = pid_wheel_speed_[i].Calculate(
            -target_speed_[i], motor_wheel_feedback_[i].velocity, dt_);

        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            LibXR::CycleValue<float>(target_angle_[i] +
                                     static_cast<float>(M_PI) + zero_[i]),
            motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] =
            pid_steer_speed_[i].Calculate(steer_angle_[i],
                                          motor_steer_feedback_[i].velocity *
                                              static_cast<float>(M_2PI) / 60.0f,
                                          dt_);
      } else {
        wheel_out_[i] = pid_wheel_speed_[i].Calculate(
            target_speed_[i], motor_wheel_feedback_[i].velocity, dt_);
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            target_angle_[i] + zero_[i], motor_steer_feedback_[i].abs_angle,
            dt_);
        steer_out_[i] =
            pid_steer_speed_[i].Calculate(steer_angle_[i],
                                          motor_steer_feedback_[i].velocity *
                                              static_cast<float>(M_2PI) / 60.0f,
                                          dt_);
      }
    }
  }

  /**
   * @brief 输出控制命令到电机
   *
   */
  void Output() {
    if (power_control_data_.is_power_limited) {
      for (int i = 0; i < 4; i++) {
        wheel_out_[i] =
            power_control_data_.new_output_current_3508[i] / 16384.0f;
        steer_out_[i] =
            power_control_data_.new_output_current_6020[i] / 16384.0f;
      }
    }

    if (chassis_event_ == (ChassisMode::RELAX)) {
      LostCtrl();
    } else {
      for (int i = 0; i < 4; i++) {
        motor_wheel_cmd_[i].velocity =
            std::clamp(wheel_out_[i], -16384.0f, 16384.0f);
        motor_steer_cmd_[i].velocity =
            std::clamp(steer_out_[i], -16384.0f, 16384.0f);
      }
      for (int i = 0; i < 4; i++) {
        motor_wheel_[i]->Control(motor_wheel_cmd_[i]);
        motor_steer_[i]->Control(motor_steer_cmd_[i]);
      }
    }
  }

 private:
  const ChassisParam PARAM;

  float target_vx_ = 0.0f;
  float target_vy_ = 0.0f;
  float target_omega_ = 0.0f;

  /* 小陀螺模式旋转方向乘数 */
  float wz_dir_mult_ = 1.0f;
  bool motor_reverse_[4]{false, false, false, false};
  LibXR::CycleValue<float> zero_[4] = {1.11367011, 3.1254859, 0.479369015,
                                       3.55500054};

  float current_yaw_ = 0.0f;
  float yawmotor_zero_=3.89784527;

  /* 转子的转速 */
  float target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::CycleValue<float> target_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /* 输出的电流值 */
  float wheel_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_angle_[4] = {0.0, 0.0, 0.0, 0.0};

  float motor_max_speed_ = 9000.0;

  LibXR::CycleValue<float> main_direct_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor *motor_wheel_0_;
  Motor *motor_wheel_1_;
  Motor *motor_wheel_2_;
  Motor *motor_wheel_3_;
  Motor *motor_steer_0_;
  Motor *motor_steer_1_;
  Motor *motor_steer_2_;
  Motor *motor_steer_3_;

  Motor *motor_wheel_[4] = {motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                            motor_wheel_3_};
  Motor *motor_steer_[4] = {motor_steer_0_, motor_steer_1_, motor_steer_2_,
                            motor_steer_3_};

  Motor::Feedback motor_wheel_feedback_[4]{};
  Motor::Feedback motor_steer_feedback_[4]{};

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {
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
  CMD::ChassisCMD cmd_data_;

  Motor::MotorCmd motor_wheel_cmd_[4]{};
  Motor::MotorCmd motor_steer_cmd_[4]{};

  MotorData motor_data_ = {};
  PowerControl *power_control_;
  PowerControlData power_control_data_;

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  ChassisMode chassis_event_ = ChassisMode::RELAX;
};
