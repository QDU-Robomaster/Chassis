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
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"

template <typename ChassisType>
class Chassis;
class Helm {
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
    INDEPENDENT,
    ROTOR,
    FOLLOW6020,
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
       RMMotor *motor_wheel_0, RMMotor *motor_wheel_1, RMMotor *motor_wheel_2,
       RMMotor *motor_wheel_3, RMMotor *motor_steer_0, RMMotor *motor_steer_1,
       RMMotor *motor_steer_2, RMMotor *motor_steer_3, CMD *cmd,
       PowerControl *power_control, uint32_t task_stack_depth,
       ChassisParam chassis_param,
       LibXR::PID<float>::Param pid_follow,
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
        motor_steer_0_(motor_steer_0),
        motor_steer_1_(motor_steer_1),
        motor_steer_2_(motor_steer_2),
        motor_steer_3_(motor_steer_3),
        pid_follow_(pid_follow),
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
    auto last_time = LibXR::Timebase::GetMilliseconds();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<float> current_yaw_suber("chassis_yaw");

    cmd_suber.StartWaiting();
    current_yaw_suber.StartWaiting();

    helm->mutex_.Unlock();
    while (true) {
      if (cmd_suber.Available()) {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (current_yaw_suber.Available()) {
        helm->current_yaw_ = current_yaw_suber.GetData();
        current_yaw_suber.StartWaiting();
      }

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateCMD();
      helm->Helmcontrol();
      helm->PowerControlUpdate();
      helm->mutex_.Unlock();
      helm->Output();
      helm->thread_.SleepUntil(last_time, 2);
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
    motor_steer_0_->Update();
    motor_steer_1_->Update();
    motor_steer_2_->Update();
    motor_steer_3_->Update();
  }

  void LostCtrl() {
    motor_wheel_0_->Relax();
    motor_wheel_1_->Relax();
    motor_wheel_2_->Relax();
    motor_wheel_3_->Relax();
    motor_steer_0_->Relax();
    motor_steer_1_->Relax();
    motor_steer_2_->Relax();
    motor_steer_3_->Relax();
  }

  /**
   * @brief 设置底盘模式 (由 Chassis 外壳调用)
   * @param mode 要设置的新模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    for (int i = 0; i < 4; i++) {
      pid_wheel_omega_[i].Reset();
      pid_steer_angle_[i].Reset();
      pid_steer_speed_[i].Reset();
    }
    chassis_event_ = mode;
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

  void PowerControlUpdate() {
    /*3508电机数据*/
    motor_data_.rotorspeed_rpm_3508[0] = motor_wheel_0_->GetRPM();
    motor_data_.rotorspeed_rpm_3508[1] = motor_wheel_1_->GetRPM();
    motor_data_.rotorspeed_rpm_3508[2] = motor_wheel_2_->GetRPM();
    motor_data_.rotorspeed_rpm_3508[3] = motor_wheel_3_->GetRPM();

    motor_data_.current_motor_omega_3508[0] =
        motor_wheel_0_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_3508[1] =
        motor_wheel_1_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_3508[2] =
        motor_wheel_2_->GetOmega() / PARAM.reductionratio;
    motor_data_.current_motor_omega_3508[3] =
        motor_wheel_3_->GetOmega() / PARAM.reductionratio;

    for (int i = 0; i < 4; i++) {
      motor_data_.target_motor_omega_3508[i] =
          static_cast<float>(speed_[i] / PARAM.reductionratio);
      motor_data_.output_current_3508[i] = wheel_out_[i];
    }

    /*6020电机数据*/
    motor_data_.rotorspeed_rpm_6020[0] = motor_steer_0_->GetRPM();
    motor_data_.rotorspeed_rpm_6020[1] = motor_steer_1_->GetRPM();
    motor_data_.rotorspeed_rpm_6020[2] = motor_steer_2_->GetRPM();
    motor_data_.rotorspeed_rpm_6020[3] = motor_steer_3_->GetRPM();

    motor_data_.current_motor_omega_6020[0] = motor_steer_0_->GetOmega();
    motor_data_.current_motor_omega_6020[1] = motor_steer_1_->GetOmega();
    motor_data_.current_motor_omega_6020[2] = motor_steer_2_->GetOmega();
    motor_data_.current_motor_omega_6020[3] = motor_steer_3_->GetOmega();

    for (int i = 0; i < 4; i++) {
      motor_data_.target_motor_omega_6020[i] =
          static_cast<float>(steer_angle_[i] * 60.0f / M_2PI);
      motor_data_.output_current_6020[i] = steer_out_[i];
    }

    /* 设置3508电机数据 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508,
                                     motor_data_.target_motor_omega_3508,
                                     motor_data_.current_motor_omega_3508);

    /* 设置6020电机数据 */
    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020,
                                     motor_data_.target_motor_omega_6020,
                                     motor_data_.current_motor_omega_6020);

    power_control_->CalculatePowerControlParam();
    power_control_->OutputLimit(20);
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
        case static_cast<uint32_t>(Chassismode::RELAX):  // break
          target_vx_ = 0.0f;
          target_vy_ = 0.0f;
          break;
        case static_cast<uint32_t>(Chassismode::INDEPENDENT): //独立模式
        case static_cast<uint32_t>(Chassismode::FOLLOW6020):  // 跟随6020

          tmp_ = sqrtf(cmd_data_.x * cmd_data_.x + cmd_data_.y * cmd_data_.y) *
                 SQRT2 / 2.0f;
          tmp_ = std::clamp(tmp_, -1.0f, 1.0f);

          target_vx_ = 0;
          target_vy_ = tmp_;
          if (tmp_ >= 0.01) {
            direct_offset_ = M_PI_2 - atan2f(cmd_data_.y, cmd_data_.x);
          } else {
            direct_offset_ = 0;
          }
          break;

        case static_cast<uint32_t>(Chassismode::FOLLOW):  // gimbal_follow
        case static_cast<uint32_t>(Chassismode::ROTOR): {
          float beta = current_yaw_;
          float cos_beta = cosf(beta);
          float sin_beta = sinf(beta);
          target_vx_ = cos_beta * cmd_data_.x - sin_beta * cmd_data_.y;  // 最大为1-(sqrt2)/2
          target_vy_ = sin_beta * cmd_data_.x + cos_beta * cmd_data_.y;
        } break;

        default:
          target_vx_ = 0.0f;
          target_vy_ = 0.0f;
          break;
      }

      /* 计算 wz */
      switch (chassis_event_) {
        case static_cast<uint32_t>(Chassismode::RELAX):
          target_omega_ = 0.0f;
          break;
        case static_cast<uint32_t>(Chassismode::INDEPENDENT):
          /* 独立模式每个轮子的方向相同，wz当作轮子转向角速度 */
          target_omega_ = cmd_data_.z;
          main_direct_ -= target_omega_ * 6.0f * dt_;  //  6.0为小陀螺转动频率
          break;
        case static_cast<uint32_t>(Chassismode::FOLLOW6020):  // 6020_follow
          target_omega_ = 0;
          main_direct_ = -current_yaw_;
          break;
        case static_cast<uint32_t>(Chassismode::FOLLOW):  // gimbal_follow
          target_omega_ = pid_follow_.Calculate(0.0f, current_yaw_, dt_) * 0.25f;
          break;
        case static_cast<uint32_t>(Chassismode::ROTOR):  // rotor
          /* 陀螺模式底盘以一定速度旋转 */
          target_omega_ = wz_dir_mult_ * 1;  // 此处100为之前随机转速
          break;
        default:
          target_omega_ = 0.0f;
          break;
      }

      /* 计算 */
      switch (chassis_event_) {
        case static_cast<uint32_t>(Chassismode::RELAX):  // break
          for (int i = 0; i < 4; i++) {
            speed_[i] = 0.0f;
            angle_[i] = 0.0;
          }
          break;
        case static_cast<uint32_t>(Chassismode::INDEPENDENT):  // independent
        case static_cast<uint32_t>(Chassismode::FOLLOW6020):   // 6020_follow
          for (int i = 0; i < 4; i++) {
            speed_[i] = target_vy_ * motor_max_speed_;
            angle_[i] = main_direct_ + direct_offset_;
          }
          break;
        case static_cast<uint32_t>(Chassismode::FOLLOW):  // gimbal_follow
        case static_cast<uint32_t>(Chassismode::ROTOR):   // rotor
        {
          float x = 0, y = 0, wheel_pos = 0;
          for (int i = 0; i < 4; i++) {
            wheel_pos = -static_cast<float>(i) * static_cast<float>(M_PI_2) +
                        static_cast<float>(M_PI) / 4.0f * 3.0f;
            x = sinf(wheel_pos) * target_omega_ + target_vx_;
            y = cosf(wheel_pos) * target_omega_ + target_vy_;
            angle_[i] = M_PI_2 - atan2f(y, x);
            speed_[i] = motor_max_speed_ * sqrtf(x * x + y * y) * SQRT2 / 2.0f;
          }
        } break;
        default:
          for (int i = 0; i < 4; i++) {
            speed_[i] = 0.0f;
            angle_[i] = 0.0;
          }
          break;
      }
      /* 最短路径 */
      for (int i = 0; i < 4; i++) {
        switch (i) {
          case 0:
            if (fabs(LibXR::CycleValue(motor_steer_0_->GetAngle() - zero_[i]) -
                     angle_[i]) > M_PI_2) {
              motor_reverse_[i] = true;
            } else {
              motor_reverse_[i] = false;
            }
            break;
          case 1:
            if (fabs(LibXR::CycleValue(motor_steer_1_->GetAngle() - zero_[i]) -
                     angle_[i]) > M_PI_2) {
              motor_reverse_[i] = true;
            } else {
              motor_reverse_[i] = false;
            }
            break;
          case 2:
            if (fabs(LibXR::CycleValue(motor_steer_2_->GetAngle() - zero_[i]) -
                     angle_[i]) > M_PI_2) {
              motor_reverse_[i] = true;
            } else {
              motor_reverse_[i] = false;
            }
            break;
          case 3:
            if (fabs(LibXR::CycleValue(motor_steer_3_->GetAngle() - zero_[i]) -
                     angle_[i]) > M_PI_2) {
              motor_reverse_[i] = true;
            } else {
              motor_reverse_[i] = false;
            }
            break;

          default:
            break;
        }
      }
      /* 输出计算 */
      for (int i = 0; i < 4; i++) {
        switch (i) {
          case 0:
            if (motor_reverse_[i]) {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  -speed_[i], motor_wheel_0_->GetRPM(), dt_);

              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  LibXR::CycleValue<float>(angle_[i] +
                                           static_cast<float>(M_PI) + zero_[i]),
                  motor_steer_0_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_0_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            } else {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  speed_[i], motor_wheel_0_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  angle_[i] + zero_[i], motor_steer_0_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_0_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            }
            break;
          case 1:
            if (motor_reverse_[i]) {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  -speed_[i], motor_wheel_1_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  LibXR::CycleValue<float>(angle_[i] +
                                           static_cast<float>(M_PI) + zero_[i]),
                  motor_steer_1_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_1_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            } else {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  speed_[i], motor_wheel_1_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  angle_[i] + zero_[i], motor_steer_1_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_1_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            }
            break;
          case 2:
            if (motor_reverse_[i]) {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  -speed_[i], motor_wheel_2_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  LibXR::CycleValue<float>(angle_[i] +
                                           static_cast<float>(M_PI) + zero_[i]),
                  motor_steer_2_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_2_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            } else {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  speed_[i], motor_wheel_2_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  angle_[i] + zero_[i], motor_steer_2_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_2_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            }
            break;
          case 3:
            if (motor_reverse_[i]) {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  -speed_[i], motor_wheel_3_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  LibXR::CycleValue<float>(angle_[i] +
                                           static_cast<float>(M_PI) + zero_[i]),
                  motor_steer_3_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_3_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            } else {
              wheel_out_[i] = pid_wheel_omega_[i].Calculate(
                  speed_[i], motor_wheel_3_->GetRPM(), dt_);
              steer_angle_[i] = pid_steer_angle_[i].Calculate(
                  angle_[i] + zero_[i], motor_steer_3_->GetAngle(), dt_);
              steer_out_[i] = pid_steer_speed_[i].Calculate(
                  steer_angle_[i],
                  motor_steer_3_->GetRPM() * static_cast<float>(M_2PI) / 60.0f,
                  dt_);
            }
            break;
          default:
            break;
        }
      }
    }

    void Output() {
      if (power_control_data_.is_power_limited) {
        for (int i = 0; i < 4; i++) {
          wheel_out_[i] = power_control_data_.new_output_current_3508[i];
          steer_out_[i] = power_control_data_.new_output_current_6020[i];
        }
      }

      if (chassis_event_ == static_cast<uint32_t>(Chassismode::RELAX)) {
        LostCtrl();
      }
      else {
      motor_wheel_0_->CurrentControl(wheel_out_[0]);
      motor_wheel_1_->CurrentControl(wheel_out_[1]);
      motor_wheel_2_->CurrentControl(wheel_out_[2]);
      motor_wheel_3_->CurrentControl(wheel_out_[3]);
      motor_steer_0_->CurrentControl(steer_out_[0]);
      motor_steer_1_->CurrentControl(steer_out_[1]);
      motor_steer_2_->CurrentControl(steer_out_[2]);
      motor_steer_3_->CurrentControl(steer_out_[3]);
      }
    }

   private:
    const ChassisParam PARAM;

    float target_vx_ = 0.0f;
    float target_vy_ = 0.0f;
    float target_omega_ = 0.0f;

    float tmp_ = 0.0f;

    /* 小陀螺模式旋转方向乘数 */
    float wz_dir_mult_ = 1.0f;
    bool motor_reverse_[4]{false, false, false, false};
    LibXR::CycleValue<float> zero_[4] = {3.22642668, 2.09004819,
                                         2.5839184,  6.25634098};

    float current_yaw_ = 0.0f;

    /* 转子的转速 */
    float speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    LibXR::CycleValue<float> angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    /* 输出的电流值 */
    float wheel_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float steer_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float steer_angle_[4] = {0.0, 0.0, 0.0, 0.0};

    float motor_max_speed_ = 9000.0;

    float direct_offset_ = 0.0f;
    LibXR::CycleValue<float> main_direct_ = 0.0f;

    float dt_ = 0;
    LibXR::MicrosecondTimestamp last_online_time_ = 0;

    RMMotor *motor_wheel_0_;
    RMMotor *motor_wheel_1_;
    RMMotor *motor_wheel_2_;
    RMMotor *motor_wheel_3_;
    RMMotor *motor_steer_0_;
    RMMotor *motor_steer_1_;
    RMMotor *motor_steer_2_;
    RMMotor *motor_steer_3_;

    LibXR::PID<float> pid_follow_;
    LibXR::PID<float> pid_velocity_x_;
    LibXR::PID<float> pid_velocity_y_;
    /* 此时姑且认为pid_omega_为gimbal_follow的pid */
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

    MotorData motor_data_ = {};
    PowerControl *power_control_;
    PowerControlData power_control_data_;

    LibXR::Thread thread_;
    LibXR::Mutex mutex_;

    CMD::ChassisCMD cmd_data_;
    uint32_t chassis_event_ = 0;
  };
