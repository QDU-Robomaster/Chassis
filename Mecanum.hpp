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
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#endif

#define M3508_NM_TO_LSB_RATIO \
  52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */

#define MECANUM_MOTOR_MAX_OMEGA 52      /* 电机输出轴最大角速度 */
#define MECANUM_CHASSIS_MAX_POWER 60.0f /* 底盘最大功率 */

template <typename ChassisType>
class Chassis;

class Mecanum {
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
  Mecanum(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
          Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
          Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
          Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd,
          PowerControl* power_control, uint32_t task_stack_depth,
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
        motor_wheel_0_(motor_wheel_0),   /*wheel0   ▲ y  wheel3*/
        motor_wheel_1_(motor_wheel_1),   /*    ↑    │     ↓    */
        motor_wheel_2_(motor_wheel_2),   /*         │          */
        motor_wheel_3_(motor_wheel_3),   /* ――――――――│―――――――▶x */
        pid_follow_(pid_follow),         /*         │          */
        pid_velocity_x_(pid_velocity_x), /*    ↑    │     ↓    */
        pid_velocity_y_(pid_velocity_y), /*wheel1   │    wheel2*/
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1,
                         pid_wheel_speed_2, pid_wheel_speed_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3},
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1,
                         pid_steer_speed_2, pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control)
#ifdef DEBUG
        ,
        cmd_file_(LibXR::RamFS::CreateFile(
            "mecanum_chassis",
            debug_core::command_thunk<Mecanum, &Mecanum::DebugCommand>, this))
#endif
  {
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

    for (int i = 0; i < 4; i++) {
      motor_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_cmd_[i].torque = 0.0f;
      motor_cmd_[i].position = 0.0f;
      motor_cmd_[i].velocity = 0.0f;
      motor_cmd_[i].kp = 0.0f;
      motor_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "MecanumChassisThread",
                   task_stack_depth, LibXR::Thread::Priority::HIGH);
#ifdef DEBUG
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
#endif
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 麦轮底盘控制线程函数
   * @param mecanum Mecanum对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Mecanum* mecanum) {
    mecanum->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    mecanum->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    mecanum->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }
      if (yawmotor_angle_suber.Available()) {
        mecanum->current_yaw_ = LibXR::CycleValue<float>(
            yawmotor_angle_suber.GetData() - mecanum->yawmotor_zero_);
        yawmotor_angle_suber.StartWaiting();
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

    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Update();
      motor_feedback_[i] = motor_wheel_[i]->GetFeedback();
    }
  }

  /**
   * @brief 设置底盘模式
   */
  void SetMode(uint32_t mode) {
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++) {
      pid_wheel_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CMD获取底盘控制指令，并转换为目标速度
   */
  void UpdateCMD() {
    float max_v = PARAM.wheel_radius * MECANUM_MOTOR_MAX_OMEGA;

    /*计算omega*/
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;
      case (ChassisMode::ROTOR):
        this->target_omega_ = PARAM.wheel_radius * MECANUM_MOTOR_MAX_OMEGA /
                              PARAM.wheel_to_center;
        break;
        /*TODO:这里可能有点问题*/
      case (ChassisMode::FOLLOW):
        target_omega_ = this->pid_follow_.Calculate(0.0f, -current_yaw_, dt_);
        break;
      case (ChassisMode::INDEPENDENT):
        target_omega_ = -cmd_data_.z * MECANUM_MOTOR_MAX_OMEGA *
                        PARAM.wheel_radius / PARAM.wheel_to_center;
        break;
      default:
        break;
    }

    /* 计算vx,vy */
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::ROTOR):
      case (ChassisMode::FOLLOW): {
        float beta = current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ =
            (cos_beta * cmd_data_.x * max_v + sin_beta * cmd_data_.y * max_v);
        target_vy_ =
            (-sin_beta * cmd_data_.x * max_v + cos_beta * cmd_data_.y * max_v);
      } break;
      case (ChassisMode::INDEPENDENT): {
        target_vx_ = cmd_data_.x * max_v;
        target_vy_ = cmd_data_.y * max_v;
      } break;
      default:
        break;
    }
  }

  /**
   * @brief 麦轮底盘正运动学解算
   * @details 根据四个麦轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    now_vx_ = (motor_feedback_[0].omega / PARAM.reduction_ratio -
               motor_feedback_[1].omega / PARAM.reduction_ratio -
               motor_feedback_[2].omega / PARAM.reduction_ratio +
               motor_feedback_[3].omega / PARAM.reduction_ratio) *
              PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
               motor_feedback_[1].omega / PARAM.reduction_ratio -
               motor_feedback_[2].omega / PARAM.reduction_ratio -
               motor_feedback_[3].omega / PARAM.reduction_ratio) *
              PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
                  motor_feedback_[1].omega / PARAM.reduction_ratio +
                  motor_feedback_[2].omega / PARAM.reduction_ratio +
                  motor_feedback_[3].omega / PARAM.reduction_ratio) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 麦轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个麦轮的目标角速度
   */
  void InverseKinematicsSolution() {
    target_motor_omega_[0] =
        (target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[1] =
        (-target_vx_ + target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[2] =
        (-target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
    target_motor_omega_[3] =
        (target_vx_ - target_vy_ + target_omega_ * PARAM.wheel_to_center) /
        PARAM.wheel_radius;
  }

  /**
   * @brief 计算 PID 输出电流
   */
  void CalculateMotorCurrent() {
    if (chassis_event_ == ChassisMode::RELAX) {
      LostCtrl();
    } else {
      for (int i = 0; i < 4; i++) {
        target_motor_current_[i] = pid_wheel_speed_[i].Calculate(
            target_motor_omega_[i],
            motor_feedback_[i].omega / PARAM.reduction_ratio, dt_);
      }
      /* 计算输出 */
      for (int i = 0; i < 4; i++) {
        output_[i] = target_motor_force_[i] * PARAM.wheel_radius +
                     target_motor_current_[i];
      }
    }
  }

  /**
   * @brief 功率控制更新
   */
  void PowerControlUpdate() {
    for (int i = 0; i < 4; i++) {
      motor_data_.rotorspeed_rpm_3508[i] = motor_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->CalculatePowerControlParam();

    for (int i = 0; i < 4; i++) {
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->OutputLimit(MECANUM_CHASSIS_MAX_POWER);
    power_control_data_ = power_control_->GetPowerControlData();
  }

  /**
   * @brief 麦轮底盘逆动力学解算
   * @details
   * 通过运动学正解算出底盘现在的运动状态，并与目标状态进行PID控制，获得目标前馈力矩
   */
  void DynamicInverseSolution() {
    float force_x = pid_velocity_x_.Calculate(target_vx_, now_vx_, dt_);
    float force_y = pid_velocity_y_.Calculate(target_vy_, now_vy_, dt_);
    float force_z = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    /* 分配（最小范数 / 平均分配 torque）//切向合力=质量*角加速度*转动半径 */
    target_motor_force_[0] = (force_x - force_y + force_z) / 4;
    target_motor_force_[1] = (force_x + force_y + force_z) / 4;
    target_motor_force_[2] = (-force_x + force_y + force_z) / 4;
    target_motor_force_[3] = (-force_x - force_y + force_z) / 4;
  }

  /**
   * @brief 麦轮底盘动力学输出
   * @details 限幅并输出四个麦轮的电流控制指令
   */
  void OutputToDynamics() {
    if (power_control_data_.is_power_limited) {
      for (int i = 0; i < 4; i++) {
        output_[i] =
            std::clamp(power_control_data_.new_output_current_3508[i] /
                           M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                       -6.0f, 6.0f);
      }
    }
    if (chassis_event_ == ChassisMode::RELAX) {
      LostCtrl();
      return;
    } else {
      for (int i = 0; i < 4; i++) {
        motor_cmd_[i].torque = std::clamp(output_[i], -6.0f, 6.0f);
      }
      for (int i = 0; i < 4; i++) {
        motor_wheel_[i]->Control(motor_cmd_[i]);
      }
    }
  }

  /**
   * @brief 失去控制处理
   *
   */
  void LostCtrl() {
    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Relax();
    }
  }
#ifdef DEBUG
  int DebugCommand(int argc, char** argv);
#endif

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
  float current_yaw_ = 0.0f;
  float yawmotor_zero_ = 0.959505022f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;

  Motor* motor_wheel_[4]{motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                         motor_wheel_3_};

  Motor::Feedback motor_feedback_[4]{};

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

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  Motor::MotorCmd motor_cmd_[4]{};

  MotorData motor_data_{};
  PowerControl* power_control_;
  PowerControlData power_control_data_;

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  ChassisMode chassis_event_ = ChassisMode::RELAX;

#ifdef DEBUG
  LibXR::RamFS::File cmd_file_;
#endif
};

#ifdef DEBUG
#define MECANUM_CHASSIS_DEBUG_IMPL
#include "MecanumDebug.inl"
#undef MECANUM_CHASSIS_DEBUG_IMPL
#endif
