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
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "SuperPower.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO \
  52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */

#define MECANUM_MOTOR_MAX_OMEGA 52.0f     /* 麦轮输出轴最大角速度 rad/s */
#define MECANUM_CHASSIS_MAX_POWER 400     /* 麦轮默认功率上限 W */
#define TRACK_MAX_LINEAR_SPEED_MPS 0.409f /* 履带最大线速度 m/s */

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
    float length = 0.0f;
    float width = 0.0f;
    float rotor_speed_scale =
        1.0f; /* 小陀螺转速缩放比例，降低可给平移留出更多功率 */
    float rotor_omega_min_scale = 0.55f; /* 动态缩放下限 */
    float rotor_buffer_low_j = 35.0f;    /* 缓冲能量低阈值 */
    float rotor_buffer_high_j = 70.0f;   /* 缓冲能量高阈值 */
    float rotor_scale_lpf_alpha = 0.2f;  /* 动态缩放低通系数 */
  };

  enum class ChassisMode : uint8_t {
    RELAX,
    INDEPENDENT,
    ROTOR,
    FOLLOW,
    TRACK_START,
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
   * @param motor_steer_0 第0个舵向电机指针（本底盘用作track）
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
   * @param pid_steer_angle_0 舵机0角度PID参数（本底盘用作track_speed_pid）
   * @param pid_steer_angle_1 舵机1角度PID参数（本底盘未使用）
   * @param pid_steer_angle_2 舵机2角度PID参数（本底盘未使用）
   * @param pid_steer_angle_3 舵机3角度PID参数（本底盘未使用）
   */
  Mecanum(
      LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
      Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
      Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
      Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd,
      PowerControl* power_control, Referee* referee, uint32_t task_stack_depth,
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
      LibXR::PID<float>::Param pid_steer_speed_3,
      LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM)
      /*
       * 麦轮编号，箭头为轮子正方向
       *
       *                 ↑ y
       *        wheel0   │   wheel3
       *          ↑      │      ↓
       *                 │
       * ────────────────┼──────────────→ x
       *                 │
       *          ↑      │      ↓
       *        wheel1   │   wheel2
       */
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        track_motor_(motor_steer_0),
        pid_follow_(pid_follow),
        pid_velocity_x_(pid_velocity_x),
        pid_velocity_y_(pid_velocity_y),
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1,
                         pid_wheel_speed_2, pid_wheel_speed_3},
        pid_track_speed_(pid_steer_angle_0),
        cmd_(cmd),
        power_control_(power_control),
        referee_(referee) {
    UNUSED(hw);
    UNUSED(app);
    UNUSED(motor_steer_1);
    UNUSED(motor_steer_2);
    UNUSED(motor_steer_3);
    UNUSED(pid_steer_speed_0);
    UNUSED(pid_steer_speed_1);
    UNUSED(pid_steer_speed_2);
    UNUSED(pid_steer_speed_3);
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
    track_motor_cmd_.mode = Motor::ControlMode::MODE_TORQUE;
    track_motor_cmd_.reduction_ratio = chassis_param.reduction_ratio;
    track_motor_cmd_.torque = 0.0f;
    track_motor_cmd_.position = 0.0f;
    track_motor_cmd_.velocity = 0.0f;
    track_motor_cmd_.kp = 0.0f;
    track_motor_cmd_.kd = 0.0f;

    thread_.Create(this, ThreadFunction, "MecanumChassisThread",
                   task_stack_depth, thread_priority);
    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->mutex_.Lock();
          mecanum->chassis_event_ = ChassisMode::RELAX;
          mecanum->mutex_.Unlock();
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Mecanum* mecanum, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          mecanum->mutex_.Lock();
          mecanum->chassis_event_ = ChassisMode::RELAX;
          mecanum->mutex_.Unlock();
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
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
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        "chassis_ref");
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    mecanum->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    mecanum->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        mecanum->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available()) {
        mecanum->referee_chassis_pack_ = referee_suber.GetData();
        mecanum->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available()) {
        mecanum->current_yaw_ = LibXR::CycleValue<float>(
            yawmotor_angle_suber.GetData() - mecanum->yawmotor_zero_);
        yawmotor_angle_suber.StartWaiting();
      }

      mecanum->mutex_.Lock();
      mecanum->Update();
      mecanum->UpdateTrack();
      mecanum->UpdateCMD();
      mecanum->SelfResolution();
      mecanum->InverseKinematicsSolution();
      mecanum->DynamicInverseSolution();
      mecanum->CalculateMotorCurrent();
      mecanum->CalculateTrackCurrent();
      mecanum->PowerControlUpdate();
      mecanum->mutex_.Unlock();
      mecanum->OutputToDynamics();
      mecanum->ControlTrack();

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
  void UpdateTrack() {
    if (track_motor_ == nullptr) {
      track_linear_speed_ = 0.0f;
      return;
    }
    track_motor_->Update();
    track_motor_feedback_ = track_motor_->GetFeedback();
    /* 转子角速度换算为履带线速度 */
    track_linear_speed_ = track_motor_feedback_.omega / PARAM.reduction_ratio *
                          TRACK_WHEEL_RADIUS_M;
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
    pid_track_speed_.Reset();
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

    /* 先生成目标角速度 */
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;

      case (ChassisMode::INDEPENDENT):
        target_omega_ = -max_v * cmd_data_.z / PARAM.wheel_to_center;
        break;

      case (ChassisMode::ROTOR):
        this->target_omega_ = static_cast<float>(max_v / PARAM.wheel_to_center);
        break;

      case (ChassisMode::FOLLOW):
        target_omega_ = -pid_follow_.Calculate(0.0f, -current_yaw_, dt_);
        break;

      case (ChassisMode::TRACK_START): {
        float max_omega = max_v / PARAM.wheel_to_center;
        /* 履带模式只保留小幅 FOLLOW 纠偏 */
        target_omega_ = -pid_follow_.Calculate(0.0f, -current_yaw_, dt_);
        target_omega_ = std::clamp(target_omega_,
                                   -max_omega * TRACK_FOLLOW_OMEGA_LIMIT_SCALE,
                                   max_omega * TRACK_FOLLOW_OMEGA_LIMIT_SCALE);
      } break;

      default:
        break;
    }

    /* 再生成目标平移速度 */
    switch (chassis_event_) {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::ROTOR):
      case (ChassisMode::FOLLOW): {
        float beta = -current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ =
            (cos_beta * cmd_data_.x * max_v + sin_beta * cmd_data_.y * max_v);
        target_vy_ =
            (-sin_beta * cmd_data_.x * max_v + cos_beta * cmd_data_.y * max_v);
      } break;
      case (ChassisMode::TRACK_START): {
        float beta = -current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        /* 履带负责前后，麦轮保留横移能力 */
        float assist_vx = cmd_data_.x * max_v * TRACK_LATERAL_SPEED_SCALE;
        float assist_vy = GetTrackWheelAssistSpeed();
        target_vx_ = cos_beta * assist_vx + sin_beta * assist_vy;
        target_vy_ = -sin_beta * assist_vx + cos_beta * assist_vy;
      } break;
      case (ChassisMode::INDEPENDENT): {
        target_vx_ = cmd_data_.x * max_v;
        target_vy_ = cmd_data_.y * max_v;
      } break;
      default:
        break;
    }

    /* 小陀螺模式下根据平移输入与功率状态动态调整旋转速度 */
    float rotor_translation_scale = 1.0f;
    if (chassis_event_ == ChassisMode::ROTOR) {
      float translation_magnitude =
          sqrtf(target_vx_ * target_vx_ + target_vy_ * target_vy_);
      float translation_ratio = 0.0f;
      if (max_v > 1e-3f) {
        translation_ratio =
            std::clamp(translation_magnitude / max_v, 0.0f, 1.0f);
      }
      rotor_translation_scale =
          1.0f - (1.0f - PARAM.rotor_speed_scale) * translation_ratio;
      target_omega_ *= rotor_translation_scale * rotor_dynamic_scale_;
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
    /* 采样当前反馈电流和转速供功率模型参数估计使用 */
    for (int i = 0; i < 4; i++) {
      motor_data_.rotorspeed_rpm_3508[i] = motor_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }
    /* 第五路是履带电机反馈 */
    motor_data_.rotorspeed_rpm_3508[4] =
        track_motor_ == nullptr ? 0.0f : track_motor_feedback_.velocity;
    motor_data_.output_current_3508[4] =
        track_motor_ == nullptr
            ? 0.0f
            : track_motor_feedback_.torque * M3508_NM_TO_LSB_RATIO;

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->CalculatePowerControlParam();

    float speed_error[5] = {};

    /* 写入五路期望电流供限功率使用 */
    for (int i = 0; i < 4; i++) {
      speed_error[i] = target_motor_omega_[i] -
                       motor_feedback_[i].omega / PARAM.reduction_ratio;
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }
    /* 履带线速度误差换算为主动轮角速度误差 */
    speed_error[4] = track_speed_error_ / TRACK_WHEEL_RADIUS_M;
    motor_data_.output_current_3508[4] = std::clamp(
        track_output_current_ * static_cast<float>(M3508_MAX_ABS_LSB),
        -static_cast<float>(M3508_MAX_ABS_LSB),
        static_cast<float>(M3508_MAX_ABS_LSB));

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508,
                                     speed_error);
    PowerControl::AllocationBias3508 allocation_bias{};
    if (track_motor_ != nullptr && chassis_event_ == ChassisMode::TRACK_START) {
      const float TRACK_CMD_MAG = GetTrackCommandMagnitude();
      const bool TRACK_ACTIVE = TRACK_CMD_MAG > TRACK_ACTIVE_SPEED_EPS_MPS;
      const bool TRACK_STALLED =
          fabsf(track_target_speed_) > TRACK_ACTIVE_SPEED_EPS_MPS &&
          fabsf(track_linear_speed_) <
              fabsf(track_target_speed_) * TRACK_STALL_SPEED_RATIO;
      float wheel_omega_abs_sum = 0.0f;
      for (int i = 0; i < 4; i++) {
        wheel_omega_abs_sum += fabsf(motor_feedback_[i].omega);
      }
      const bool WHEEL_FREE_SPIN =
          wheel_omega_abs_sum >
          TRACK_WHEEL_FREE_SPIN_OMEGA_RADPS * WHEEL_COUNT_FLOAT;
      const bool TRACK_NEEDS_PRIORITY =
          TRACK_ACTIVE &&
          (fabsf(track_speed_error_) > TRACK_PRIORITY_ERROR_EPS_MPS ||
           (TRACK_STALLED && WHEEL_FREE_SPIN));

      allocation_bias.enabled = true;
      allocation_bias.reserve_fraction = TRACK_NEEDS_PRIORITY
                                             ? TRACK_PRIORITY_RESERVE_FRACTION
                                             : TRACK_CRUISE_RESERVE_FRACTION;
      for (int i = 0; i < 4; i++) {
        allocation_bias.reserve_weight[i] = 0.0f;
        allocation_bias.allocation_weight_scale[i] =
            TRACK_NEEDS_PRIORITY ? TRACK_WHEEL_DEPRIORITY_SCALE
                                 : TRACK_WHEEL_ASSIST_PRIORITY_SCALE;
      }
      allocation_bias.reserve_weight[4] = 1.0f;
      allocation_bias.allocation_weight_scale[4] =
          TRACK_NEEDS_PRIORITY ? TRACK_PRIORITY_WEIGHT_SCALE
                               : TRACK_CRUISE_WEIGHT_SCALE;
    }
    power_control_->SetAllocationBias3508(allocation_bias);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    /* 裁判系统离线或上限异常时回退到本地默认功率上限 */
    float max_power =
        static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f) {
      max_power = MECANUM_CHASSIS_MAX_POWER;
    }

    /* BOOST 模式按电容能量分档提升可用功率上限 */
    if (power_control_online && boost_mode) {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f) {
        max_power += 300.0f;
      } else if (cap_energy > 0.5f) {
        max_power += 200.0f;
      } else if (cap_energy > 0.25f) {
        max_power += 100.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();

    /* 受限程度估计为限幅后电流总量除以请求电流总量 */
    float req_current_abs_sum = 0.0f;
    float lim_current_abs_sum = 0.0f;
    for (int i = 0; i < 5; i++) {
      float req_current_abs = fabsf(motor_data_.output_current_3508[i]);
      float lim_current_abs =
          power_control_data_.is_power_limited
              ? fabsf(power_control_data_.new_output_current_3508[i])
              : req_current_abs;
      req_current_abs_sum += req_current_abs;
      lim_current_abs_sum += lim_current_abs;
    }

    float power_limit_ratio = 1.0f;
    if (req_current_abs_sum > 1e-3f) {
      power_limit_ratio =
          std::clamp(lim_current_abs_sum / req_current_abs_sum, 0.0f, 1.0f);
    }

    /* 将裁判缓冲能量映射为缩放因子离线时保持 1.0 */
    float buffer_scale = 1.0f;
    if (referee_online) {
      float buffer_range =
          std::max(PARAM.rotor_buffer_high_j - PARAM.rotor_buffer_low_j, 1.0f);
      float referee_buffer_j =
          static_cast<float>(referee_chassis_pack_.power_buffer);
      float buffer_norm = std::clamp(
          (referee_buffer_j - PARAM.rotor_buffer_low_j) / buffer_range, 0.0f,
          1.0f);
      buffer_scale = PARAM.rotor_omega_min_scale +
                     (1.0f - PARAM.rotor_omega_min_scale) * buffer_norm;
    }

    /* 合成目标缩放并做一阶低通抑制跳变 */
    float limit_scale =
        power_control_data_.is_power_limited
            ? std::clamp(power_limit_ratio, PARAM.rotor_omega_min_scale, 1.0f)
            : 1.0f;
    float target_dynamic_scale = std::clamp(buffer_scale * limit_scale,
                                            PARAM.rotor_omega_min_scale, 1.0f);
    float lpf_alpha = std::clamp(PARAM.rotor_scale_lpf_alpha, 0.0f, 1.0f);
    rotor_dynamic_scale_ +=
        (target_dynamic_scale - rotor_dynamic_scale_) * lpf_alpha;
    rotor_dynamic_scale_ =
        std::clamp(rotor_dynamic_scale_, PARAM.rotor_omega_min_scale, 1.0f);

    /* 仅在小陀螺模式保留缩放其他模式统一复位 */
    if (chassis_event_ != ChassisMode::ROTOR) {
      rotor_dynamic_scale_ = 1.0f;
    }
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

    /* 按麦轮受力方向分配前馈力 */
    target_motor_force_[0] = (force_x + force_y + force_z) / 4;
    target_motor_force_[1] = (-force_x + force_y + force_z) / 4;
    target_motor_force_[2] = (-force_x - force_y + force_z) / 4;
    target_motor_force_[3] = (force_x - force_y + force_z) / 4;
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
  float GetTrackCommandMagnitude() const {
    /* 遥控 y 先缩放再开方让低速段更细 */
    const float INPUT = cmd_data_.y * TRACK_INPUT_SCALE;
    return std::sqrt(std::abs(INPUT)) * TRACK_MAX_LINEAR_SPEED_MPS;
  }
  float GetTrackSetpointSpeed() const {
    /* 履带电机方向和底盘前进方向相反 */
    const float INPUT = cmd_data_.y * TRACK_INPUT_SCALE;
    const float SIGN = (INPUT < 0.0f) ? 1.0f : -1.0f;
    return SIGN * GetTrackCommandMagnitude();
  }
  float GetTrackWheelAssistSpeed() const {
    /* 麦轮辅助方向和底盘前进方向一致 */
    const float INPUT = cmd_data_.y * TRACK_INPUT_SCALE;
    const float SIGN = (INPUT < 0.0f) ? -1.0f : 1.0f;
    return SIGN * GetTrackCommandMagnitude() * TRACK_WHEEL_ASSIST_SCALE;
  }
  void CalculateTrackCurrent() {
    if (track_motor_ == nullptr || chassis_event_ != ChassisMode::TRACK_START) {
      track_target_speed_ = 0.0f;
      track_output_current_ = 0.0f;
      track_speed_error_ = 0.0f;
      return;
    }

    const float DESIRED_TRACK_SPEED = GetTrackSetpointSpeed();
    const float MAX_DELTA = TRACK_SPEED_RAMP_MPS2 * dt_;
    /* 目标速度加斜坡避免履带突然打满 */
    track_target_speed_ += std::clamp(DESIRED_TRACK_SPEED - track_target_speed_,
                                      -MAX_DELTA, MAX_DELTA);
    track_speed_error_ = track_target_speed_ - track_linear_speed_;
    track_output_current_ = pid_track_speed_.Calculate(
        track_target_speed_, track_linear_speed_, dt_);
  }
  void ControlTrack() {
    if (track_motor_ == nullptr) {
      return;
    }

    mutex_.Lock();
    const bool RELAX = chassis_event_ == ChassisMode::RELAX;
    float track_output_current = track_output_current_;
    const PowerControlData POWER_CONTROL_DATA = power_control_data_;
    mutex_.Unlock();

    if (RELAX) {
      track_motor_->Relax();
      return;
    }
    if (POWER_CONTROL_DATA.is_power_limited) {
      /* 限功率时使用第五路重新分配后的电流 */
      track_output_current =
          std::clamp(POWER_CONTROL_DATA.new_output_current_3508[4] /
                         static_cast<float>(M3508_MAX_ABS_LSB),
                     -1.0f, 1.0f);
    }

    /* 按麦轮相同的电流到输出轴扭矩关系下发 */
    track_motor_cmd_.torque = std::clamp(
        track_output_current * static_cast<float>(M3508_MAX_ABS_LSB) /
            M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
        -6.0f, 6.0f);
    track_motor_cmd_.velocity = 0.0f;
    track_motor_->Control(track_motor_cmd_);
  }
  /**
   * @brief 失去控制处理
   *
   */
  void LostCtrl() {
    for (int i = 0; i < 4; i++) {
      motor_wheel_[i]->Relax();
    }
    if (track_motor_ != nullptr) {
      track_motor_->Relax();
    }
  }
#ifdef DEBUG
  int DebugCommand(int argc, char** argv);
#endif

 private:
  /* 履带模式使用完整遥控行程，避免目标速度被额外压低 */
  static constexpr float TRACK_INPUT_SCALE = 1.0f;
  /* 麦轮辅助速度按履带目标速度 1:1 跟随 */
  static constexpr float TRACK_WHEEL_ASSIST_SCALE = 1.0f;
  /* 履带 FOLLOW 纠偏只允许使用普通最大角速度的 35% */
  static constexpr float TRACK_FOLLOW_OMEGA_LIMIT_SCALE = 0.35f;
  /* 履带目标线速度最大变化率, 1.2 表示每秒最多变化 1.2 m/s */
  static constexpr float TRACK_SPEED_RAMP_MPS2 = 1.2f;
  /* 履带主动轮半径 m */
  static constexpr float TRACK_WHEEL_RADIUS_M = 0.025f;
  static constexpr float TRACK_ACTIVE_SPEED_EPS_MPS = 0.05f;
  static constexpr float TRACK_PRIORITY_ERROR_EPS_MPS = 0.12f;
  static constexpr float TRACK_STALL_SPEED_RATIO = 0.35f;
  static constexpr float TRACK_WHEEL_FREE_SPIN_OMEGA_RADPS = 18.0f;
  static constexpr float TRACK_CRUISE_RESERVE_FRACTION = 0.55f;
  static constexpr float TRACK_PRIORITY_RESERVE_FRACTION = 0.8f;
  static constexpr float TRACK_LATERAL_SPEED_SCALE = 0.85f;
  static constexpr float TRACK_WHEEL_ASSIST_PRIORITY_SCALE = 0.45f;
  static constexpr float TRACK_WHEEL_DEPRIORITY_SCALE = 0.1f;
  static constexpr float TRACK_CRUISE_WEIGHT_SCALE = 1.35f;
  static constexpr float TRACK_PRIORITY_WEIGHT_SCALE = 2.5f;
  static constexpr float WHEEL_COUNT_FLOAT = 4.0f;

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
  float rotor_dynamic_scale_ = 1.0f; /* 功率相关动态缩放 */

  float current_yaw_ = 0.0f;
  float yawmotor_zero_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;
  Motor* track_motor_;

  Motor* motor_wheel_[4]{motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                         motor_wheel_3_};
  Motor::Feedback motor_feedback_[4]{};
  Motor::MotorCmd motor_cmd_[4]{};
  MotorData motor_data_{};

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param()),
      LibXR::PID<float>(LibXR::PID<float>::Param())};

  LibXR::PID<float> pid_track_speed_;

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

  float track_linear_speed_ = 0.0f;
  float track_target_speed_ = 0.0f;
  float track_speed_error_ = 0.0f;
  float track_output_current_ = 0.0f;
  Motor::Feedback track_motor_feedback_{};
  Motor::MotorCmd track_motor_cmd_{};

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  PowerControl* power_control_;
  PowerControlData power_control_data_;

  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  Referee::ChassisPack referee_chassis_pack_{};

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  ChassisMode chassis_event_ = ChassisMode::RELAX;
  Referee* referee_;
};
