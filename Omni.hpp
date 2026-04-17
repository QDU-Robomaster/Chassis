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
#include <cstdio>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "app_framework.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"
#include "timebase.hpp"
#include "timer.hpp"

#ifdef DEBUG
#include "DebugCore.hpp"
#endif

#define M3508_NM_TO_LSB_RATIO \
  52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */

#define OMNI_MOTOR_MAX_OMEGA 52 /* 电机输出轴最大角速度 */

#define OMNI_CHASSIS_MAX_POWER 100

template <typename ChassisType>
class Chassis;

class Omni {
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
    float rotor_speed_scale =1.0f; /* 小陀螺转速缩放比例，降低可给平移留出更多功率 */
    float rotor_omega_min_scale = 0.55f; /* 功率受限时小陀螺最小转速比例 */
    float rotor_buffer_low_j = 35.0f;    /* 缓冲能量低阈值，单位 J */
    float rotor_buffer_high_j = 70.0f;   /* 缓冲能量高阈值，单位 J */
    float rotor_scale_lpf_alpha = 0.2f;  /* 动态缩放一阶低通系数 */
  };
  enum class ChassisMode : uint8_t {
    RELAX,
    INDEPENDENT,
    ROTOR,
    FOLLOW,
  };
  struct Eulerangle {
    LibXR::EulerAngle<float> roll;
    LibXR::EulerAngle<float> pitch;
    LibXR::EulerAngle<float> yaw;
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
  Omni(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
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
       LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH)
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),   /*wheel0   ▲ y  wheel3*/
        motor_wheel_1_(motor_wheel_1),   /*    ↙    │     ↖    */
        motor_wheel_2_(motor_wheel_2),   /*         │          */
        motor_wheel_3_(motor_wheel_3),   /* ――――――――│―――――――▶x */
        pid_follow_(pid_follow),         /*         │          */
        pid_velocity_x_(pid_velocity_x), /*    ➘    │     ➚    */
        pid_velocity_y_(pid_velocity_y), /*wheel1   │    wheel2*/
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1,
                         pid_wheel_speed_2, pid_wheel_speed_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3},
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1,
                         pid_steer_speed_2, pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control),
        referee_(referee)
#ifdef DEBUG
        ,
        cmd_file_(LibXR::RamFS::CreateFile(
            "omni_chassis",
            debug_core::command_thunk<Omni, &Omni::DebugCommand>, this))
#endif
  {
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

    for (int i = 0; i < 4; i++) {
      motor_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_cmd_[i].torque = 0.0f;
      motor_cmd_[i].position = 0.0f;
      motor_cmd_[i].velocity = 0.0f;
      motor_cmd_[i].kp = 0.0f;
      motor_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   thread_priority);
    if (referee_ != nullptr) {
      timer_static_ = LibXR::Timer::CreateTask(DrawUI, this, 300);
      LibXR::Timer::Add(timer_static_);
      LibXR::Timer::Start(timer_static_);
    }
#ifdef DEBUG
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);
#endif

    auto start_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni* omni, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->chassis_event_ = ChassisMode::RELAX;
        },
        this);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni* omni, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->chassis_event_ = ChassisMode::RELAX;
        },
        this);

    cmd_->GetEvent().Register(CMD::CMD_EVENT_START_CTRL, start_ctrl_callback);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 全向轮底盘控制线程函数
   * @param omni Omni对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Omni* omni) {
    omni->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        "chassis_ref");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        "ahrs_euler");
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");
    LibXR::Topic::ASyncSubscriber<float> pitchmotor_angle_suber("pitchmotor_angle");
    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    euler_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();
    pitchmotor_angle_suber.StartWaiting();
    omni->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();
    omni->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        omni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available()) {
        omni->referee_chassis_pack_ = referee_suber.GetData();
        omni->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (euler_suber.Available()) {
        omni->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
        omni->imu_pitch_ = -(omni->euler_.Pitch());
        omni->imu_roll_ = -(omni->euler_.Roll());
        omni->imu_yaw_ = omni->euler_.Yaw();
      }

      if (yawmotor_angle_suber.Available()) {
        omni->yawmotor_angle_ = yawmotor_angle_suber.GetData();
        yawmotor_angle_suber.StartWaiting();
      }
      if (pitchmotor_angle_suber.Available()) {
        omni->pitchmotor_angle_ = pitchmotor_angle_suber.GetData();
        pitchmotor_angle_suber.StartWaiting();
      }

      omni->mutex_.Lock();
      omni->Update();
      omni->UpdateCMD();
      omni->SelfResolution();
      omni->InverseKinematicsSolution();
      omni->DynamicInverseSolution();
      omni->FeedForward();
      omni->CalculateMotorCurrent();
      omni->PowerControlUpdate();
      omni->mutex_.Unlock();
      omni->OutputToDynamics();

      omni->thread_.SleepUntil(last_time, 2);
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
    }
    mutex_.Unlock();
  }

  /**
   * @brief 更新底盘控制指令状态
   * @details 从CMD获取底盘控制指令，并转换为目标速度
   */
  void UpdateCMD() {
    float max_v = PARAM.wheel_radius * OMNI_MOTOR_MAX_OMEGA;

    /*计算omega*/
    switch (chassis_event_) {
      case ChassisMode::RELAX:
        target_omega_ = 0.0f;
        break;

      case ChassisMode::INDEPENDENT:
        target_omega_ = max_v * cmd_data_.z / PARAM.wheel_to_center;
        break;

      case ChassisMode::ROTOR:
        target_omega_ = -static_cast<float>(max_v / PARAM.wheel_to_center);
        break;

        /*正方向跟随云台*/
      case ChassisMode::FOLLOW:
        target_omega_ = -pid_follow_.Calculate(0.0f, yawmotor_angle_, dt_);
        break;

      default:
        break;
    }

    /* 计算vx,vy */
    switch (chassis_event_) {
      case ChassisMode::RELAX:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case ChassisMode::ROTOR:
      case ChassisMode::FOLLOW: {
        float beta = yawmotor_angle_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ =
            (cos_beta * cmd_data_.x * max_v - sin_beta * cmd_data_.y * max_v);
        target_vy_ =
            (sin_beta * cmd_data_.x * max_v + cos_beta * cmd_data_.y * max_v);
      } break;
      case ChassisMode::INDEPENDENT: {
        const float SQRT2 = 1.41421356237f;
        /* 适配菱形速度 */
        float s = fabsf(cmd_data_.x) + fabsf(cmd_data_.y);
        float k = (s <= 1.0f) ? max_v : (max_v / s);
        target_vx_ = SQRT2 * k * cmd_data_.x;
        target_vy_ = SQRT2 * k * cmd_data_.y;
      } break;
      default:
        break;
    }

    /* 小陀螺模式下，根据平移输入与功率状态动态调整旋转速度 */
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
   * @brief 交给上坡前馈的软限位
   *
   * @param x
   * @param dz
   * @return float
   */
  float SoftDeadzone(float x, float dz) {
    if (fabs(x) < dz) {
      return 0.0f;
    } else {
      return (fabs(x) - dz) * (x > 0.0f ? 1.0f : -1.0f);
    }
  }

  void FeedForward() {
    // ===== Step0: wrap =====
    auto WrapToPi = [](float a) {
      while (a > M_PI) a -= 2.0f * M_PI;
      while (a < -M_PI) a += 2.0f * M_PI;
      return a;
    };

    float yaw_g = WrapToPi(imu_yaw_);
    float pitch_g = WrapToPi(imu_pitch_);
    float roll_g = WrapToPi(imu_roll_);

    float yaw_m = WrapToPi(yawmotor_angle_);
    float pitch_m = WrapToPi(pitchmotor_angle_);

    // ===== 三角函数 =====
    float cy = cosf(yaw_g), sy = sinf(yaw_g);
    float cp = cosf(pitch_g), sp = sinf(pitch_g);
    float cr = cosf(roll_g), sr = sinf(roll_g);

    float cy_m = cosf(yaw_m), sy_m = sinf(yaw_m);
    float cp_m = cosf(pitch_m), sp_m = sinf(pitch_m);

    // ===== R_wg = Rz * Ry * Rx =====
    float R_wg[3][3];

    R_wg[0][0] = cy * cp;
    R_wg[0][1] = cy * sp * sr - sy * cr;
    R_wg[0][2] = cy * sp * cr + sy * sr;

    R_wg[1][0] = sy * cp;
    R_wg[1][1] = sy * sp * sr + cy * cr;
    R_wg[1][2] = sy * sp * cr - cy * sr;

    R_wg[2][0] = -sp;
    R_wg[2][1] = cp * sr;
    R_wg[2][2] = cp * cr;

    // ===== R_cg = Rz * Ry =====
    float R_cg[3][3];

    R_cg[0][0] = cy_m * cp_m;
    R_cg[0][1] = -sy_m;
    R_cg[0][2] = cy_m * sp_m;

    R_cg[1][0] = sy_m * cp_m;
    R_cg[1][1] = cy_m;
    R_cg[1][2] = sy_m * sp_m;

    R_cg[2][0] = -sp_m;
    R_cg[2][1] = 0.0f;
    R_cg[2][2] = cp_m;

    // ===== R_gc = 转置 =====
    float R_gc[3][3];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R_gc[i][j] = R_cg[j][i];
      }
    }

    // ===== R_wc = R_wg * R_gc =====
    float R_wc[3][3] = {{0}};

    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R_wc[i][j] = R_wg[i][0] * R_gc[0][j] + R_wg[i][1] * R_gc[1][j] +
                     R_wg[i][2] * R_gc[2][j];
      }
    }

    // ===== 提取底盘姿态 =====
    float val = -R_wc[2][0];
    if (val > 1.0f) val = 1.0f;
    if (val < -1.0f) val = -1.0f;

    chassis_pitch_ = asinf(val);
    chassis_roll_ = atan2f(R_wc[2][1], R_wc[2][2]);

    chassis_pitch_ = WrapToPi(chassis_pitch_);
    chassis_roll_ = WrapToPi(chassis_roll_);

    float k = M_PI / 50;
    gy_ff_ = -PARAM.gravity * SoftDeadzone(sinf(chassis_pitch_), sinf(k));
    gx_ff_ = -PARAM.gravity * SoftDeadzone(sinf(chassis_roll_), sinf(k));

    py = -PARAM.gravity_height * SoftDeadzone(sinf(chassis_pitch_), sin(k));
    px = -PARAM.gravity_height * SoftDeadzone(sinf(chassis_roll_), sin(k));

    for (int i = 0; i < 4; i++) {
      float dx = post_x_[i] - px;
      float dy = post_y_[i] - py;
      length_[i] = sqrtf(dx * dx + dy * dy);
    }

    const float SQRT2 = 0.70710678118f * 2;
    baseff_[0] = (gx_ff_ + gy_ff_) * SQRT2;
    baseff_[1] = (-gx_ff_ + gy_ff_) * SQRT2;
    baseff_[2] = (-gx_ff_ - gy_ff_) * SQRT2;
    baseff_[3] = (gx_ff_ - gy_ff_) * SQRT2;

    for (int i = 0; i < 4; i++) {
      baseff_l_[i] = 1.0f / length_[i];
      torque_n_[i] = baseff_l_[i] / (baseff_l_[0] + baseff_l_[1] +
                                     baseff_l_[2] + baseff_l_[3]);
      baseff_[i] = baseff_[i] * torque_n_[i];
    }

    for (int i = 0; i < 4; i++) {
      torque_ff_[i] = baseff_[i] * PARAM.wheel_radius;
    }
  }

  /**
   * @brief 全向轮底盘正运动学解算
   * @details 根据四个全向轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    const float SQRT2 = 1.41421356237f;

    now_vx_ = -(motor_feedback_[0].omega / PARAM.reduction_ratio -
                motor_feedback_[1].omega / PARAM.reduction_ratio -
                motor_feedback_[2].omega / PARAM.reduction_ratio +
                motor_feedback_[3].omega / PARAM.reduction_ratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_vy_ = -(motor_feedback_[0].omega / PARAM.reduction_ratio +
                motor_feedback_[1].omega / PARAM.reduction_ratio -
                motor_feedback_[2].omega / PARAM.reduction_ratio -
                motor_feedback_[3].omega / PARAM.reduction_ratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_feedback_[0].omega / PARAM.reduction_ratio +
                  motor_feedback_[1].omega / PARAM.reduction_ratio +
                  motor_feedback_[2].omega / PARAM.reduction_ratio +
                  motor_feedback_[3].omega / PARAM.reduction_ratio) *
                 PARAM.wheel_radius / (4.0f * PARAM.wheel_to_center);
  }

  /**
   * @brief 全向轮底盘逆运动学解算
   * @details 根据目标底盘速度（vx, vy, ω），计算四个全向轮的目标角速度
   */
  void InverseKinematicsSolution() {
    const float SQRT1 = 0.70710678118f;

    target_motor_omega_[0] = (-SQRT1 * target_vx_ - SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[1] = (SQRT1 * target_vx_ - SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[2] = (SQRT1 * target_vx_ + SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
    target_motor_omega_[3] = (-SQRT1 * target_vx_ + SQRT1 * target_vy_ +
                              target_omega_ * PARAM.wheel_to_center) /
                             PARAM.wheel_radius;
  }

  /**
   * @brief 计算 PID 输出电流
   */
  void CalculateMotorCurrent() {
    if (chassis_event_ == ChassisMode::RELAX) {
      LostCtrl();
    } else {
      /*目标角速度*/
      for (int i = 0; i < 4; i++) {
        target_motor_current_[i] = pid_wheel_speed_[i].Calculate(
            target_motor_omega_[i],
            motor_feedback_[i].omega / PARAM.reduction_ratio, dt_);
      }

      /*计算输出*/
      for (int i = 0; i < 4; i++) {
        output_[i] = target_motor_force_[i] * PARAM.wheel_radius +
                     target_motor_current_[i]+torque_ff_[i];
      }
    }
  }

  /**
   * @brief 功率控制更新
   */
  void PowerControlUpdate() {
    /* 采样当前反馈电流和转速，供功率模型参数估计使用。 */
    for (int i = 0; i < 4; i++) {
      motor_data_.rotorspeed_rpm_3508[i] = motor_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);
    power_control_->CalculatePowerControlParam();

    float speed_error[4];
    for (int i = 0; i < 4; i++) {
      speed_error[i] = target_motor_omega_[i] -
                       motor_feedback_[i].omega / PARAM.reduction_ratio;
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508,
                                     speed_error);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    /* 裁判系统离线或上限异常时，回退到本地默认功率上限。 */
    float max_power =
        static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f) {
      max_power = OMNI_CHASSIS_MAX_POWER;
    }

    /* BOOST 模式按电容能量分档提升可用功率上限。 */
    if (power_control_online && boost_mode) {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f) {
        max_power += 100.0f;
      } else if (cap_energy > 0.5f) {
        max_power += 60.0f;
      } else if (cap_energy > 0.25f) {
        max_power += 40.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();

    /* 受限程度估计：限幅后电流总量 / 请求电流总量。 */
    float req_current_abs_sum = 0.0f;
    float lim_current_abs_sum = 0.0f;
    for (int i = 0; i < 4; i++) {
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

    /* 将裁判缓冲能量映射为缩放因子；离线时保持 1.0。 */
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

    /* 合成目标缩放并做一阶低通，抑制跳变。 */
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

    /* 仅在小陀螺模式保留缩放，其他模式统一复位。 */
    if (chassis_event_ != ChassisMode::ROTOR) {
      rotor_dynamic_scale_ = 1.0f;
    }
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
    float force_z = pid_omega_.Calculate(target_omega_, now_omega_, dt_);

    /* 分配（最小范数 / 平均分配 torque）//切向合力=质量*角加速度*转动半径 */
    target_motor_force_[0] = (-SQRT2 * force_x - SQRT2 * force_y + force_z) / 4;
    target_motor_force_[1] = (SQRT2 * force_x - SQRT2 * force_y + force_z) / 4;
    target_motor_force_[2] = (SQRT2 * force_x + SQRT2 * force_y + force_z) / 4;
    target_motor_force_[3] = (-SQRT2 * force_x + SQRT2 * force_y + force_z) / 4;
  }

  /**
   * @brief 全向轮底盘动力学输出
   * @details 限幅并输出四个全向轮的电流控制指令
   */
  void OutputToDynamics() {
    /*如果超功率了output根据功率的数值来计算*/
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
   * @brief 失去控制时的处理
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
  // UI 参数约定：
  // - SCREEN_W/H: 客户端坐标系分辨率（用于计算中心点和绝对位置）
  // - LAYER_CHASSIS: 底盘 UI 专用图层
  // - ORBIT_*: 车头方位点绕屏幕中心旋转的轨迹半径/点半径
  // - DEFAULT_WIDTH/CHAR_WIDTH: 图形线宽与字符线宽
  static constexpr uint16_t UI_SCREEN_W = 1920;
  static constexpr uint16_t UI_SCREEN_H = 1080;
  static constexpr uint16_t UI_LAYER_CHASSIS = 0;
  static constexpr uint16_t UI_ORBIT_RADIUS = 260;
  static constexpr uint16_t UI_ORBIT_DOT_RADIUS = 20;
  static constexpr uint16_t UI_DEFAULT_WIDTH = 1;
  static constexpr uint16_t UI_CHAR_WIDTH = 2;

  // 裁判系统客户端 ID 与机器人 ID 的映射关系。
  static uint16_t GetClientID(uint16_t robot_id) {
    /* 蓝方 */
    if (robot_id > 100) {
      return static_cast<uint16_t>(robot_id - 101 + 0x0165);
    }
    /* 红方 */
    return static_cast<uint16_t>(robot_id + 0x0100);
  }

  // 图元名固定为 3 字节，不足补空格，便于后续按名称 MODIFY。
  static void SetFigureName(uint8_t (&dst)[3], const char* name) {
    dst[0] = ' ';
    dst[1] = ' ';
    dst[2] = ' ';
    if (name == nullptr) {
      return;
    }
    for (int i = 0; i < 3 && name[i] != '\0'; ++i) {
      dst[i] = static_cast<uint8_t>(name[i]);
    }
  }

  static void FillCharacter(Referee::UICharacter& fig, const char* name,
                            Referee::UIFigureOp op, uint8_t layer,
                            Referee::UIColor color, uint16_t font_size,
                            uint16_t width, uint16_t x, uint16_t y,
                            const char* text) {
    // 每次都从空结构体开始，避免复用旧数据导致字段污染。
    fig = {};
    SetFigureName(fig.grapic_data_struct.figure_name, name);
    fig.grapic_data_struct.operate_type = static_cast<uint32_t>(op);
    fig.grapic_data_struct.figure_type =
        static_cast<uint32_t>(Referee::UIFigureType::UI_TYPE_CHAR);
    fig.grapic_data_struct.layer = layer;
    fig.grapic_data_struct.color = static_cast<uint32_t>(color);
    fig.grapic_data_struct.details_a = font_size;

    uint16_t text_len = 0;
    if (text != nullptr) {
      // 文本长度受 fig.data 缓冲区限制，防止越界。
      text_len = static_cast<uint16_t>(strnlen(text, sizeof(fig.data)));
      memcpy(fig.data, text, text_len);
    }
    fig.grapic_data_struct.details_b = text_len;
    fig.grapic_data_struct.width = width;
    fig.grapic_data_struct.start_x = x;
    fig.grapic_data_struct.start_y = y;
  }

  static void FillCircle(Referee::UIFigure& fig, const char* name,
                         Referee::UIFigureOp op, uint8_t layer,
                         Referee::UIColor color, uint16_t width, uint16_t x,
                         uint16_t y, uint16_t radius) {
    // 圆形图元统一通过此函数填充，确保字段含义一致。
    fig = {};
    SetFigureName(fig.figure_name, name);
    fig.operate_type = static_cast<uint32_t>(op);
    fig.figure_type =
        static_cast<uint32_t>(Referee::UIFigureType::UI_TYPE_CIRCLE);
    fig.layer = layer;
    fig.color = static_cast<uint32_t>(color);
    fig.width = width;
    fig.start_x = x;
    fig.start_y = y;
    fig.details_c = radius;
  }

  /* 定时器回调：仅保留底盘模式与车头方位圆 */
  static void DrawUI(Omni* omni) {
    if (omni->referee_ == nullptr) return;
    const uint16_t id = omni->referee_chassis_pack_.rs.robot_id;
    if (id == 0) return;
    const uint16_t client = GetClientID(id);

    if (!omni->ui_layer_cleared_) {
      // 上电后先清一次目标图层，确保本模块接管该层时画面可预期。
      Referee::UILayerDelete ui_del{};
      ui_del.delete_type =
          static_cast<uint8_t>(Referee::UIDeleteType::UI_DELETE_LAYER);
      ui_del.layer = UI_LAYER_CHASSIS;
      const ErrorCode EC =
          omni->referee_->SendUILayerDelete(id, client, ui_del);
      if (EC != ErrorCode::OK) {
        return;
      }

      // Layer 删除后需要重新 ADD 当前保留图元，避免只发 MODIFY。
      omni->ui_initialized_ = false;
      omni->ui_dot_initialized_ = false;
      omni->ui_layer_cleared_ = true;
    }

    // 将 2 类 UI 内容分 2 帧发送，降低单帧通信负载。
    omni->ui_tick_++;

    switch (omni->ui_tick_ % 2) {
      case 0: {
        // 文本模式显示：底盘模式字符串。
        omni->mutex_.Lock();
        const ChassisMode mode = omni->chassis_event_;
        omni->mutex_.Unlock();
        const char* mode_str = "RELX";
        switch (mode) {
          case ChassisMode::RELAX:
            mode_str = "RELX";
            break;
          case ChassisMode::INDEPENDENT:
            mode_str = "INDP";
            break;
          case ChassisMode::ROTOR:
            mode_str = "ROTO";
            break;
          case ChassisMode::FOLLOW:
            mode_str = "FOLW";
            break;
          default:
            break;
        }
        // 首次发送用 ADD，后续同名图元用 MODIFY 做增量更新。
        const Referee::UIFigureOp OP = omni->ui_initialized_
                                           ? Referee::UIFigureOp::UI_OP_MODIFY
                                           : Referee::UIFigureOp::UI_OP_ADD;
        Referee::UICharacter char_fig{};
        FillCharacter(char_fig, "CM", OP, UI_LAYER_CHASSIS,
                      Referee::UIColor::UI_COLOR_CYAN, 20, UI_CHAR_WIDTH, 160,
                      580, mode_str);
        if (omni->referee_->SendUICharacter(id, client, char_fig) ==
            ErrorCode::OK) {
          omni->ui_initialized_ = true;
        }
        break;
      }
      case 1: {
        // 车头方位点：以屏幕中心为圆心，按 chassis_yaw_ 投影到圆周。
        const Referee::UIFigureOp OP = omni->ui_dot_initialized_
                                           ? Referee::UIFigureOp::UI_OP_MODIFY
                                           : Referee::UIFigureOp::UI_OP_ADD;
        const uint16_t DOT_X = static_cast<uint16_t>(
            UI_SCREEN_W / 2 + UI_ORBIT_RADIUS * sinf(omni->chassis_yaw_));
        const uint16_t DOT_Y = static_cast<uint16_t>(
            UI_SCREEN_H / 2 + UI_ORBIT_RADIUS * cosf(omni->chassis_yaw_));
        Referee::UIFigure fig{};
        FillCircle(fig, "CS", OP, UI_LAYER_CHASSIS,
                   Referee::UIColor::UI_COLOR_CYAN, UI_DEFAULT_WIDTH * 7, DOT_X,
                   DOT_Y, UI_ORBIT_DOT_RADIUS);
        if (omni->referee_->SendUIFigure(id, client, fig) == ErrorCode::OK) {
          omni->ui_dot_initialized_ = true;
        }
        break;
      }
    }
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
  float rotor_dynamic_scale_ = 1.0f; /* 功率相关动态缩放 */

  float chassis_yaw_ = 0.0f;
  float dt_ = 0;

  float imu_yaw_ = 0.0f;
  float imu_roll_ = 0.0f;
  float imu_pitch_ = 0.0f;
  float yawmotor_angle_ = 0.0f;
  float pitchmotor_angle_ = 0.0f;

  float chassis_roll_ = 0.0f;
  float chassis_pitch_ = 0.0f;

  float torque_ff_[4]{0.0, 0.0, 0.0, 0.0};
  float baseff_[4]{0.0, 0.0, 0.0, 0.0};
  float gx_ff_;
  float gy_ff_;


  float length_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float torque_n_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float baseff_l_[4]{0.0f, 0.0f, 0.0f, 0.0f};
  float px = 0.0f;
  float py = 0.0f;
  float post_x_[4] = {-PARAM.width / 2, -PARAM.width / 2, PARAM.width / 2,
                      PARAM.width / 2};
  float post_y_[4] = {PARAM.length / 2, -PARAM.length / 2, -PARAM.length / 2,
                      PARAM.length / 2};

  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;

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

  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  PowerControl* power_control_;
  PowerControlData power_control_data_;

  Referee* referee_;
  Referee::ChassisPack referee_chassis_pack_{};
  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;

  LibXR::EulerAngle<float> euler_;
  ChassisMode chassis_event_ = ChassisMode::RELAX;

  bool ui_initialized_ = false;
  bool ui_dot_initialized_ = false;
  bool ui_layer_cleared_ = false;
  uint32_t ui_tick_ = 0;
  LibXR::Timer::TimerHandle timer_static_;

#ifdef DEBUG
  LibXR::RamFS::File cmd_file_;
#endif
};

#ifdef DEBUG
#define OMNI_CHASSIS_DEBUG_IMPL
#include "OmniDebug.inl"
#undef OMNI_CHASSIS_DEBUG_IMPL
#endif
