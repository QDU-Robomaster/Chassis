#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: 舵轮底盘控制实现，由 Chassis 模板使用 / Helm (swerve) chassis controller used by the Chassis template
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "Motor.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "Referee.hpp"
#include "cycle_value.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "pid.hpp"
#include "thread.hpp"
#include "timebase.hpp"

#define M3508_NM_TO_LSB_RATIO 52437.5f /* 3508转子扭矩转化为电机控制单位的比例 */
#define GM6020_NM_TO_LSB_RATIO 7370.0f /* 6020转子扭矩转化为电机控制单位的比例 */

#define HELM_CHASSIS_MAX_POWER 100

template <typename ChassisType>
class Chassis;

/**
 * @brief 舵轮底盘控制实现，由 Chassis 模板使用。
 *        Helm (swerve) chassis controller used by the Chassis template.
 */
class Helm
{
 public:
  /**
   * @brief 底盘几何、动力与小陀螺缩放参数。
   *        Chassis geometry, dynamics and spin-scaling parameters.
   */
  struct ChassisParam
  {
    float wheel_radius = 0.0f;  ///< 轮半径 (m)
    ///< Wheel radius (m)
    float wheel_to_center = 0.0f;  ///< 轮心到底盘中心的距离 (m)
    ///< Distance from the wheel center to the chassis center (m)
    float gravity_height = 0.0f;  ///< 质心高度 (m)，用于全向轮姿态前馈
    ///< Center-of-mass height (m), used by the Omni attitude feedforward
    float reduction_ratio = 0.0f;  ///< 轮电机减速比
    ///< Reduction ratio of the wheel motors
    float wheel_resistance = 0.0f;  ///< 轮阻
    ///< Wheel resistance
    float error_compensation = 0.0f;  ///< 误差补偿
    ///< Error compensation
    float gravity = 0.0f;  ///< 底盘重力 (N)，用于全向轮姿态前馈
    ///< Chassis weight (N), used by the Omni attitude feedforward
    float length = 0.0f;  ///< 轮距的长 (m)，用于全向轮姿态前馈
    ///< Length of the wheel layout (m), used by the Omni attitude feedforward
    float width = 0.0f;  ///< 轮距的宽 (m)，用于全向轮姿态前馈
    ///< Width of the wheel layout (m), used by the Omni attitude feedforward
    float rotor_speed_scale = 1.0f;  ///< 平移输入下的小陀螺转速缩放比例
    ///< Spin speed scale under translation input
    float rotor_omega_min_scale = 0.55f;  ///< 小陀螺动态缩放的下限
    ///< Lower bound of the dynamic spin scale
    float rotor_buffer_low_j = 35.0f;  ///< 缓冲能量低阈值 (J)
    ///< Low buffer energy threshold (J)
    float rotor_buffer_high_j = 70.0f;  ///< 缓冲能量高阈值 (J)
    ///< High buffer energy threshold (J)
    float rotor_scale_lpf_alpha = 0.2f;  ///< 动态缩放的一阶低通系数
    ///< First-order low-pass coefficient of the dynamic scale
  };
  /**
   * @brief 底盘模式。
   *        Chassis modes.
   */
  enum class ChassisMode : uint8_t
  {
    RELAX,  ///< 放松：全部电机放松
    ///< Relax: all motors relaxed
    INDEPENDENT,  ///< 独立：平移相对底盘坐标系
    ///< Independent: translation in the chassis frame
    ROTOR,  ///< 小陀螺：底盘旋转，平移相对云台坐标系
    ///< Spin: the chassis rotates, translation in the gimbal frame
    FOLLOW  ///< 跟随：底盘跟随云台 yaw，平移相对云台坐标系
    ///< Follow: the chassis follows the gimbal yaw, translation in the gimbal frame
  };

  /**
   * @brief 构造舵轮底盘控制对象，创建控制线程。
   *        Construct the helm chassis controller and create the control thread.
   *
   * @param motor_wheel_0 第 0 个驱动轮电机。
   *                      Drive wheel motor 0.
   * @param motor_wheel_1 第 1 个驱动轮电机。
   *                      Drive wheel motor 1.
   * @param motor_wheel_2 第 2 个驱动轮电机。
   *                      Drive wheel motor 2.
   * @param motor_wheel_3 第 3 个驱动轮电机。
   *                      Drive wheel motor 3.
   * @param motor_steer_0 第 0 个舵向电机，非空。
   *                      Steering motor 0, non-null.
   * @param motor_steer_1 第 1 个舵向电机，非空。
   *                      Steering motor 1, non-null.
   * @param motor_steer_2 第 2 个舵向电机，非空。
   *                      Steering motor 2, non-null.
   * @param motor_steer_3 第 3 个舵向电机，非空。
   *                      Steering motor 3, non-null.
   * @param cmd 控制命令模块实例。
   *            Control command Module instance.
   * @param power_control 功率控制模块实例。
   *                      Power control Module instance.
   * @param referee 裁判系统模块实例。
   *                Referee Module instance.
   * @param task_stack_depth 控制线程栈深。
   *                         Control thread stack depth.
   * @param chassis_param 底盘几何与动力参数。
   *                      Chassis geometry and dynamics parameters.
   * @param pid_follow 跟随云台的角度 PID 参数。
   *                   PID parameters of the gimbal-following angle loop.
   * @param pid_velocity_x x 方向速度 PID 参数。
   *                       PID parameters of the x velocity loop.
   * @param pid_velocity_y y 方向速度 PID 参数。
   *                       PID parameters of the y velocity loop.
   * @param pid_omega 角速度 PID 参数。
   *                  PID parameters of the angular-velocity loop.
   * @param pid_wheel_speed_0 轮 0 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 0.
   * @param pid_wheel_speed_1 轮 1 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 1.
   * @param pid_wheel_speed_2 轮 2 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 2.
   * @param pid_wheel_speed_3 轮 3 速度 PID 参数。
   *                          PID parameters of the speed loop of wheel 3.
   * @param pid_steer_angle_0 舵 0 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 0.
   * @param pid_steer_angle_1 舵 1 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 1.
   * @param pid_steer_angle_2 舵 2 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 2.
   * @param pid_steer_angle_3 舵 3 角度 PID 参数。
   *                          PID parameters of the angle loop of steering motor 3.
   * @param pid_steer_speed_0 舵 0 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 0.
   * @param pid_steer_speed_1 舵 1 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 1.
   * @param pid_steer_speed_2 舵 2 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 2.
   * @param pid_steer_speed_3 舵 3 速度 PID 参数。
   *                          PID parameters of the speed loop of steering motor 3.
   * @param thread_priority 控制线程优先级。
   *                        Control thread priority.
   * @param topic_names 订阅的 Topic 名称。
   *                    Names of the subscribed Topics.
   */
  Helm(Motor* motor_wheel_0, Motor* motor_wheel_1, Motor* motor_wheel_2,
       Motor* motor_wheel_3, Motor* motor_steer_0, Motor* motor_steer_1,
       Motor* motor_steer_2, Motor* motor_steer_3, CMD* cmd, PowerControl* power_control,
       Referee* referee, uint32_t task_stack_depth, ChassisParam chassis_param,
       LibXR::PID<float>::Param pid_follow, LibXR::PID<float>::Param pid_velocity_x,
       LibXR::PID<float>::Param pid_velocity_y, LibXR::PID<float>::Param pid_omega,
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
       LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::HIGH,
       ChassisTopicNames topic_names = {})
      : PARAM(chassis_param),
        motor_wheel_0_(motor_wheel_0),
        motor_wheel_1_(motor_wheel_1),
        motor_wheel_2_(motor_wheel_2),
        motor_wheel_3_(motor_wheel_3),
        motor_steer_0_(motor_steer_0),   /* wheel3   ▲ y  wheel2 */
        motor_steer_1_(motor_steer_1),   /*     ↑    │     ↑     */
        motor_steer_2_(motor_steer_2),   /*          │           */
        motor_steer_3_(motor_steer_3),   /* ─────────┼────────▶x */
        pid_follow_(pid_follow),         /*          │           */
        pid_velocity_x_(pid_velocity_x), /*     ↑    │     ↑     */
        pid_velocity_y_(pid_velocity_y), /* wheel0   │    wheel1 */
        pid_omega_(pid_omega),
        pid_wheel_speed_{pid_wheel_speed_0, pid_wheel_speed_1, pid_wheel_speed_2,
                         pid_wheel_speed_3},
        pid_steer_angle_{
            pid_steer_angle_0,
            pid_steer_angle_1,
            pid_steer_angle_2,
            pid_steer_angle_3,
        },
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1, pid_steer_speed_2,
                         pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control)
  {
    topic_names_ = topic_names;
    UNUSED(referee);

    /* 舵轮底盘控制线程无条件访问四个舵向电机 */
    ASSERT(motor_steer_0 != nullptr);
    ASSERT(motor_steer_1 != nullptr);
    ASSERT(motor_steer_2 != nullptr);
    ASSERT(motor_steer_3 != nullptr);

    for (int i = 0; i < 4; i++)
    {
      motor_wheel_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_wheel_cmd_[i].reduction_ratio = chassis_param.reduction_ratio;
      motor_wheel_cmd_[i].torque = 0.0f;
      motor_wheel_cmd_[i].position = 0.0f;
      motor_wheel_cmd_[i].velocity = 0.0f;
      motor_wheel_cmd_[i].kp = 0.0f;
      motor_wheel_cmd_[i].kd = 0.0f;
    }
    for (int i = 0; i < 4; i++)
    {
      motor_steer_cmd_[i].mode = Motor::ControlMode::MODE_TORQUE;
      motor_steer_cmd_[i].reduction_ratio = 1.0f;
      motor_steer_cmd_[i].torque = 0.0f;
      motor_steer_cmd_[i].position = 0.0f;
      motor_steer_cmd_[i].velocity = 0.0f;
      motor_steer_cmd_[i].kp = 0.0f;
      motor_steer_cmd_[i].kd = 0.0f;
    }

    thread_.Create(this, ThreadFunction, "HelmChassisThread", task_stack_depth,
                   thread_priority);
    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Helm* helm, uint32_t event_id)
        {
          UNUSED(in_isr);
          UNUSED(event_id);
          helm->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 舵轮底盘控制线程函数。
   *        Control thread function of the helm chassis.
   *
   * @details 线程主循环：接收控制命令，执行运动学解算并输出控制量。
   *          Main loop: receives the control command, solves the kinematics and sends the
   *          outputs.
   *
   * @param helm Helm 对象指针。
   *             Pointer to the Helm object.
   */
  static void ThreadFunction(Helm* helm)
  {
    helm->mutex_.Lock();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber(helm->topic_names_.chassis_cmd);
    LibXR::Topic::ASyncSubscriber<Referee::ChassisPack> referee_suber(
        helm->topic_names_.chassis_ref);
    LibXR::Topic::ASyncSubscriber<float> yawmotor_angle_suber("yawmotor_angle");

    cmd_suber.StartWaiting();
    referee_suber.StartWaiting();
    yawmotor_angle_suber.StartWaiting();

    helm->last_online_time_ = LibXR::Timebase::GetMicroseconds();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    helm->mutex_.Unlock();
    while (true)
    {
      if (cmd_suber.Available())
      {
        helm->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (referee_suber.Available())
      {
        helm->referee_chassis_pack_ = referee_suber.GetData();
        helm->referee_last_rx_time_ = LibXR::Timebase::GetMilliseconds();
        referee_suber.StartWaiting();
      }

      if (yawmotor_angle_suber.Available())
      {
        helm->current_yaw_ = LibXR::CycleValue<float>(yawmotor_angle_suber.GetData());
        helm->delta_yaw_ = -helm->current_yaw_;
        yawmotor_angle_suber.StartWaiting();
      }

      helm->mutex_.Lock();
      helm->Update();
      helm->UpdateCMD();
      helm->topic_delta_yaw_.Publish(helm->delta_yaw_);
      helm->Helmcontrol();
      helm->PowerControlUpdate();
      helm->mutex_.Unlock();
      helm->Output();
      helm->thread_.SleepUntil(last_time, 2);
    }
  }
  /**
   * @brief 更新电机状态。
   *        Update the motor states.
   *
   * @details 记录采样间隔，更新四个驱动轮电机与四个舵向电机并读取反馈。
   *          Records the sampling interval, updates the four wheel motors and the four
   *          steering motors and reads their feedback.
   */
  void Update()
  {
    auto now = LibXR::Timebase::GetMicroseconds();
    dt_ = (now - last_online_time_).ToSecondf();
    last_online_time_ = now;

    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Update();
      motor_steer_[i]->Update();
      motor_wheel_feedback_[i] = motor_wheel_[i]->GetFeedback();
      motor_steer_feedback_[i] = motor_steer_[i]->GetFeedback();
    }
  }

  /**
   * @brief 失去控制时使全部电机放松。
   *        Relax all motors when control is lost.
   */
  void LostCtrl()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_wheel_[i]->Relax();
      motor_steer_[i]->Relax();
    }
  }

  /**
   * @brief 设置底盘模式并复位速度、轮速与舵向 PID，由 Chassis 外壳调用。
   *        Set the chassis mode and reset the velocity, wheel and steering PIDs; called
   *        by the Chassis shell.
   *
   * @param mode 新模式，取 ChassisMode 的值。
   *             New mode, a value of ChassisMode.
   */
  void SetMode(uint32_t mode)
  {
    mutex_.Lock();
    chassis_event_ = static_cast<ChassisMode>(mode);
    pid_omega_.Reset();
    pid_velocity_x_.Reset();
    pid_velocity_y_.Reset();
    for (int i = 0; i < 4; i++)
    {
      pid_wheel_speed_[i].Reset();
      pid_steer_angle_[i].Reset();
      pid_steer_speed_[i].Reset();
    }
    mutex_.Unlock();
  }

  /**
   * @brief 从 CMD 命令更新目标速度。
   *        Update the target velocities from the CMD command.
   */
  void UpdateCMD()
  {
    target_vx_ = cmd_data_.x;
    target_vy_ = cmd_data_.y;
    target_omega_ = cmd_data_.z;
  }

  /**
   * @brief 更新功率控制：提交反馈与期望输出，计算功率上限并读取限幅结果。
   *        Update power control: submit the feedback and the requested outputs, compute
   *        the power limit and read the limited result.
   */
  void PowerControlUpdate()
  {
    for (int i = 0; i < 4; i++)
    {
      motor_data_.rotorspeed_rpm_3508[i] = motor_wheel_feedback_[i].velocity;
      motor_data_.output_current_3508[i] =
          motor_wheel_feedback_[i].torque * M3508_NM_TO_LSB_RATIO;

      motor_data_.rotorspeed_rpm_6020[i] = motor_steer_feedback_[i].velocity;
      motor_data_.output_current_6020[i] =
          motor_steer_feedback_[i].torque * GM6020_NM_TO_LSB_RATIO;
    }

    /* 第一次设置: 反馈数据用于 RLS 参数估计 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020);

    power_control_->CalculatePowerControlParam();

    /* 计算 3508 速度跟踪误差 */
    float speed_error_3508[4];
    for (int i = 0; i < 4; i++)
    {
      float actual_speed = motor_reverse_[i] ? -motor_wheel_feedback_[i].velocity
                                             : motor_wheel_feedback_[i].velocity;
      speed_error_3508[i] = target_speed_[i] - actual_speed;

      motor_data_.output_current_3508[i] =
          std::clamp(wheel_out_[i] * M3508_NM_TO_LSB_RATIO / PARAM.reduction_ratio,
                     -16384.0f, 16384.0f);
      motor_data_.output_current_6020[i] =
          std::clamp(steer_out_[i] * GM6020_NM_TO_LSB_RATIO, -16384.0f, 16384.0f);
    }

    /* 计算 6020 舵向速度跟踪误差 */
    float speed_error_6020[4];
    for (int i = 0; i < 4; i++)
    {
      speed_error_6020[i] = steer_angle_[i] - motor_steer_feedback_[i].velocity *
                                                  static_cast<float>(LibXR::TWO_PI) /
                                                  60.0f;
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508, speed_error_3508);
    power_control_->SetMotorData6020(motor_data_.output_current_6020,
                                     motor_data_.rotorspeed_rpm_6020, speed_error_6020);

    auto now_ms = LibXR::Timebase::GetMilliseconds();
    bool referee_online = (now_ms - referee_last_rx_time_).ToSecondf() <= 1.0f;
    bool power_control_online = power_control_->IsOnline();
    bool boost_mode = (cmd_data_.self_define == CMD::ChasStat::BOOST);

    float max_power = static_cast<float>(referee_chassis_pack_.rs.chassis_power_limit);
    if (!referee_online || max_power <= 1.0f)
    {
      max_power = HELM_CHASSIS_MAX_POWER;
    }

    if (power_control_online && boost_mode)
    {
      float cap_energy = power_control_->GetCapEnergy();
      if (cap_energy > 0.8f)
      {
        max_power += 100.0f;
      }
      else if (cap_energy > 0.5f)
      {
        max_power += 70.0f;
      }
      else if (cap_energy > 0.25f)
      {
        max_power += 40.0f;
      }
    }

    power_control_->OutputLimit(max_power);
    power_control_data_ = power_control_->GetPowerControlData();
  }

  /**
   * @brief 按模式计算目标速度、舵角，以及轮与舵向电机的输出。
   *        Compute the target velocity and steering angles for the current mode, and the
   *        outputs of the wheel and steering motors.
   */
  void Helmcontrol()
  {
    /* 计算目标平移速度 */
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case (ChassisMode::INDEPENDENT):
      {
        target_vx_ = cmd_data_.x;
        target_vy_ = cmd_data_.y;
      }
      break;
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR):
      {
        float beta = current_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = cos_beta * cmd_data_.x - sin_beta * cmd_data_.y;
        target_vy_ = sin_beta * cmd_data_.x + cos_beta * cmd_data_.y;
      }
      break;

      default:
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
    }

    const float SQRT2 = 1.41421356237f;
    /* 计算 wz */
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        target_omega_ = 0.0f;
        break;
      case (ChassisMode::INDEPENDENT):
        /* 独立模式每个轮子的方向相同，wz当作轮子转向角速度 */
        target_omega_ = -cmd_data_.z;
        break;
      case (ChassisMode::FOLLOW):
        target_omega_ = pid_follow_.Calculate(0.0f, current_yaw_, dt_);
        break;
      case (ChassisMode::ROTOR):
        /* 陀螺模式底盘以一定速度旋转 */
        target_omega_ = -cmd_data_.z;
        break;
      default:
        target_omega_ = 0.0f;
        break;
    }
    switch (chassis_event_)
    {
      case (ChassisMode::RELAX):
        for (int i = 0; i < 4; i++)
        {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0;
        }
        break;
      case (ChassisMode::INDEPENDENT):
      case (ChassisMode::FOLLOW):
      case (ChassisMode::ROTOR):
      {
        float x = 0, y = 0, wheel_pos = 0;
        for (int i = 0; i < 4; i++)
        {
          wheel_pos = -static_cast<float>(i) * static_cast<float>(M_PI_2) +
                      static_cast<float>(M_PI) / 4.0f * 3.0f;
          x = -sinf(wheel_pos) * target_omega_ * SQRT2 + target_vx_;
          y = -cosf(wheel_pos) * target_omega_ * SQRT2 + target_vy_;

          if (fabsf(x) < 0.05f && fabsf(y) < 0.05f)
          {
            target_angle_[i] = wheel_pos;
            target_speed_[i] = 0.0f;
          }
          else
          {
            target_angle_[i] = M_PI_2 - atan2f(y, x);
            target_speed_[i] = motor_max_speed_ * (fmaxf(fabsf(x), fabsf(y)));
          }
        }
      }
      break;
      default:
        for (int i = 0; i < 4; i++)
        {
          target_speed_[i] = 0.0f;
          target_angle_[i] = 0.0;
        }
        break;
    }

    /* 舵向差超过 90 度时反转轮电机, 减少舵向转动 */
    for (int i = 0; i < 4; i++)
    {
      if (fabs(LibXR::CycleValue(motor_steer_feedback_[i].abs_angle - zero_[i]) -
               target_angle_[i]) > M_PI_2)
      {
        motor_reverse_[i] = true;
      }
      else
      {
        motor_reverse_[i] = false;
      }
    }

    /* 根据反转状态计算轮电机和舵向电机输出 */
    for (int i = 0; i < 4; i++)
    {
      if (motor_reverse_[i])
      {
        wheel_out_[i] = pid_wheel_speed_[i].Calculate(
            -target_speed_[i], motor_wheel_feedback_[i].velocity, dt_);

        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            LibXR::CycleValue<float>(target_angle_[i] + static_cast<float>(M_PI) +
                                     zero_[i]),
            motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity * static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
      else
      {
        wheel_out_[i] = pid_wheel_speed_[i].Calculate(
            target_speed_[i], motor_wheel_feedback_[i].velocity, dt_);
        steer_angle_[i] = pid_steer_angle_[i].Calculate(
            target_angle_[i] + zero_[i], motor_steer_feedback_[i].abs_angle, dt_);
        steer_out_[i] = pid_steer_speed_[i].Calculate(
            steer_angle_[i],
            motor_steer_feedback_[i].velocity * static_cast<float>(LibXR::TWO_PI) / 60.0f,
            dt_);
      }
    }
  }

  /**
   * @brief 向电机下发控制命令；RELAX 模式下使全部电机放松。
   *        Send the control commands to the motors; all motors are relaxed in RELAX mode.
   */
  void Output()
  {
    if (power_control_data_.is_power_limited)
    {
      for (int i = 0; i < 4; i++)
      {
        wheel_out_[i] = std::clamp(power_control_data_.new_output_current_3508[i] /
                                       M3508_NM_TO_LSB_RATIO * PARAM.reduction_ratio,
                                   -4.4f, 4.4f);
        steer_out_[i] = static_cast<float>(std ::clamp(
            power_control_data_.new_output_current_6020[i] / GM6020_NM_TO_LSB_RATIO * 1.0,
            -2.5, 2.5));
      }
    }

    if (chassis_event_ == (ChassisMode::RELAX))
    {
      LostCtrl();
    }
    else
    {
      for (int i = 0; i < 4; i++)
      {
        motor_wheel_cmd_[i].torque = std::clamp(wheel_out_[i], -4.4f, 4.4f);
        motor_steer_cmd_[i].torque = std::clamp(steer_out_[i], -2.5f, 2.5f);
      }
      for (int i = 0; i < 4; i++)
      {
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

  bool motor_reverse_[4]{false, false, false, false};
  LibXR::CycleValue<float> zero_[4] = {1.10063124, 2.05323339, 3.2244277, 0.979446769};
  float current_yaw_ = 0.0f;
  float delta_yaw_ = 0.0f;

  /* 目标轮速与目标舵角 */
  float target_speed_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  LibXR::CycleValue<float> target_angle_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  /* 轮与舵向电机的输出 */
  float wheel_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_out_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float steer_angle_[4] = {0.0, 0.0, 0.0, 0.0};

  float motor_max_speed_ = 9000.0;

  LibXR::CycleValue<float> main_direct_ = 0.0f;

  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  Motor* motor_wheel_0_;
  Motor* motor_wheel_1_;
  Motor* motor_wheel_2_;
  Motor* motor_wheel_3_;
  Motor* motor_steer_0_;
  Motor* motor_steer_1_;
  Motor* motor_steer_2_;
  Motor* motor_steer_3_;

  Motor* motor_wheel_[4] = {motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                            motor_wheel_3_};
  Motor* motor_steer_[4] = {motor_steer_0_, motor_steer_1_, motor_steer_2_,
                            motor_steer_3_};

  Motor::Feedback motor_wheel_feedback_[4]{};
  Motor::Feedback motor_steer_feedback_[4]{};

  LibXR::PID<float> pid_follow_;
  LibXR::PID<float> pid_velocity_x_;
  LibXR::PID<float> pid_velocity_y_;
  LibXR::PID<float> pid_omega_;

  LibXR::PID<float> pid_wheel_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_angle_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};
  LibXR::PID<float> pid_steer_speed_[4] = {LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param()),
                                           LibXR::PID<float>(LibXR::PID<float>::Param())};

  CMD* cmd_;
  CMD::ChassisCMD cmd_data_;

  Motor::MotorCmd motor_wheel_cmd_[4]{};
  Motor::MotorCmd motor_steer_cmd_[4]{};

  MotorData motor_data_ = {};
  PowerControl* power_control_;
  PowerControlData power_control_data_;

  Referee::ChassisPack referee_chassis_pack_{};
  LibXR::MillisecondTimestamp referee_last_rx_time_ = 0;
  ChassisTopicNames topic_names_{};
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  LibXR::Topic topic_delta_yaw_ = LibXR::Topic::CreateTopic<float>("delta_yaw");
  ChassisMode chassis_event_ = ChassisMode::RELAX;
};
