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
#include <cstdint>

#include "CMD.hpp"
#include "Chassis.hpp"
#include "PowerControl.hpp"
#include "RMMotor.hpp"
#include "app_framework.hpp"
#include "event.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_time.hpp"
#include "message.hpp"
#include "pid.hpp"

#define OMNI_MOTOR_MAX_OMEGA 52 /* 电机输出轴最大角速度 */
#define OMNI_CHASSIS_MAX_POWER 60.0f  /* 底盘最大功率 */

template <typename ChassisType>
class Chassis;

class Omni {
 public:
  struct ChassisParam {
    float wheel_radius = 0.065f;
    float wheel_to_center = 0.26f;
    float gravity_height = 0.0f;
    float reductionratio = 15.746f;  // 减速比
    float wheel_resistance = 0.0f;    // 轮子阻力
    float error_compensation = 0.0f;  // 误差补偿
    float gravity = 0.0f;
  };
  enum class Chassismode : uint8_t {
    RELAX,
    INDEPENDENT,
    ROTOR,
    FOLLOW_GIMBAL_INTERSECT,
    FOLLOW_GIMBAL_CROSS,
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
  Omni(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app,
       RMMotor *motor_wheel_0, RMMotor *motor_wheel_1, RMMotor *motor_wheel_2,
       RMMotor *motor_wheel_3, RMMotor *motor_steer_0, RMMotor *motor_steer_1,
       RMMotor *motor_steer_2, RMMotor *motor_steer_3, CMD *cmd,
       PowerControl *power_control, uint32_t task_stack_depth,
       ChassisParam chassis_param, LibXR::PID<float>::Param pid_follow,
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
        motor_wheel_0_(motor_wheel_0),   /*wheel0   ▲ y  wheel3*/
        motor_wheel_1_(motor_wheel_1),   /*    ↙    │     ↖    */
        motor_wheel_2_(motor_wheel_2),   /*         │          */
        motor_wheel_3_(motor_wheel_3),   /* ――――――――│―――――――▶x */
        pid_follow_(pid_follow),         /*         │          */
        pid_velocity_x_(pid_velocity_x), /*    ➘    │     ➚    */
        pid_velocity_y_(pid_velocity_y), /*wheel1   │    wheel2*/
        pid_omega_(pid_omega),
        pid_wheel_omega_{pid_wheel_omega_0, pid_wheel_omega_1,
                         pid_wheel_omega_2, pid_wheel_omega_3},
        pid_steer_angle_{pid_steer_angle_0, pid_steer_angle_1,
                         pid_steer_angle_2, pid_steer_angle_3},
        pid_steer_speed_{pid_steer_speed_0, pid_steer_speed_1,
                         pid_steer_speed_2, pid_steer_speed_3},
        cmd_(cmd),
        power_control_(power_control),
        cmd_file_(InitCmdFile()) {
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
    thread_.Create(this, ThreadFunction, "OmniChassisThread", task_stack_depth,
                   LibXR::Thread::Priority::HIGH);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    auto lost_ctrl_callback = LibXR::Callback<uint32_t>::Create(
        [](bool in_isr, Omni *omni, uint32_t event_id) {
          UNUSED(in_isr);
          UNUSED(event_id);
          omni->LostCtrl();
        },
        this);
    cmd_->GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL, lost_ctrl_callback);
  }

  /**
   * @brief 全向轮底盘控制线程函数
   * @param omni Omni对象指针
   * @details 控制线程主循环，负责接收控制指令、执行运动学解算和动力学控制输出
   */
  static void ThreadFunction(Omni *omni) {
    omni->mutex_.Lock();
    auto last_time = LibXR::Timebase::GetMilliseconds();

    LibXR::Topic::ASyncSubscriber<CMD::ChassisCMD> cmd_suber("chassis_cmd");
    LibXR::Topic::ASyncSubscriber<LibXR::EulerAngle<float>> euler_suber(
        "ahrs_euler");
    LibXR::Topic::ASyncSubscriber<float> chassis_yaw("chassis_yaw");

    cmd_suber.StartWaiting();
    euler_suber.StartWaiting();
    chassis_yaw.StartWaiting();

    omni->mutex_.Unlock();

    while (true) {
      if (cmd_suber.Available()) {
        omni->cmd_data_ = cmd_suber.GetData();
        cmd_suber.StartWaiting();
      }

      if (euler_suber.Available()) {
        omni->euler_ = euler_suber.GetData();
        euler_suber.StartWaiting();
        omni->current_pitch_ = omni->euler_.Pitch();
        omni->current_roll_ = omni->euler_.Roll();
        omni->current_yaw_ = omni->euler_.Yaw();
      }

      if (chassis_yaw.Available()) {
        omni->chassis_yaw_ = chassis_yaw.GetData();
        chassis_yaw.StartWaiting();
      }

      omni->mutex_.Lock();
      omni->Update();
      omni->UpdateCMD();
      omni->SelfResolution();
      omni->InverseKinematicsSolution();
      omni->DynamicInverseSolution();
      omni->CalculateMotorCurrent();
      omni->PowerControlUpdate();
      omni->mutex_.Unlock();
      omni->OutputToDynamics();
      omni->thread_.SleepUntil(last_time,2);
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

    switch (chassis_event_) {
      /*放松模式 不输出*/
      case static_cast<uint32_t>(Chassismode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        target_omega_ = 0.0f;
        break;

        /*独立模式*/
      case static_cast<uint32_t>(Chassismode::INDEPENDENT):
        target_omega_ = PARAM.wheel_radius * OMNI_MOTOR_MAX_OMEGA *
                        cmd_data_.z / PARAM.wheel_to_center;
        break;

        /*小陀螺*/
      case static_cast<uint32_t>(Chassismode::ROTOR):
        target_omega_ = -static_cast<float>(
            PARAM.wheel_radius * OMNI_MOTOR_MAX_OMEGA / PARAM.wheel_to_center);
        break;

        /*正方向跟随云台*/
      case static_cast<uint32_t>(Chassismode::FOLLOW_GIMBAL_INTERSECT):
        target_omega_ = -pid_follow_.Calculate(0.0f, chassis_yaw_, dt_);
        break;

      /*十字跟随云台*/
      case static_cast<uint32_t>(Chassismode::FOLLOW_GIMBAL_CROSS):
        target_omega_ = -pid_follow_.Calculate(0.0f, static_cast<float>(chassis_yaw_ - M_PI / 4.0f), dt_);
        break;

      default:
        break;
    }

    /* 计算vx,vy */
    switch (chassis_event_) {
      case static_cast<uint32_t>(Chassismode::RELAX):
        target_vx_ = 0.0f;
        target_vy_ = 0.0f;
        break;
      case static_cast<uint32_t>(Chassismode::ROTOR):
      case static_cast<uint32_t>(Chassismode::FOLLOW_GIMBAL_CROSS):
      case static_cast<uint32_t>(Chassismode::FOLLOW_GIMBAL_INTERSECT):{
        float beta = chassis_yaw_;
        float cos_beta = cosf(beta);
        float sin_beta = sinf(beta);
        target_vx_ = (cos_beta * cmd_data_.x * PARAM.wheel_radius *
                          OMNI_MOTOR_MAX_OMEGA -
                      sin_beta * cmd_data_.y * PARAM.wheel_radius *
                          OMNI_MOTOR_MAX_OMEGA);
        target_vy_ = (sin_beta * cmd_data_.x * PARAM.wheel_radius *
                          OMNI_MOTOR_MAX_OMEGA +
                      cos_beta * cmd_data_.y * PARAM.wheel_radius *
                          OMNI_MOTOR_MAX_OMEGA);
    }
    break;
      case static_cast<uint32_t>(Chassismode::INDEPENDENT): {
        const float SQRT2 = 1.41421356237f;
        float s = fabsf(cmd_data_.x) + fabsf(cmd_data_.y);  // 适配菱形速度
        float k = (s <= 1.0f) ? (PARAM.wheel_radius * OMNI_MOTOR_MAX_OMEGA)
                              : ((PARAM.wheel_radius * OMNI_MOTOR_MAX_OMEGA) / s);
        target_vx_ = SQRT2 * k * cmd_data_.x;
        target_vy_ = SQRT2 * k * cmd_data_.y;
      } break;
      default:
        break;
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

  /**
   * @brief 前馈
   *
   */
  void FeedForward() {
  //TODO: 这里后期需要改成相对角度
    float k = M_PI / 100;

    gx_ff_ = PARAM.gravity *
             SoftDeadzone(sin(current_pitch_) * cos(current_roll_), sin(k));
    gy_ff_ = PARAM.gravity * SoftDeadzone(sin(current_roll_), sin(k));

    const float SQRT2_2 = 0.70710678118f;
    baseff_[0] = (gx_ff_ - gy_ff_) * SQRT2_2 / 2;
    baseff_[1] = (gx_ff_ + gy_ff_) * SQRT2_2 / 2;
    baseff_[2] = (-gx_ff_ + gy_ff_) * SQRT2_2 / 2;
    baseff_[3] = (-gx_ff_ - gy_ff_) * SQRT2_2 / 2;

    torque_ff_[0] = baseff_[0] * PARAM.wheel_radius;
    torque_ff_[1] = baseff_[1] * PARAM.wheel_radius;
    torque_ff_[2] = baseff_[2] * PARAM.wheel_radius;
    torque_ff_[3] = baseff_[3] * PARAM.wheel_radius;
  }

  /**
   * @brief 全向轮底盘正运动学解算
   * @details 根据四个全向轮的角速度，解算出底盘当前的运动状态
   */
  void SelfResolution() {
    const float SQRT2 = 1.41421356237f;
    now_vx_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio -
               motor_wheel_1_->GetOmega() / PARAM.reductionratio -
               motor_wheel_2_->GetOmega() / PARAM.reductionratio +
               motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_vy_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio +
               motor_wheel_1_->GetOmega() / PARAM.reductionratio -
               motor_wheel_2_->GetOmega() / PARAM.reductionratio -
               motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
              SQRT2 * PARAM.wheel_radius / 4.0f;

    now_omega_ = (motor_wheel_0_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_1_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_2_->GetOmega() / PARAM.reductionratio +
                  motor_wheel_3_->GetOmega() / PARAM.reductionratio) *
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
    if (chassis_event_ == static_cast<uint32_t>(Chassismode::RELAX)) {
      LostCtrl();
    } else {
      // 计算速度环 PID
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

      // 计算原始输出 (PID + 前馈力矩)
      for (int i = 0; i < 4; i++) {
        output_[i] = target_motor_force_[i] * PARAM.wheel_radius+
        target_motor_current_[i];
      }
    }
  }

  /**
   * @brief 功率控制更新
   */
  void PowerControlUpdate() {
    /*扭矩电流转换为命令电流的比率*/
    float ratio_3508 =
        motor_wheel_0_->GetLSB() / motor_wheel_0_->GetCurrentMAX();

    RMMotor *motor_wheels[4] = {motor_wheel_0_, motor_wheel_1_, motor_wheel_2_,
                                motor_wheel_3_};

    for (int i = 0; i < 4; i++) {
      motor_data_.rotorspeed_rpm_3508[i] = motor_wheels[i]->GetRPM();
      motor_data_.output_current_3508[i] =
          motor_wheels[i]->GetCurrent() * ratio_3508;
    }

    /* 设置3508电机数据 */
    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->CalculatePowerControlParam();

    /*将扭矩转换为电流的比率*/
    float ratio_current =
        (motor_wheel_0_->GetLSB() / PARAM.reductionratio /
         motor_wheel_0_->KGetTorque() / motor_wheel_0_->GetCurrentMAX());

    for (int i = 0; i < 4; i++) {
      motor_data_.output_current_3508[i] =
          std::clamp(output_[i] * ratio_current, -16384.0f, 16384.0f);
    }

    power_control_->SetMotorData3508(motor_data_.output_current_3508,
                                     motor_data_.rotorspeed_rpm_3508);

    power_control_->OutputLimit(OMNI_CHASSIS_MAX_POWER);
    power_control_data_ = power_control_->GetPowerControlData();
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
                           (motor_wheel_0_->GetLSB() / PARAM.reductionratio /
                            motor_wheel_0_->KGetTorque() /
                            motor_wheel_0_->GetCurrentMAX()),
                       -6.0f, 6.0f);
      }
    }
    if(chassis_event_==static_cast<uint32_t>(Chassismode::RELAX)){
      LostCtrl();
      return;
    }
    else {
    motor_wheel_0_->TorqueControl(output_[0], PARAM.reductionratio);
    motor_wheel_1_->TorqueControl(output_[1], PARAM.reductionratio);
    motor_wheel_2_->TorqueControl(output_[2], PARAM.reductionratio);
    motor_wheel_3_->TorqueControl(output_[3], PARAM.reductionratio);
    }

  }
/**
 * @brief 失去控制时的处理
 *
 */
  void LostCtrl() {
    motor_wheel_0_->Relax();
    motor_wheel_1_->Relax();
    motor_wheel_2_->Relax();
    motor_wheel_3_->Relax();
  }

  static int CommandFunc(Omni *self, int argc, char **argv) {
    if (argc == 1) {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf(
          "  monitor                          - Print chassis motor feedback "
          "once\r\n");
      LibXR::STDIO::Printf(
          "  monitor <time_ms> [interval_ms]  - Monitor chassis motor "
          "feedback\r\n");
      return 0;
    }

    if (strcmp(argv[1], "monitor") != 0) {
      LibXR::STDIO::Printf("Error: Unknown command '%s'.\r\n", argv[1]);
      return -1;
    }

    auto print_feedback = [self]() {
      LibXR::STDIO::Printf("[%lu ms] Chassis Motors Feedback:\r\n",
                           LibXR::Thread::GetTime());
      LibXR::STDIO::Printf(
          "  Motor 0 - Angle: %.2f rad, RPM: %.0f, Current: %.2f A, Temp: %.0f "
          "C\r\n",
          self->motor_wheel_0_->GetAngle(), self->motor_wheel_0_->GetRPM(),
          self->motor_wheel_0_->GetCurrent(), self->motor_wheel_0_->GetTemp());
      self->thread_.Sleep(1);
      LibXR::STDIO::Printf(
          "  Motor 1 - Angle: %.2f rad, RPM: %.0f, Current: %.2f A, Temp: %.0f "
          "C\r\n",
          self->motor_wheel_1_->GetAngle(), self->motor_wheel_1_->GetRPM(),
          self->motor_wheel_1_->GetCurrent(), self->motor_wheel_1_->GetTemp());
      self->thread_.Sleep(1);
      LibXR::STDIO::Printf(
          "  Motor 2 - Angle: %.2f rad, RPM: %.0f, Current: %.2f A, Temp: %.0f "
          "C\r\n",
          self->motor_wheel_2_->GetAngle(), self->motor_wheel_2_->GetRPM(),
          self->motor_wheel_2_->GetCurrent(), self->motor_wheel_2_->GetTemp());
      self->thread_.Sleep(1);
      LibXR::STDIO::Printf(
          "  Motor 3 - Angle: %.2f rad, RPM: %.0f, Current: %.2f A, Temp: %.0f "
          "C\r\n",
          self->motor_wheel_3_->GetAngle(), self->motor_wheel_3_->GetRPM(),
          self->motor_wheel_3_->GetCurrent(), self->motor_wheel_3_->GetTemp());
      self->thread_.Sleep(1);
      LibXR::STDIO::Printf(
          "  Chassis Now State - VX: %.2f, VY: %.2f, Omega: %.2f\r\n",
          self->now_vx_, self->now_vy_, self->now_omega_);
      LibXR::STDIO::Printf(
          "  Chassis Target State - VX: %.2f, VY: %.2f, Omega: %.2f\r\n",
          self->target_vx_, self->target_vy_, self->target_omega_);
      self->thread_.Sleep(1);
    };

    if (argc == 2) {
      print_feedback();
    } else if (argc >= 3) {
      int time = atoi(argv[2]);
      int delay = (argc == 4) ? atoi(argv[3]) : 1000;
      while (time > 0) {
        print_feedback();
        LibXR::Thread::Sleep(delay);
        time -= delay;
      }
    } else {
      LibXR::STDIO::Printf("Error: Invalid arguments.\r\n");
      return -1;
    }

    return 0;
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

  float current_yaw_ = 0.0f;
  float current_roll_ = 0.0;
  float current_pitch_ = 0.0;
  float chassis_yaw_ = 0.0f;
  float torque_ff_[4]{0.0, 0.0, 0.0, 0.0};
  float baseff_[4]{0.0, 0.0, 0.0, 0.0};

  float gx_ff_;
  float gy_ff_;
  float dt_ = 0;
  LibXR::MicrosecondTimestamp last_online_time_ = 0;

  RMMotor *motor_wheel_0_;
  RMMotor *motor_wheel_1_;
  RMMotor *motor_wheel_2_;
  RMMotor *motor_wheel_3_;

  LibXR::PID<float> pid_follow_;
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
  LibXR::Thread thread_;
  LibXR::Mutex mutex_;

  CMD *cmd_;
  CMD::ChassisCMD cmd_data_;

  MotorData motor_data_ = {};
  PowerControl *power_control_;
  PowerControlData power_control_data_;
  LibXR::EulerAngle<float> euler_;
  uint32_t chassis_event_ = 0;

  LibXR::RamFS::File cmd_file_;
  char *cmd_name_;

  LibXR::RamFS::File InitCmdFile() {
    const char *prefix = "omni_chassis";
    size_t prefix_len = strlen(prefix);

    cmd_name_ = new char[prefix_len + 1];
    memcpy(cmd_name_, prefix, prefix_len);
    cmd_name_[prefix_len] = '\0';

    return LibXR::RamFS::CreateFile(cmd_name_, CommandFunc, this);
  }
};
