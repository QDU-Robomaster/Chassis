#pragma once

#ifndef MECANUM_CHASSIS_DEBUG_IMPL
#include "Mecanum.hpp"
#endif

inline int Mecanum::DebugCommand(int argc, char** argv) {
  enum class DebugView : uint8_t { STATE, CMD, MOTION, POWER, WHEEL, FULL };

  constexpr uint8_t view_state = static_cast<uint8_t>(DebugView::STATE);
  constexpr uint8_t view_cmd = static_cast<uint8_t>(DebugView::CMD);
  constexpr uint8_t view_motion = static_cast<uint8_t>(DebugView::MOTION);
  constexpr uint8_t view_power = static_cast<uint8_t>(DebugView::POWER);
  constexpr uint8_t view_wheel = static_cast<uint8_t>(DebugView::WHEEL);
  constexpr uint8_t view_full = static_cast<uint8_t>(DebugView::FULL);

  constexpr debug_core::ViewMask mask_state = debug_core::view_bit(view_state);
  constexpr debug_core::ViewMask mask_cmd = debug_core::view_bit(view_cmd);
  constexpr debug_core::ViewMask mask_motion =
      debug_core::view_bit(view_motion);
  constexpr debug_core::ViewMask mask_power = debug_core::view_bit(view_power);
  constexpr debug_core::ViewMask mask_wheel = debug_core::view_bit(view_wheel);

  static constexpr std::array<debug_core::ViewEntry<uint8_t>, 6> view_table{{
      {"state", view_state},
      {"cmd", view_cmd},
      {"motion", view_motion},
      {"power", view_power},
      {"wheel", view_wheel},
      {"full", view_full},
  }};

  static const debug_core::LiveFieldDesc<Mecanum> fields[] = {
      DEBUG_CORE_LIVE_CUSTOM(
          Mecanum, "mode", mask_state,
          +[](const char* field_name, const Mecanum* self) {
            const char* text = "UNKNOWN";
            switch (self->chassis_event_) {
              case ChassisMode::RELAX:
                text = "RELAX";
                break;
              case ChassisMode::INDEPENDENT:
                text = "INDEPENDENT";
                break;
              case ChassisMode::ROTOR:
                text = "ROTOR";
                break;
              case ChassisMode::FOLLOW:
                text = "FOLLOW";
                break;
            }
            LibXR::STDIO::Printf("  %s=%s\r\n", field_name, text);
          }),
      DEBUG_CORE_LIVE_CUSTOM(
          Mecanum, "ctrl_mode", mask_state,
          +[](const char* field_name, const Mecanum* self) {
            const char* text = "N/A";
            if (self->cmd_ != nullptr) {
              text = "UNKNOWN";
              switch (self->cmd_->GetCtrlMode()) {
                case CMD::Mode::CMD_OP_CTRL:
                  text = "CMD_OP_CTRL";
                  break;
                case CMD::Mode::CMD_AUTO_CTRL:
                  text = "CMD_AUTO_CTRL";
                  break;
              }
            }
            LibXR::STDIO::Printf("  %s=%s\r\n", field_name, text);
          }),
      DEBUG_CORE_LIVE_BOOL(Mecanum, "cmd_online", mask_state,
                           self->cmd_ != nullptr && self->cmd_->Online()),
      DEBUG_CORE_LIVE_F32(Mecanum, "dt_s", mask_state, self->dt_),
      DEBUG_CORE_LIVE_F32(Mecanum, "yaw", mask_state, self->current_yaw_),
      DEBUG_CORE_LIVE_F32(Mecanum, "cmd_x", mask_cmd, self->cmd_data_.x),
      DEBUG_CORE_LIVE_F32(Mecanum, "cmd_y", mask_cmd, self->cmd_data_.y),
      DEBUG_CORE_LIVE_F32(Mecanum, "cmd_z", mask_cmd, self->cmd_data_.z),
      DEBUG_CORE_LIVE_F32(Mecanum, "target_vx", mask_motion, self->target_vx_),
      DEBUG_CORE_LIVE_F32(Mecanum, "target_vy", mask_motion, self->target_vy_),
      DEBUG_CORE_LIVE_F32(Mecanum, "target_omega", mask_motion,
                          self->target_omega_),
      DEBUG_CORE_LIVE_F32(Mecanum, "now_vx", mask_motion, self->now_vx_),
      DEBUG_CORE_LIVE_F32(Mecanum, "now_vy", mask_motion, self->now_vy_),
      DEBUG_CORE_LIVE_F32(Mecanum, "now_omega", mask_motion, self->now_omega_),
      DEBUG_CORE_LIVE_CUSTOM(
          Mecanum, "wheel_motion", mask_motion,
          +[](const char* field_name, const Mecanum* self) {
            LibXR::STDIO::Printf("  %s:\r\n", field_name);
            for (int i = 0; i < 4; ++i) {
              float now_omega =
                  self->motor_feedback_[i].omega / self->PARAM.reduction_ratio;
              float omega_err = self->target_motor_omega_[i] - now_omega;
              LibXR::STDIO::Printf(
                  "    m%d: omega_t=%.3f omega_n=%.3f err=%.3f i_pid=%.3f "
                  "f_dyn=%.3f out=%.3f\r\n",
                  i, self->target_motor_omega_[i], now_omega, omega_err,
                  self->target_motor_current_[i], self->target_motor_force_[i],
                  self->output_[i]);
            }
          }),
      DEBUG_CORE_LIVE_BOOL(Mecanum, "power_limited", mask_power,
                           self->power_control_data_.is_power_limited),
      DEBUG_CORE_LIVE_CUSTOM(
          Mecanum, "power_current", mask_power,
          +[](const char* field_name, const Mecanum* self) {
            LibXR::STDIO::Printf("  %s:\r\n", field_name);
            for (int i = 0; i < 4; ++i) {
              float req_i = self->motor_data_.output_current_3508[i];
              float lim_i =
                  self->power_control_data_.is_power_limited
                      ? self->power_control_data_.new_output_current_3508[i]
                      : req_i;
              LibXR::STDIO::Printf(
                  "    m%d: req_i=%.1f lim_i=%.1f cmd_tq=%.3f "
                  "out_clamped=%d\r\n",
                  i, req_i, lim_i, self->motor_cmd_[i].torque,
                  fabsf(self->output_[i]) >= 6.0f ? 1 : 0);
            }
          }),
      DEBUG_CORE_LIVE_CUSTOM(
          Mecanum, "wheel_feedback", mask_wheel,
          +[](const char* field_name, const Mecanum* self) {
            LibXR::STDIO::Printf("  %s:\r\n", field_name);
            for (int i = 0; i < 4; ++i) {
              const auto& fb = self->motor_feedback_[i];
              LibXR::STDIO::Printf(
                  "    m%d: state=%u abs=%.3f rpm=%.0f omega=%.3f torque=%.3f "
                  "temp=%.0f\r\n",
                  i, static_cast<unsigned>(fb.state),
                  static_cast<float>(fb.abs_angle), fb.velocity, fb.omega,
                  fb.torque, fb.temp);
            }
          }),
  };

  auto lock_self = +[](Mecanum* self) { self->mutex_.Lock(); };
  auto unlock_self = +[](Mecanum* self) { self->mutex_.Unlock(); };

  return debug_core::run_live_command(
      this, "mecanum_chassis", "state|cmd|motion|power|wheel|full", view_table,
      fields, sizeof(fields) / sizeof(fields[0]), argc, argv, view_full,
      lock_self, unlock_self);
}
