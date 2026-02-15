#pragma once

#ifndef HELM_CHASSIS_DEBUG_IMPL
#include "Helm.hpp"
#endif

inline int Helm::DebugCommand(int argc, char** argv) {
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

  static const debug_core::LiveFieldDesc<Helm> fields[] = {
      DEBUG_CORE_LIVE_CUSTOM(
          Helm, "mode", mask_state,
          +[](const char* field_name, const Helm* self) {
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
          Helm, "ctrl_mode", mask_state,
          +[](const char* field_name, const Helm* self) {
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
      DEBUG_CORE_LIVE_BOOL(Helm, "cmd_online", mask_state,
                           self->cmd_ != nullptr && self->cmd_->Online()),
      DEBUG_CORE_LIVE_F32(Helm, "dt_s", mask_state, self->dt_),
      DEBUG_CORE_LIVE_F32(Helm, "yaw", mask_state, self->current_yaw_),
      DEBUG_CORE_LIVE_F32(Helm, "target_vx", mask_state, self->target_vx_),
      DEBUG_CORE_LIVE_F32(Helm, "target_vy", mask_state, self->target_vy_),
      DEBUG_CORE_LIVE_F32(Helm, "target_omega", mask_state,
                          self->target_omega_),
      DEBUG_CORE_LIVE_F32(Helm, "cmd_x", mask_cmd, self->cmd_data_.x),
      DEBUG_CORE_LIVE_F32(Helm, "cmd_y", mask_cmd, self->cmd_data_.y),
      DEBUG_CORE_LIVE_F32(Helm, "cmd_z", mask_cmd, self->cmd_data_.z),
      DEBUG_CORE_LIVE_CUSTOM(
          Helm, "module_motion", mask_motion,
          +[](const char* field_name, const Helm* self) {
            LibXR::STDIO::Printf("  %s:\r\n", field_name);
            for (int i = 0; i < 4; ++i) {
              float speed_err = self->target_speed_[i] -
                                self->motor_wheel_feedback_[i].velocity;
              LibXR::STDIO::Printf(
                  "    m%d: speed_t=%.2f speed_n=%.2f err=%.2f ang_t=%.3f "
                  "ang_n=%.3f rev=%d wheel_out=%.1f steer_out=%.1f\r\n",
                  i, self->target_speed_[i],
                  self->motor_wheel_feedback_[i].velocity, speed_err,
                  static_cast<float>(self->target_angle_[i]),
                  static_cast<float>(self->motor_steer_feedback_[i].abs_angle),
                  self->motor_reverse_[i] ? 1 : 0, self->wheel_out_[i],
                  self->steer_out_[i]);
            }
          }),
      DEBUG_CORE_LIVE_BOOL(Helm, "power_limited", mask_power,
                           self->power_control_data_.is_power_limited),
      DEBUG_CORE_LIVE_CUSTOM(
          Helm, "power_current", mask_power,
          +[](const char* field_name, const Helm* self) {
            LibXR::STDIO::Printf("  %s:\r\n", field_name);
            for (int i = 0; i < 4; ++i) {
              float req_3508 = self->motor_data_.output_current_3508[i];
              float req_6020 = self->motor_data_.output_current_6020[i];
              float lim_3508 =
                  self->power_control_data_.is_power_limited
                      ? self->power_control_data_.new_output_current_3508[i]
                      : req_3508;
              float lim_6020 =
                  self->power_control_data_.is_power_limited
                      ? self->power_control_data_.new_output_current_6020[i]
                      : req_6020;
              LibXR::STDIO::Printf(
                  "    m%d: 3508(req/lim)=%.1f/%.1f 6020(req/lim)=%.1f/%.1f "
                  "wheel_cmd=%.1f steer_cmd=%.1f\r\n",
                  i, req_3508, lim_3508, req_6020, lim_6020,
                  self->motor_wheel_cmd_[i].velocity,
                  self->motor_steer_cmd_[i].velocity);
            }
          }),
      DEBUG_CORE_LIVE_CUSTOM(
          Helm, "module_feedback", mask_wheel,
          +[](const char* field_name, const Helm* self) {
            LibXR::STDIO::Printf("  %s:\r\n", field_name);
            for (int i = 0; i < 4; ++i) {
              const auto& wheel = self->motor_wheel_feedback_[i];
              const auto& steer = self->motor_steer_feedback_[i];
              LibXR::STDIO::Printf(
                  "    m%d wheel: state=%u abs=%.3f rpm=%.0f omega=%.3f "
                  "torque=%.3f temp=%.0f\r\n",
                  i, static_cast<unsigned>(wheel.state),
                  static_cast<float>(wheel.abs_angle), wheel.velocity,
                  wheel.omega, wheel.torque, wheel.temp);
              LibXR::STDIO::Printf(
                  "    m%d steer: state=%u abs=%.3f rpm=%.0f omega=%.3f "
                  "torque=%.3f temp=%.0f\r\n",
                  i, static_cast<unsigned>(steer.state),
                  static_cast<float>(steer.abs_angle), steer.velocity,
                  steer.omega, steer.torque, steer.temp);
            }
          }),
  };

  auto lock_self = +[](Helm* self) { self->mutex_.Lock(); };
  auto unlock_self = +[](Helm* self) { self->mutex_.Unlock(); };

  return debug_core::run_live_command(
      this, "helm_chassis", "state|cmd|motion|power|wheel|full", view_table,
      fields, sizeof(fields) / sizeof(fields[0]), argc, argv, view_full,
      lock_self, unlock_self);
}
