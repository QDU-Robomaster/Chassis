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

#include "Chassis.hpp"
#include "app_framework.hpp"

class Mecanum {
 public:
  Mecanum(LibXR::HardwareContainer &hw, LibXR::ApplicationManager &app) {
    // Hardware initialization example:
    // auto dev = hw.template Find<LibXR::GPIO>("led");
  }

  void SelfResolution() {}
  void KinematicsInverseResolution() {}
  void OutputToDynamics() {}

 private:
};
