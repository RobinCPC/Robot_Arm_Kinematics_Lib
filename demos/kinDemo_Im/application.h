#pragma once
// Refernce: Cherno video https://www.youtube.com/watch?v=vWXrFetSH8w
#include "kin/artic.h"
#include <memory>

#define KINDEMO_VERSION   "1.0.1"

namespace MyApp
{
  void RenderUI();

  extern std::unique_ptr<rb::kin::Artic> robot;
  extern rb::kin::ArmPose pose_tcp;
  extern rb::kin::ArmAxisValue joint_value;
  extern bool show_gizmo_window;
}
