#pragma once
// Refernce: Cherno video https://www.youtube.com/watch?v=vWXrFetSH8w
#include "kin/artic.h"
#include <memory>

namespace MyApp
{
  void RenderUI();

  extern std::unique_ptr<rb::kin::Artic> robot;
  extern rb::kin::ArmPose pose_tcp;
  extern rb::kin::ArmAxisValue joint_value;
}
