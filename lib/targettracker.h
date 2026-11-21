#pragma once

#include "lib/robot.h"
#include <optional>

class TargetTrackerInterface
{
  public:
    TargetTrackerInterface () = default;
    virtual void onTargetDetected (float r, float phi, int index = 0) = 0;
    virtual void onGyroTurn (float dphi) = 0;
    virtual void onLIDARworld (std::shared_ptr<b2World> world, Robot robot) = 0;

    inline std::optional<float> getAngle () const { return targetAngle; }

  protected:
    std::optional<float> targetAngle;
};
