#pragma once

#include "lib/robot.h"
#include <optional>

class TargetTrackerInterface
{
  public:
    TargetTrackerInterface ();
    void onTargetDetected (float r, float phi, int index = 0) const = delete;
    void onGyroTurn (float dphi) const = delete;
    void onLIDARworld (std::shared_ptr<b2World> world, Robot robot) const
        = delete;

    inline std::optional<float> getAngle () const { return targetAngle; }

  protected:
    std::optional<float> targetAngle;
};
