#pragma once

#include "lib/robot.h"
#include <optional>

/**
 * @brief Interface definition of a Target Tracker.
 * The tracker has the same event names as throughout and they are being called
 * from within the task.
 */
class TargetTrackerInterface
{
  public:
    TargetTrackerInterface () = default;

    /**
     * @brief Event: new target detected
     * 
     * @param phi Angle of the target
     */
    virtual void onTargetDetected (const float phi) = 0;

    /**
     * @brief Event: new gyro readings (robot's rotation)
     * 
     * @param dphi The robot's rotation in rad reflecting its turning behaviour.
     */
    virtual void onGyroTurn (const float dphi) = 0;

    /**
     * @brief Event: new LIDAR scan
     * 
     * @param world The box2d World generated from the LIDAR scan.
     * @param robot The location of the robot in the world.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world, Robot robot) = 0;

    /**
     * @brief Get the estimated Angle of the target.
     * 
     * @return std::optional<float> The angle of the target if it exists.
     */
    inline std::optional<float> getAngle () const { return targetAngle; }

  protected:
   /**
    * @brief The current estimated target angle
    * Needs to be updated by the derived classes.
    */
    std::optional<float> targetAngle;
};

// Note the simple target tracker just using the gyro is implemented in task.h