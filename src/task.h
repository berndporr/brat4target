#pragma once

#include "const.h"
#include "debug.h"
#include "robot.h"
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>
#include <box2d/b2_world_callbacks.h>
#include <functional>
#include <memory>
#include <optional>

/**
 * @brief Turns sensorevents into motor outputs in a reactive way
 * This is an abstract task
 */
class AbstractTask
{
  public:
    float nearbyObstacleDetectionRadius = NEARBY_OBSTACLE_DETECTION_RADIUS;
    float nearbyMaxObstacleDetectionAngle = NEARBY_MAX_OBSTACLE_DETECTION_ANGLE;

    using MotorEvent = std::function<void (const float l, const float r)>;
    void registerMotorEvent (MotorEvent me) { motorEvent = me; }

    struct TerminationMessage
    {
        enum TerminationReason
        {
            NONE,
            ObstacleDetected,
            ObstacleAvoided,
            TargetReached
        };
        TerminationReason terminationReason = NONE;
    };

    using TerminatedEvent = std::function<void (TerminationMessage)>;
    void registerTerminated (TerminatedEvent te) { terminatedEvent = te; }

    void emitTwoWheeledEvent ()
    {
        if (!motorEvent)
            return;
        const float maxAngularVel = 2.0f * linearSpeed / BETWEEN_WHEELS;
        float r = 0;
        float l = 0;
        if (maxAngularVel > 0)
        {
            l = linearSpeed - angularVelocity / maxAngularVel;
            r = linearSpeed + angularVelocity / maxAngularVel;
        }
        if (DEBUG)
            fprintf (
                stderr,
                "l=%f, r=%f, linearSpeed=%f, angularVel=%f, maxAngVel=%f\n", l,
                r, linearSpeed, angularVelocity, maxAngularVel);
        motorEvent (l, r);
    }

    void setAngularVelocity (const float a)
    {
        if (DEBUG)
            fprintf (stderr, "Setting angular vel to: %f\n", angularVelocity);
        angularVelocity = a;
        emitTwoWheeledEvent ();
    }

    void setLinearSpeed (const float s)
    {
        linearSpeed = s;
        emitTwoWheeledEvent ();
    }

    /**
     * Task needs to react to the LIDAR world.
     * This has already a working functionality where it detects close objects
     * which would lead to an imminent crash or might be the target itself.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               std::shared_ptr<Robot> robot);

    /**
     * Task needs to incorporate new target angle info from the camera.
     */
    virtual void onTargetAngle (float phi);

    /**
     * Task needs to take into account the new gyro readings
     */
    virtual void onGyroTurn (float dphi);

    /**
     * Callback when the task is started
     */
    virtual void onStart () = 0;

    b2Vec2 getLinearVelocity (const float dt = 1) const
    { //dt integrates
        b2Vec2 velocity;
        velocity.x = linearSpeed * cos (angularVelocity) * dt;
        velocity.y = linearSpeed * sin (angularVelocity) * dt;
        return velocity;
    }

    /**
        * @brief Gets the ROBOT's displacement in the world frame after dt seconds
        * 
        * @param dt delta time (in seconds)
        **/
    b2Transform getTransform (const float dt = 1) const
    { //dt integrates
        return b2Transform (getLinearVelocity (dt),
                            b2Rot (getangularVelocity (dt)));
    }

    float getLinearSpeed () const { return linearSpeed; }

    float getangularVelocity (const float dt = 1) const
    {
        return angularVelocity * dt;
    }

    bool isMoving () const { return fabs (linearSpeed) > 0; }

  protected:
    class CloseObjectDetector : public b2QueryCallback
    {
      public:
        CloseObjectDetector (const float maxDetectionRadius,
                             const float maxDetectionAngle)
            : maxDetectionRadius (maxDetectionRadius),
              maxDetectionAngle (maxDetectionAngle)
        {
        }

        float relativeThreatAngle = 0;

        bool detect (std::shared_ptr<b2World> world,
                     std::shared_ptr<Robot> robot);

      protected:
        // This is called by Box2D for every fixture found in the AABB
        bool ReportFixture (b2Fixture *fixture) override;

      private:
        std::shared_ptr<Robot> robot;
        b2Body *detectedBody = nullptr;
        const float maxDetectionRadius;
        const float maxDetectionAngle;
        b2Vec2 robotPos;
    };

  public:
    void init(std::shared_ptr<AbstractTask> at) {
      linearSpeed = at->linearSpeed;
      angularVelocity = at->angularVelocity;
      motorEvent = at->motorEvent;
      terminatedEvent = at->terminatedEvent;
      targetAngle = at->targetAngle;
    }

  protected:
    float linearSpeed = 0;
    float angularVelocity = 0;
    MotorEvent motorEvent;
    TerminatedEvent terminatedEvent;
    std::optional<float> targetAngle;
};

/**
 * Task which is doing nothing and just stopping the robot.
 */
class StopTask : public AbstractTask
{
  public:
    virtual void onStart () override { setLinearSpeed (0); }
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               std::shared_ptr<Robot> robot) override
    {
    }
    virtual void onTargetAngle (float phi) override {}
};

/**
 * Task which is driving towards the target.
 */
class TargetTask : public AbstractTask
{
  public:
    float targetSteeringGain = TARGET_STEERING_GAIN;

    virtual void onStart () override {}
    virtual void onTargetAngle (float phi) override
    {
        targetAngle = phi;
        updateTargeting ();
    }
    /**
     * Task needs to take into account the new gyro readings
     */
    virtual void onGyroTurn (float dphi) override
    {
        AbstractTask::onGyroTurn (dphi);
        updateTargeting ();
    }

  protected:
    void updateTargeting ()
    {
        if (targetAngle)
            setAngularVelocity (targetAngle.value () * targetSteeringGain);
    }
    float targetPhi = 0;
    bool targetValid = false;
};

class AvoidTask : public AbstractTask
{
  public:
    float avoidTaskSteeringGain = AVOID_TASK_STEERING_GAIN;
    int minLidarAvoidSamples = MIN_LIDAR_AVOID_SAMPLES;

    virtual void onStart () override {}
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               std::shared_ptr<Robot> robot) override;

  protected:
    class BraitenbergAvoider : public b2QueryCallback
    {
      public:
        BraitenbergAvoider (const float maxDetectionRadius,
                            const float maxDetectionAngle)
            : maxDetectionRadius (maxDetectionRadius),
              maxDetectionAngle (maxDetectionAngle){};

        bool detect (std::shared_ptr<b2World> world,
                     std::shared_ptr<Robot> _robot);
        float getAverageThreatAngle () const { return averageThreatAngle; }
        bool hasDetectedThreats () { return totalWeight > 0; }

      protected:
        // This is called by Box2D for every fixture found in the AABB
        bool ReportFixture (b2Fixture *fixture) override;

      private:
        std::shared_ptr<Robot> robot;
        const float maxDetectionRadius;
        const float maxDetectionAngle;
        b2Vec2 robotPos;
        float robotAngle = 0;
        float totalWeightedAngle = 0.0f;
        float totalWeight = 0.0f;
        float averageThreatAngle = 0.0f;
    };
    int nSamplesWithoutThreat = 0;
};
