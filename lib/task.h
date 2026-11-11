#pragma once

#include "const.h"
#include "debug.h"
#include "robot.h"
#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>
#include <box2d/b2_world_callbacks.h>
#include <functional>
#include <memory>
#include <optional>

/**
 * @brief Turns sensor events into motor outputs in a reactive way
 * This is an abstract task which has all common features of the derived classes
 * such as attracion or avoidance.
 * Units are physical units. The Speed it m/s and converted here to
 * motor units. The angular velocity is rad/s.
 */
class AbstractTask : public std::enable_shared_from_this<AbstractTask>
{
  public:
    /**
     * Distance from which obstacles are detected in the LIDAR input (=world).
     * Default value defined in const.h in meter.
     */
    float nearbyObstacleDetectionRadius = NEARBY_OBSTACLE_DETECTION_RADIUS;

    /**
     * Viewing angle at which obstacles are detected.
     * Default value defined in const. h in rad.
     */
    float nearbyMaxObstacleDetectionAngle
        = NEARBY_MAX_OBSTACLE_DETECTION_ANGLE;

    /**
     * Distance to target which warrants arrival triggers the arrival event.
     */
    float targetArrivedDistance = TARGET_ARRIVED_DISTANCE;

    /**
     * Motor event callback which drives the wheels of the robot. Values
     * of l and r are between -1 and +1. Not physical speed.
     */
    using MotorEvent = std::function<void (const float l, const float r)>;

    /**
     * Registers the motor callback with the physical robot.
     */
    void registerMotorEvent (MotorEvent me) { motorEvent = me; }

    /**
     * Message emitted when the task decides to terminate for the reasons below.
     */
    struct TerminationMessage
    {
        // Termination reasons
        enum TerminationReason
        {
            INVALID,
            ObstacleDetected,
            ObstacleAvoided,
            TargetReached,
            TargetDetected
        };
        // The termination reason
        TerminationReason terminationReason = INVALID;
        // This task which has sent the termination message.
        std::shared_ptr<AbstractTask> taskWhichSentMessage = nullptr;

        // Constructor which fills both the reason and pointer to this class.
        TerminationMessage (
            TerminationReason _tr,
            std::shared_ptr<AbstractTask> _taskWhichSentMessage)
            : terminationReason (_tr),
              taskWhichSentMessage (_taskWhichSentMessage)
        {
        }

        // Default constructor which constructs an invalid message.
        TerminationMessage () = default;
    };

    /**
     * Termination event with the termination message.
     */
    using TerminatedEvent = std::function<void (TerminationMessage)>;

    /**
     * Register a callback of the termination event.
     */
    void registerTerminated (TerminatedEvent te) { terminatedEvent = te; }

    float getMaxAngularVelocity () const
    {
        return 2.0f * linearSpeed / BETWEEN_WHEELS;
    }

  private:
    // Converts the phys speeds into servo speeds and sends them to the motors.
    void emitTwoWheeledEvent ()
    {
        if (!motorEvent)
            return;
        float r = 0;
        float l = 0;
        // max angular possible velocity at a given linear speed
        const float maxAngularVel = getMaxAngularVelocity ();
        if (fabs (linearSpeed) > 0)
        {
            // turning behaviour the robot from -1..+1
            // -1 or +1 the robot rotates on its spot with max angular velocity
            const float servoAngularVelocity = angularVelocity / maxAngularVel;
            // speed in continous servo units from -1..+1
            const float contServoSpeed
                = linearSpeed * REAL_SPEED_TO_CONT_SERVO_SPEED;
            l = contServoSpeed - contServoSpeed * servoAngularVelocity;
            r = contServoSpeed + contServoSpeed * servoAngularVelocity;
        }
        bratlogger.log ("emitTwoWheeledEvent: l=%f, r=%f, linearSpeed=%f, "
                        "angularVel=%f, maxAngVel=%f\n",
                        l, r, linearSpeed, angularVelocity, maxAngularVel);
        motorEvent (l, r);
    }

  public:
    /**
     * Sets the Angular velocity and triggers a motor event. 
     */
    void setAngularVelocity (const float a)
    {
        bratlogger.log ("Setting angular vel to: %f\n", a);
        angularVelocity = a;
        emitTwoWheeledEvent ();
    }

    /**
     * Sets the linear speed in m/s. It's converted here to motor units.
     */
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
    virtual void onTargetDetected (float r, float phi, int targetIndex = 0);

    /**
    * Task needs to take into account the new gyro readings
    */
    virtual void onGyroTurn (float dphi);

    /**
    * Callback when the task is started
    */
    virtual void onStart () = 0;

    /**
     * Gets the linear speed in cont servo units -1..+1.
     */
    float getLinearSpeed () const { return linearSpeed; }

    /**
     * Gets the angular velocity.
     */
    float getangularVelocity () const { return angularVelocity; }

    /**
     * If the robot is moving (speed > 0).
     */
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

        // returns the threat angle if detected
        std::optional<float> detect (std::shared_ptr<b2World> world,
                                     std::shared_ptr<Robot> robot);

      protected:
        // This is called by Box2D for every fixture found in the AABB
        bool ReportFixture (b2Fixture *fixture) override;

      private:
        std::shared_ptr<Robot> robot;
        const float maxDetectionRadius;
        const float maxDetectionAngle;
        b2Vec2 robotPos;
        float avgThreatAngle = 0;
        int nThreats = 0;
    };

  public:
    /**
     * To create the 1st task on a chain of tasks.
     */
    AbstractTask () = default;

    /**
     * Creates a task based on a previous one receiving
     * all the previous internal values such as linear speed
     * or angular velocity.
     */
    AbstractTask (std::shared_ptr<AbstractTask> at)
    {
        if (at)
        {
            linearSpeed = at->linearSpeed;
            angularVelocity = at->angularVelocity;
            motorEvent = at->motorEvent;
            terminatedEvent = at->terminatedEvent;
            targetAngle = at->targetAngle;
            targetDistance = at->targetDistance;
        }
    }

  public:
    /**
     * Returns the target angle as an optional as it might not know it (yet!).
     */
    std::optional<float> getTargetAngle () { return targetAngle; };

    /**
     * Returns the target distance as an optional as it might not know it (yet!).
     */
    std::optional<float> getTargetDistance () { return targetDistance; };

    /**
     * Get the angle of a potential threat
     */
    std::optional<float> getThreatAngle () { return threatAngle; }

  protected:
    float linearSpeed = 0;
    float angularVelocity = 0;
    MotorEvent motorEvent;
    TerminatedEvent terminatedEvent;
    std::optional<float> targetAngle;
    std::optional<float> targetDistance;
    std::optional<float> threatAngle;
};

/**
 * Task which is doing nothing and just stopping the robot. But it terminates if a target is detected.
 */
class StopTask : public AbstractTask
{
  public:
    StopTask () : AbstractTask () {}
    StopTask (std::shared_ptr<AbstractTask> at) : AbstractTask (at) {}
    virtual void onStart () override { setLinearSpeed (0); }
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               std::shared_ptr<Robot> robot) override
    {
    }

    /**
     * Target is detected and that triggers the termination of the stop task
     * and then emits a message that we have a target.
     */
    virtual void onTargetDetected (float r, float phi, int index = 0) override
    {
        targetAngle = phi;
        targetDistance = r;
        if (terminatedEvent)
        {
            TerminationMessage tm (TerminationMessage::TargetDetected,
                                   shared_from_this ());
            terminatedEvent (tm);
        }
    }
};

/**
 * Task which is doing nothing and won't terminate.
 */
class EndTask : public AbstractTask
{
  public:
    EndTask () : AbstractTask () {}
    EndTask (std::shared_ptr<AbstractTask> at) : AbstractTask (at) {}
    virtual void onStart () override { setLinearSpeed (0); }
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               std::shared_ptr<Robot> robot) override
    {
    }

    /**
     * Target is detected and that triggers the termination of the stop task
     * and then emits a message that we have a target.
     */
    virtual void onTargetDetected (float r, float phi, int index = 0) override
    {
    }
};

/**
 * Task which is driving towards the target.
 */
class TargetTask : public AbstractTask
{
  public:
    TargetTask (std::shared_ptr<AbstractTask> at) : AbstractTask (at) {}
    TargetTask () = default;

    float targetSteeringGain = TARGET_STEERING_GAIN;

    virtual void onStart () override { setLinearSpeed (LINEAR_SPEED_DEFAULT); }

    /**
     * Event checks if we have arrived or need to continue targeting.
     */
    virtual void onTargetDetected (float r, float phi, int index = 0) override
    {
        targetAngle = phi;
        if (r < targetArrivedDistance)
        {
            if (terminatedEvent)
            {
                TerminationMessage tm (TerminationMessage::TargetReached,
                                       shared_from_this ());
                terminatedEvent (tm);
            }
        }
        bratlogger.logVariable ("targetAngle", targetAngle.value ());
        updateTargeting ();
    }

    /**
     * Task needs to take into account the new gyro readings
     */
    virtual void onGyroTurn (float dphi) override
    {
        bratlogger.logVariable ("dphi", dphi);
        AbstractTask::onGyroTurn (dphi);
        updateTargeting ();
    }

  private:
    // Adjusts steering angle that we drive towards the target.
    void updateTargeting ()
    {
        if (targetAngle)
        {
            setAngularVelocity (targetAngle.value () * targetSteeringGain);
        }
    }
};

/**
 * Task which is driving away from an obstacle either to the left or right.
 */
class AvoidTask : public AbstractTask
{
  public:
    /**
     * Number of LIDAR/World callbacks till we decide we are really out of the woods.
     */
    int minLidarAvoidSamples = MIN_LIDAR_AVOID_SAMPLES;

    /**
     * Direction of the avoidance behaviour: to the left or right.
     */
    enum AvoidDirecton
    {
        LEFT,
        RIGHT
    };

    /**
     * Avoid task constructor with the direction (left or right).
     */
    AvoidTask (AvoidDirecton ad) : AbstractTask (), avoidDirection (ad) {}

    /**
     * Avoid task constructor absorbing all variables from its predecessor.
     */
    AvoidTask (std::shared_ptr<AbstractTask> a, AvoidDirecton ad)
        : AbstractTask (a), avoidDirection (ad)
    {
    }

    /**
     * No default constructor as we need to set the direction at least.
     */
    AvoidTask () = delete;

    /**
     * Called when the task is starting.
     */
    virtual void onStart () override { setLinearSpeed (LINEAR_SPEED_DEFAULT); }

    /**
     * Called with a fresh LIDAR scan as World and the robot in it for simulation.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               std::shared_ptr<Robot> robot) override;

  protected:
    class BraitenbergAvoider : public b2QueryCallback
    {
      public:
        BraitenbergAvoider (const float maxDetectionRadius,
                            const float maxDetectionAngle)
            : maxDetectionRadius (maxDetectionRadius),
              maxDetectionAngle (maxDetectionAngle) {};

        std::optional<float> detect (std::shared_ptr<b2World> world,
                                     std::shared_ptr<Robot> _robot);

      protected:
        // This is called by Box2D for every fixture found in the AABB
        bool ReportFixture (b2Fixture *fixture) override;

      private:
        std::shared_ptr<Robot> robot;
        const float maxDetectionRadius;
        const float maxDetectionAngle;
        b2Vec2 robotPos;
        float robotAngle = 0;
        float accDistance = 0;
        int nDistance = 0;
    };
    int nSamplesWithoutThreat = 0;
    const AvoidDirecton avoidDirection;
};
