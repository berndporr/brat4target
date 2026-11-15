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
 * @brief Turns sensor events into motor outputs.
 * This is an abstract task which has all common features of the derived classes
 * such as attracion or avoidance.
 * Input units are physical units. The Speed is in m/s. The angular velocity is rad/s.
 * Output units are servo units (-1..+1).
 */
class AbstractTask : public std::enable_shared_from_this<AbstractTask>
{
  public:
    /**
     * @brief Distance from which obstacles are detected in the LIDAR input (=world).
     * Default value defined in const.h in meter.
     */
    float nearbyObstacleDetectionRadius = NEARBY_OBSTACLE_DETECTION_RADIUS;

    /**
     * @brief Viewing angle at which obstacles are detected.
     * Default value defined in const. h in rad.
     */
    float nearbyMaxObstacleDetectionAngle
        = NEARBY_MAX_OBSTACLE_DETECTION_ANGLE;

    /**
     * @brief Distance to target which warrants arrival triggers the arrival event.
     */
    float targetArrivedDistance = TARGET_ARRIVED_DISTANCE;

    /**
     * @brief Motor event callback which drives the wheels of the robot. 
     * Values of l and r are between -1 and +1. Not physical speed but servo speed.
     */
    using MotorEvent = std::function<void (const float l, const float r)>;

    /**
     * @brief Registers the motor callback with the physical robot.
     */
    void registerMotorEvent (MotorEvent me) { motorEvent = me; }

    /**
     * @brief Termination message emitted by the task.
     * Message emitted when the task decides to terminate for the reasons below.
     * Contains also a pointer to itself so that the configurator can inspect the
     * task and the next task can aquire values from the terminated task.
     */
    struct TerminationMessage
    {
        /**
         * @brief List of termination reasons why a task has been terminated.
         */
        enum TerminationReason
        {
            INVALID,
            ObstacleDetected,
            ObstacleAvoided,
            TargetReached,
            TargetDetected
        };

        /**
         * @brief The termination reason why this has happened here.
         */
        TerminationReason terminationReason = INVALID;

        /** 
         * @brief The task which has sent the termination message.
         */
        std::shared_ptr<AbstractTask> taskWhichSentMessage = nullptr;

        /** 
         * @brief Constructor which fills both the reason and pointer to this class.
         */
        TerminationMessage (
            TerminationReason _tr,
            std::shared_ptr<AbstractTask> _taskWhichSentMessage)
            : terminationReason (_tr),
              taskWhichSentMessage (_taskWhichSentMessage)
        {
        }

        /** 
         * @brief Default constructor which constructs an invalid message.
         */
        TerminationMessage () = default;
    };

    /**
     * @brief Termination event with the termination message.
     */
    using TerminatedEvent = std::function<void (TerminationMessage)>;

    /**
     * @brief Register a callback of the termination event.
     */
    void registerTerminated (TerminatedEvent te) { terminatedEvent = te; }

    /**
     * @brief Current max angular velocity.
     * Gets the max anglular velocity of the robot which depends on its wheel
     * speed. At max angular velocity the robot just rotates around its axis.
     */
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
                        "angularVel=%f, maxAngVel=%f",
                        l, r, linearSpeed, angularVelocity, maxAngularVel);
        motorEvent (l, r);
    }

  public:
    /**
     * @brief Sets the Angular velocity and triggers a motor event. 
     */
    void setAngularVelocity (const float a)
    {
        bratlogger.log ("Setting angular vel to: %f.", a);
        angularVelocity = a;
        emitTwoWheeledEvent ();
    }

    /**
     * @brief Sets the linear speed in m/s. 
     * It's converted to motor units and triggers a motor event.
     */
    void setLinearSpeed (const float s)
    {
        linearSpeed = s;
        emitTwoWheeledEvent ();
    }

    /**
     * @brief Gets the linear speed in m/s.
     */
    float getLinearSpeed () const { return linearSpeed; }

    /**
     * @brief Event: new LIDAR scan.
     * Task needs to react to the LIDAR world. Needs to be fille with content
     * by its derived classes.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               Robot robot)
        = 0;

    /**
    * @brief Event: new target distance and angle.
    * Task needs to incorporate new target direction from the camera.
    */
    virtual void onTargetDetected (float r, float phi, int targetIndex = 0);

    /**
     * @brief Event: new gyro rotation.
     * Gyro rotation of the robot. Called from the gyro driver.
     * Needs to be implemented by actual tasks below.
     */
    virtual void onGyroTurn (float dphi) = 0;

    /**
    * @brief Event: called when the task is being started.
    */
    virtual void onStart () = 0;

    /**
     * @brief Gets the angular velocity.
     */
    float getangularVelocity () const { return angularVelocity; }

    /**
     * @brief If the robot is moving (speed > 0).
     */
    bool isMoving () const { return fabs (linearSpeed) > 0; }

  protected:
    void correctTargetAngleWithGyro (float dphi);

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
                                     Robot robot);

      protected:
        // This is called by Box2D for every fixture found in the AABB
        bool ReportFixture (b2Fixture *fixture) override;

      private:
        Robot robot;
        const float maxDetectionRadius;
        const float maxDetectionAngle;
        b2Vec2 robotPos;
        float avgThreatAngle = 0;
        int nThreats = 0;
    };

  public:
    /**
     * @brief Default constructor.
     * Is used to create the 1st task in a chain of tasks.
     * This won't inherit anything from previous tasks so uses default values.
     */
    AbstractTask () = default;

    /**
     * @brief Constructor receiving data from previous task.
     * Creates a task based on a previous one receiving
     * all the previous internal values such as linear speed
     * or angular velocity. And most importantly where the
     * target is!
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
     * @brief Returns the target angle as an optional as it might not know it (yet!).
     */
    std::optional<float> getTargetAngle () { return targetAngle; };

    /**
     * @brief Returns the target distance as an optional as it might not know it (yet!).
     */
    std::optional<float> getTargetDistance () { return targetDistance; };

    /**
     * @brief Gets the angle of a potential threat. It's optional as there might be no threat.
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
 * @brief Task which is doing nothing till it receives target info.
 * The task idles till the event onTargetDetected() recieves info
 * about the target. Then this task terminates and it emits a message
 * that a target has been detected.
 */
class StopTask : public AbstractTask
{
  public:
    /**
     * @brief Default constructor with default values.
     */
    StopTask () : AbstractTask () {}

    /**
     * @brief Constructor receiving info from previous task.
     */
    StopTask (std::shared_ptr<AbstractTask> at) : AbstractTask (at) {}

    /**
     * @brief Event: when the task is starting we set the robot's speed to zero.
     */
    virtual void onStart () override { setLinearSpeed (0); }

    /**
     * @brief Event: New LIDAR scan. Ignore!
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               Robot robot) override
    {
    }

    /**
     * @brief Event: new gyro rotation. Ignore!
     */
    virtual void onGyroTurn (float dphi) override {}

    /**
     * @brief Event: target detected.
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
 * @brief Final task not terminating ever.
 * Task which is doing nothing and won't terminate. This can be created
 * once a target has been reached.
 */
class EndTask : public AbstractTask
{
  public:
    /**
     * @brief Default constructor with default values.
     */
    EndTask () : AbstractTask () {}
    /**
     * @brief Constructor receiving info from previous task.
     */
    EndTask (std::shared_ptr<AbstractTask> at) : AbstractTask (at) {}
    virtual void onStart () override { setLinearSpeed (0); }
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               Robot robot) override
    {
    }
    virtual void onTargetDetected (float r, float phi, int index = 0) override
    {
    }
    virtual void onGyroTurn (float dphi) override {}
};

/**
 * @brief Task which is driving towards a target.
 */
class TargetTask : public AbstractTask
{
  public:
    /**
     * @brief Default constructor with default values.
     */
    TargetTask () = default;

    /**
     * @brief Constructor receiving info from previous task.
     */
    TargetTask (std::shared_ptr<AbstractTask> at) : AbstractTask (at) {}

    /** 
     * @brief Steering gain turning the target angle into a steering angle.
     * Default is defined in const.h.
     */
    float targetSteeringGain = TARGET_STEERING_GAIN;

    /**
     * @brief Event: called when the task is starting. 
     * We set the linear speed to the default speed defined in const.h (fixme!).
     */
    virtual void onStart () override { setLinearSpeed (LINEAR_SPEED_DEFAULT); }

    /**
     * @brief Event checks if we have arrived or need to continue targeting.
     */
    virtual void onTargetDetected (float r, float phi, int index = 0) override
    {
        targetAngle = phi;
        bratlogger.logVariable("targetDistance",r);
        if (r < targetArrivedDistance)
        {
            bratlogger.log (
                "onTargetDetected in TargetTask: We have arrived. r=%f < %f",
                r, targetArrivedDistance);
            if (terminatedEvent)
            {
                TerminationMessage tm (TerminationMessage::TargetReached,
                                       shared_from_this ());
                terminatedEvent (tm);
            }
        }
        targetAngle2MotorAngularVelocity ();
    }

    /**
     * @brief Event: new robot rotation info.
     * Task needs to take into account the new gyro readings
     */
    virtual void onGyroTurn (float dphi) override
    {
        // We correct the target angle with the robot rotation.
        correctTargetAngleWithGyro (dphi);
        // We generate a new motor event based on the new target angle.
        targetAngle2MotorAngularVelocity ();
    }

    /**
     * @brief Event: New LIDAR scan.
     * We need to check if we are crashing into an obstacle and to terminate
     * the task!
     */
    void onLIDARworld (std::shared_ptr<b2World> world,
                       Robot robot) override
    {
        // Do we need to terminate the task?
        CloseObjectDetector nearbyBody (nearbyObstacleDetectionRadius,
                                        nearbyMaxObstacleDetectionAngle);

        threatAngle = nearbyBody.detect (world, robot);

        if (threatAngle)
        {
            if (terminatedEvent)
            {
                bratlogger.log ("Task terminating because of obstacle at: %f",
                                threatAngle.value ());
                TerminationMessage tm (TerminationMessage::ObstacleDetected,
                                       shared_from_this ());
                terminatedEvent (tm);
            }
        }
    }

  private:
    // Adjusts steering angle that we drive towards the target.
    void targetAngle2MotorAngularVelocity ()
    {
        if (targetAngle)
        {
            setAngularVelocity (targetAngle.value () * targetSteeringGain);
            bratlogger.logVariable ("angularVelocity", angularVelocity);
        }
    }
};

/**
 * @brief Task which is driving away from an obstacle either to the left or right.
 */
class AvoidTask : public AbstractTask
{
  public:
    /**
     * @brief Number of LIDAR/World callbacks till we decide we are really out of the woods.
     */
    int minLidarAvoidSamples = MIN_LIDAR_AVOID_SAMPLES;

    /**
     * @brief Direction of the avoidance behaviour: to the left or right.
     */
    enum AvoidDirecton
    {
        LEFT,
        RIGHT
    };

    /**
     * @brief Avoid task constructor with the direction (left or right).
     */
    AvoidTask (AvoidDirecton ad) : AbstractTask (), avoidDirection (ad) {}

    /**
     * @brief Avoid task constructor absorbing all variables from its predecessor.
     */
    AvoidTask (std::shared_ptr<AbstractTask> a, AvoidDirecton ad)
        : AbstractTask (a), avoidDirection (ad)
    {
    }

    /**
     * @brief No default constructor as we need to set the direction at least.
     */
    AvoidTask () = delete;

    /**
     * @brief Event: Called when the task is starting.
     * Sets the default speed based on const.h.
     */
    virtual void onStart () override { setLinearSpeed (LINEAR_SPEED_DEFAULT); }

    /**
     * @brief Event: Called with a fresh LIDAR scan as World and the robot in it for simulation.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world,
                               Robot robot) override;

    /**
     * @brief gyroEvent allowing to update the target angle.
     * Most likely the camera can't see the target so
     * we correct the target angle with the gyro readings.
     */
    void onGyroTurn (float dphi) override
    {
        correctTargetAngleWithGyro (dphi);
    }

  protected:
    class BraitenbergAvoider : public b2QueryCallback
    {
      public:
        BraitenbergAvoider (const float maxDetectionRadius,
                            const float maxDetectionAngle)
            : maxDetectionRadius (maxDetectionRadius),
              maxDetectionAngle (maxDetectionAngle) {};

        std::optional<float> detect (std::shared_ptr<b2World> world,
                                     Robot _robot);

      protected:
        // This is called by Box2D for every fixture found in the AABB
        bool ReportFixture (b2Fixture *fixture) override;

      private:
        Robot robot;
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
