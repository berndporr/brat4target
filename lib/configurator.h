#pragma once

#include "debug.h"
#include "robot.h"
#include "task.h"
#include <box2d/b2_math.h>
#include <box2d/b2_world.h>
#include <dirent.h>
#include <memory>
#include <optional>
#include <sys/stat.h>
#include <vector>

/** 
 * Cognitive map node.
 */
struct State
{
    std::vector<std::shared_ptr<State> > children;
    std::shared_ptr<AbstractTask> task = std::make_shared<StopTask> ();
};

// The base configurator does not plan.
class BaseConfigurator
{
  public:
    BaseConfigurator () = default;

    /**
     * Needs to react to the LIDAR world
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world)
    {
        currentTask->onLIDARworld (world, std::make_shared<Robot> (world));
    }

    /**
     * Task needs to incorporate new target angle info from the camera
     */
    virtual void onTargetDetected (float r, float phi, int targetIndex = 0)
    {
        currentTask->onTargetDetected (r, phi, targetIndex);
    }

    /**
     * Task needs to take into account the new gyro readings
     */
    virtual void onGyroTurn (float dphi) { currentTask->onGyroTurn (dphi); }

    /**
     * @brief changes tasks executing on the robot
     */
    virtual void setCurrentTask (std::shared_ptr<AbstractTask> task)
    {
        currentTask = task;
        currentTask->registerTerminated (
            [&] (AbstractTask::TerminationMessage tm) {
                onTaskTerminated (tm);
            });
        currentTask->registerMotorEvent (motorEvent);
        currentTask->onStart ();
    }

    std::shared_ptr<AbstractTask> getCurrentTask () { return currentTask; }

    /**
     * Registers the motor event callback which sets the wheel speeds of the
     * real robot.
     */
    void registerMotorEvent (AbstractTask::MotorEvent me) { motorEvent = me; }

    void setDefaultSpeed (float speed) { defaultSpeed = speed; }

    class TaskSimulator
    {
      public:
        TaskSimulator (std::shared_ptr<b2World> _world)
        {
            world = _world;
            start = b2Transform_zero;
            robot = std::make_shared<Robot> (world, start);
            stepb2d = 0;
            theta = start.q.GetAngle ();
            instVelocity = { 0, 0 };
        }

        void setTarget (float r, float phi)
        {
            b2Vec2 t;
            t.x = r * cos (phi);
            t.y = r * sin (phi);
            targetPos = std::optional<b2Vec2> (t);
        }

        AbstractTask::TerminationMessage
        run (std::shared_ptr<AbstractTask> task,
             const char *plan_file = nullptr);

      private:
        float cameraAngle = M_PI / 2;
        void checkTarget (std::shared_ptr<AbstractTask> task);
        virtual float
        remainingSimulationTime (std::shared_ptr<AbstractTask> task);
        std::shared_ptr<b2World> world;
        std::shared_ptr<Robot> robot;
        float theta = 0;
        b2Vec2 instVelocity = { 0, 0 };
        int stepb2d = 0;
        b2Transform start = b2Transform_zero;
        std::optional<b2Vec2> targetPos;
    };

  protected:
    virtual void onTaskTerminated (AbstractTask::TerminationMessage tm)
    {
        setCurrentTask (std::make_shared<StopTask> ());
        bratlogger.log ("Terminated: stopping.\n");
    }

    AbstractTask::MotorEvent motorEvent;
    std::shared_ptr<AbstractTask> currentTask;
    float defaultSpeed = 0.5;
};

class ReactiveConfigurator : public BaseConfigurator
{
  public:
    State planner (std::shared_ptr<b2World> world, float target_distance,
                   float target_angle);

  private:
    virtual void
    onTaskTerminated (AbstractTask::TerminationMessage tm) override
    {
        bratlogger.log ("Reactive: onTaskTerminated");
        switch (tm.terminationReason)
        {
        case AbstractTask::TerminationMessage::TargetDetected:
            setCurrentTask (std::make_shared<TargetTask> (getCurrentTask ()));
            bratlogger.log ("Targeting again.\n");
            break;
        case AbstractTask::TerminationMessage::ObstacleDetected:
            bratlogger.log ("Obstacle detected.\n");
            if (tm.taskWhichSentMessage->getTargetAngle ())
            {
                if (tm.taskWhichSentMessage->getTargetAngle ().value () > 0)
                {
                    bratlogger.log (
                        "Avoidance to the LEFT b/c target to the left.\n");
                    auto t = std::make_shared<AvoidTask> (
                        getCurrentTask (), AvoidTask::AvoidDirecton::LEFT);
                    setCurrentTask (t);
                }
                else
                {
                    bratlogger.log (
                        "Avoidance to the RIGHT b/c target to the right.\n");
                    auto t = std::make_shared<AvoidTask> (
                        getCurrentTask (), AvoidTask::AvoidDirecton::RIGHT);
                    setCurrentTask (t);
                }
            }
            else
            {
                if (random () & 1)
                {
                    bratlogger.log (
                        "Avoidance to the LEFT b/c random choice.\n");
                    auto t = std::make_shared<AvoidTask> (
                        getCurrentTask (), AvoidTask::AvoidDirecton::LEFT);
                    setCurrentTask (t);
                }
                else
                {
                    bratlogger.log (
                        "Avoidance to the RIGHT b/c random choice.\n");
                    auto t = std::make_shared<AvoidTask> (
                        getCurrentTask (), AvoidTask::AvoidDirecton::LEFT);
                    setCurrentTask (t);
                }
            }
            break;
        case AbstractTask::TerminationMessage::ObstacleAvoided:
            setCurrentTask (std::make_shared<TargetTask> (getCurrentTask ()));
            bratlogger.log ("Obstacle avoided. Now targeting again.\n");
            break;
        case AbstractTask::TerminationMessage::TargetReached:
        case AbstractTask::TerminationMessage::NONE:
            setCurrentTask (std::make_shared<StopTask> (getCurrentTask ()));
            bratlogger.log ("Terminated: stopping.\n");
            break;
        }
    }
};
