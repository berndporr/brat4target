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
        if (currentState)
        {
            if (currentState->task)
            {
                currentState->task->onLIDARworld (
                    world, std::make_shared<Robot> (world));
            }
        }
    }

    /**
     * Task needs to incorporate new target angle info from the camera
     */
    virtual void onTargetDetected (float r, float phi, int targetIndex = 0)
    {
        currentState->task->onTargetDetected (r, phi, targetIndex);
    }

    /**
     * Task needs to take into account the new gyro readings
     */
    virtual void onGyroTurn (float dphi)
    {
        if (currentState)
        {
            if (currentState->task)
            {
                currentState->task->onGyroTurn (dphi);
            }
        }
    }

    /**
     * @brief changes tasks executing on the robot
     */
    virtual void setCurrentTask (std::shared_ptr<AbstractTask> task)
    {
        currentState->task = task;
        currentState->task->registerTerminated (
            [&] (AbstractTask::TerminationMessage tm) {
                onTaskTerminated (tm);
            });
        currentState->task->registerMotorEvent (motorEvent);
        currentState->task->onStart ();
    }

    std::shared_ptr<AbstractTask> getCurrentTask ()
    {
        return currentState->task;
    }

    /**
     * Registers the motor event callback which sets the wheel speeds of the
     * real robot.
     */
    void registerMotorEvent (AbstractTask::MotorEvent me) { motorEvent = me; }

    void setDefaultSpeed (float speed) { defaultSpeed = speed; }

    class Simulator
    {
      public:
        struct Result
        {
            AbstractTask::TerminationMessage terminationMessage;
            b2Transform endPose = b2Transform (b2Vec2 (0.0, 0.0), b2Rot (0));
            int step = 0;
            Result () = default;
            std::shared_ptr<AbstractTask> nextTask;
        };

        Simulator (std::shared_ptr<b2World> _world)
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

        BaseConfigurator::Simulator::Result
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
    // the root
    std::shared_ptr<State> plan;
    // the current position in the tree
    std::shared_ptr<State> currentState;
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
        switch (tm.terminationReason)
        {
        case AbstractTask::TerminationMessage::TerminationReason::
            ObstacleDetected:
            bratlogger.log ("Obstacle detected.\n");
            if (tm.targetAngle)
            {
                if (tm.targetAngle.value () > 0)
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
    void onTargetDetected (float r, float phi, int targetIndex = 0) override
    {
        // We have no plan. Let's create one.
        if (nullptr == plan)
        {
            plan = std::make_shared<State> ();
            currentState = plan;
            plan->task = std::make_shared<TargetTask> ();
            plan->task->setLinearSpeed (defaultSpeed);
            setCurrentTask (plan->task);
            bratlogger.log ("Created Target plan.\n");
        }
        BaseConfigurator::onTargetDetected (r, phi, targetIndex);
    }
};
