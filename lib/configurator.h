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

/**
 * The base configurator which has all the common functionality.
 */
class BaseConfigurator
{
  public:
    BaseConfigurator () = default;

    /**
     * Needs to react to the LIDAR world. Feeds this simply to the current task.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world)
    {
        if (currentTask)
            currentTask->onLIDARworld (world, std::make_shared<Robot> (world));
    }

    /**
     * Task needs to incorporate new target angle info from the camera.
     * Feeds it to the current task.
     */
    virtual void onTargetDetected (float r, float phi, int targetIndex = 0)
    {
        if (currentTask)
            currentTask->onTargetDetected (r, phi, targetIndex);
    }

    /**
     * Task needs to take into account the new gyro readings.
     * Feeds it into the current task.
     */
    virtual void onGyroTurn (float dphi)
    {
        if (currentTask)
            currentTask->onGyroTurn (dphi);
    }

    /**
     * Changes the current task, registers the termination callback,
     * the motor callback and calls the task's "onStart()" telling
     * it's now running.
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

    /**
     * Gets the current task as a shared pointer.
     */
    std::shared_ptr<AbstractTask> getCurrentTask () { return currentTask; }

    /**
     * Registers the motor event callback which sets the wheel speeds of the
     * real robot.
     */
    void registerMotorEvent (AbstractTask::MotorEvent me) { motorEvent = me; }

    /**
     * Sets the default speed. That's mostly if we have a stop task and the next
     * task needs an initial speed. Fixme: at the moment the speed is enforced.
     */
    void setDefaultSpeed (float speed) { defaultSpeed = speed; }

    /**
     * Simulates a task in the gaming engine box2d till it terminates.
     * Then returns the result. It's not suggesting the next task but
     * just reports why it has terminated. The configurator flavour needs
     * to then decide what to do next. The simulator can be called
     * with a chain of tasks.
     */
    class TaskSimulator
    {
      public:
        /**
         * The simulator it initialised with the world (LIDAR scan)
         */
        TaskSimulator (std::shared_ptr<b2World> _world)
        {
            world = _world;
            start = b2Transform_zero;
            robot = std::make_shared<Robot> (world, start);
            stepb2d = 0;
            theta = start.q.GetAngle ();
            instVelocity = { 0, 0 };
        }

        /**
         * Setting the target locaton. This will then be turned into
         * callbacks to onTargetDetected.
         */
        void setTarget (float r, float phi)
        {
            b2Vec2 t;
            t.x = r * cos (phi);
            t.y = r * sin (phi);
            targetPos = std::optional<b2Vec2> (t);
        }

        /**
         * Runs the task in box2d till it terminates.
         * Optionally, one can add a file which logs the coordinates of the robot till it terminates.
         */
        AbstractTask::TerminationMessage
        run (std::shared_ptr<AbstractTask> task,
             const char *plan_file = nullptr);

      private:
        float cameraAngle = M_PI / 2;
        void checkTarget (std::shared_ptr<AbstractTask> task);
        std::shared_ptr<b2World> world;
        std::shared_ptr<Robot> robot;
        float theta = 0;
        b2Vec2 instVelocity = { 0, 0 };
        int stepb2d = 0;
        b2Transform start = b2Transform_zero;
        std::optional<b2Vec2> targetPos;
    };

    /**
     * On task terminated needs to be filled with life when inheriting this
     * base constructor. It needs to decide which task is next!
     */
    virtual void onTaskTerminated (AbstractTask::TerminationMessage tm) = 0;

  private:
    AbstractTask::MotorEvent motorEvent;
    std::shared_ptr<AbstractTask> currentTask;
    float defaultSpeed = 0.5;
};

class ReactiveConfigurator : public BaseConfigurator
{
  public:
    // static so that the tests can test it without touching the configurator state.
    static std::shared_ptr<AbstractTask>
    terminationMessage2task (AbstractTask::TerminationMessage tm)
    {
        bratlogger.log ("Reactive: onTaskTerminated");
        switch (tm.terminationReason)
        {
        case AbstractTask::TerminationMessage::TargetDetected:
            bratlogger.log (
                "ReactiveConfigurator: Target detected. We are targeting.\n");
            return std::make_shared<TargetTask> (tm.taskWhichSentMessage);
        case AbstractTask::TerminationMessage::ObstacleDetected:
            bratlogger.log ("ReactiveConfigurator: Obstacle detected.\n");
            if (tm.taskWhichSentMessage->getThreatAngle())
            {
                if (tm.taskWhichSentMessage->getThreatAngle().value () > 0)
                {
                    bratlogger.log ("ReactiveConfigurator: Avoidance to the "
                                    "RIGHT b/c obstacle to the left.\n");
                    return std::make_shared<AvoidTask> (
                        tm.taskWhichSentMessage,
                        AvoidTask::AvoidDirecton::RIGHT);
                }
                else
                {
                    bratlogger.log ("ReactiveConfigurator: Avoidance to the "
                                    "LEFT b/c obstacle to the right.\n");
                    return std::make_shared<AvoidTask> (
                        tm.taskWhichSentMessage,
                        AvoidTask::AvoidDirecton::LEFT);
                }
            }
            else
            {
                if (random () & 1)
                {
                    bratlogger.log ("ReactiveConfigurator: Avoidance to the "
                                    "LEFT b/c random choice.\n");
                    return std::make_shared<AvoidTask> (
                        tm.taskWhichSentMessage,
                        AvoidTask::AvoidDirecton::LEFT);
                }
                else
                {
                    bratlogger.log ("ReactiveConfigurator: Avoidance to the "
                                    "RIGHT b/c random choice.\n");
                    return std::make_shared<AvoidTask> (
                        tm.taskWhichSentMessage,
                        AvoidTask::AvoidDirecton::LEFT);
                }
            }
            break;
        case AbstractTask::TerminationMessage::ObstacleAvoided:
            bratlogger.log ("ReactiveConfigurator: Obstacle avoided. Now "
                            "targeting again.\n");
            return std::make_shared<TargetTask> (tm.taskWhichSentMessage);
        case AbstractTask::TerminationMessage::TargetReached:
            bratlogger.log (
                "ReactiveConfigurator: TargetReached: stopping.\n");
            return std::make_shared<StopTask> (tm.taskWhichSentMessage);
        case AbstractTask::TerminationMessage::INVALID:
            bratlogger.log (
                "ReactiveConfigurator: Message==INVALID (BUG?): stopping.\n");
            return std::make_shared<StopTask> (tm.taskWhichSentMessage);
        }
        bratlogger.log (
            "ReactiveConfigurator: Message==??? (BUG?): stopping.\n");
        return std::make_shared<StopTask> (tm.taskWhichSentMessage);
    }

  protected:
    virtual void
    onTaskTerminated (AbstractTask::TerminationMessage tm) override
    {
        setCurrentTask (terminationMessage2task (tm));
    }
};
