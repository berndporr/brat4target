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
 * @brief Cognitive map node.
 */
struct State
{
    std::vector<std::shared_ptr<State> > children;
    std::shared_ptr<AbstractTask> task = std::make_shared<EndTask> ();
};

/**
 * @brief Decides which task is next after it has terminated.
 * The configurator hosts the current task and decides which
 * task to create next after the task has terminated.
 * The base configurator has all the common functionality transmitting
 * the sensor events to the task but has a pure virual function for the
 * task termination message which needs to be implemented by its
 * children.
 */
class BaseConfigurator
{
  public:
    BaseConfigurator () = default;

    /**
     * @brief Event: new LIDAR scan.
     * Needs to react to the LIDAR world. Feeds this simply to the current task.
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world)
    {
        if (currentTask)
            currentTask->onLIDARworld (world, Robot (world));
    }

    /**
     * @brief Event: new target detected.
     * Task needs to incorporate new target angle info from the camera.
     * Feeds it to the current task.
     */
    virtual void onTargetDetected (float r, float phi, int targetIndex = 0)
    {
        if (currentTask)
        {
            currentTask->onTargetDetected (r, phi, targetIndex);
            bratlogger.logVariable ("targetPhi", phi);
        }
    }

    /**
     * @brief Event: New robot rotation event detected.
     * Task needs to take into account the new gyro readings.
     * Feeds it into the current task.
     */
    virtual void onGyroTurn (float dphi)
    {
        if (currentTask)
        {
            currentTask->onGyroTurn (dphi);
            bratlogger.logVariable ("dphi", dphi);
        }
    }

    /**
     * @brief Registers the motor event callback.
     * This in turn then is registered with the current task and sets the wheel speeds of the
     * real robot.
     */
    void registerMotorEvent (AbstractTask::MotorEvent me) { motorEvent = me; }

    /**
     * @brief Gets the current task as a shared pointer.
     */
    std::shared_ptr<AbstractTask> getCurrentTask () { return currentTask; }

    /**
     * @brief Sets the initial task of the configurator.
     * This sets the initital task from which the configurator then creates
     * the follow up tasks either reactive or as a plan.
     * 
     * @param initialTask The initial task. That's usually one which is waiting for a target.
     */
    void setInitialTask (std::unique_ptr<AbstractTask> initialTask)
    {
        setCurrentTask (std::move (initialTask));
    }

  protected:
    /**
     * @brief Changes the current task, registers the termination callback,
     * the motor callback and calls the task's "onStart()" telling
     * it's now running.
     * @param task New task which shall be runnung
     * @param targetTracker Sets a new target tracker. Will be passed on.
     */
    void setCurrentTask (std::shared_ptr<AbstractTask> task)
    {
        currentTask = task;
        currentTask->registerTerminated (
            [&] (AbstractTask::TerminationMessage tm) {
                onTaskTerminated (tm);
            });
        currentTask->registerMotorEvent (motorEvent);
        currentTask->onStart ();
    }

  public:
    /**
     * @brief Simulating one or a chain of tasks in a LIDAR scan.
     * Simulates a task in the gaming engine box2d till it terminates.
     * Then returns the termination message. It's not suggesting the next task but
     * just reports why it has terminated. The configurator flavour needs
     * to then decide what to do next.
     */
    class TaskSimulator
    {
      public:
        /**
         * @brief Constructor receiving the world (=LIDAR scan)
         * The simulator is initialised with the world (LIDAR scan). The LIDAR
         * scan stays always the same while run() can be called multiple times
         * with different tasks.
         */
        TaskSimulator (std::shared_ptr<b2World> _world)
        {
            world = _world;
            start = b2Transform_zero;
        }

        /**
         * @brief Setting the target locaton (radius and angle).
         * This will then be turned into callbacks onTargetDetected 
         * by the simulator for the task being executed.
         */
        void setTarget (float r, float phi)
        {
            b2Vec2 t;
            t.x = r * cos (phi);
            t.y = r * sin (phi);
            targetPos = std::optional<b2Vec2> (t);
        }

        /**
         * @brief Runs the task in box2d till it terminates.
         * Optionally, one can add a file which logs the coordinates of the robot till it terminates.
         */
        AbstractTask::TerminationMessage
        run (std::shared_ptr<AbstractTask> task,
             const char *plan_file = nullptr);

      private:
        float cameraAngle = M_PI / 2;
        void checkTarget (Robot &robot, std::shared_ptr<AbstractTask> task);
        std::shared_ptr<b2World> world;
        float theta = 0;
        b2Transform start = b2Transform_zero;
        std::optional<b2Vec2> targetPos;
    };

    /**
     * @brief On task terminated needs to be filled with life when inheriting this
     * base constructor. It needs to decide which task is next!
     */
    virtual void onTaskTerminated (AbstractTask::TerminationMessage tm) = 0;

  private:
    AbstractTask::MotorEvent motorEvent;
    std::shared_ptr<AbstractTask> currentTask = std::make_shared<EndTask> ();
};

/**
 * @brief The simplest configurator: a reactive one.
 * The reactive configurator won't use the simulation engine and directly
 * decides which taks is next without imagining what might happen in the future.
 * This implements "onTaskTerminated" which is called after a task has
 * terminated, decides which task is next and then sets a new current task.
 */
class ReactiveConfigurator : public BaseConfigurator
{
  public:
    /**
     * @brief Turns termination reason into a task.
     * Translates termination reasons from the termination message into
     * a new task. It constructs that new task and returns it.
     * This method is static so that unit tests can test it without
     * creating an instance of the configurator.
     **/
    static std::unique_ptr<AbstractTask>
    terminationMessage2task (AbstractTask::TerminationMessage tm)
    {
        bratlogger.log ("Reactive: onTaskTerminated");
        switch (tm.terminationReason)
        {
        case AbstractTask::TerminationMessage::TargetDetected:
            bratlogger.log (
                "ReactiveConfigurator: Target detected. We are targeting.");
            return std::make_unique<TargetTask> (tm.taskWhichSentMessage);
        case AbstractTask::TerminationMessage::ObstacleDetected:
        {
            bratlogger.log ("ReactiveConfigurator: Obstacle detected.");
            const AbstractTask::Threats thr = tm.taskWhichSentMessage->getThreats ();
            if (thr.nLeft > thr.nRight)
            {
                bratlogger.log ("ReactiveConfigurator: Avoidance to the "
                                "RIGHT b/c obstacle to the left.");
                return std::make_unique<AvoidTask> (
                    tm.taskWhichSentMessage, AvoidTask::AvoidDirecton::RIGHT);
            }
            else
            {
                bratlogger.log ("ReactiveConfigurator: Avoidance to the "
                                "LEFT b/c obstacle to the right.");
                return std::make_unique<AvoidTask> (
                    tm.taskWhichSentMessage, AvoidTask::AvoidDirecton::LEFT);
            }
        }
        break;
        case AbstractTask::TerminationMessage::ObstacleAvoided:
            bratlogger.log ("ReactiveConfigurator: Obstacle avoided. Now "
                            "targeting again.");
            return std::make_unique<TargetTask> (tm.taskWhichSentMessage);
        case AbstractTask::TerminationMessage::TargetReached:
            bratlogger.log ("ReactiveConfigurator: TargetReached: stopping.");
            return std::make_unique<EndTask> (tm.taskWhichSentMessage);
        case AbstractTask::TerminationMessage::INVALID:
            bratlogger.log (
                "ReactiveConfigurator: Message==INVALID (BUG?): stopping.");
            return std::make_unique<EndTask> (tm.taskWhichSentMessage);
        }
        bratlogger.log (
            "ReactiveConfigurator: Message==??? (BUG?): stopping.");
        return std::make_unique<EndTask> (tm.taskWhichSentMessage);
    }

    /**
     * @brief Max number of tasks in a sequence. 
     * If it's more than max tasks the configurator will stop the robot with a EndTask.
     */
    void setMaxTasks (int mt) { maxTasks = mt; }

  protected:
    /**
     * @brief Event: the current task has signalled that it's terminated.
     * Create a new task if the number of max tasks hasn't been reached.
     */
    virtual void
    onTaskTerminated (AbstractTask::TerminationMessage tm) override
    {
        if (nTasks > maxTasks)
        {
            setCurrentTask (std::make_shared<EndTask> ());
            bratlogger.log ("Reached max tasks. Stopping!");
        }
        setCurrentTask (terminationMessage2task (tm));
    }

  private:
    int maxTasks = 0xfffffff;
    int nTasks = 1;
};
