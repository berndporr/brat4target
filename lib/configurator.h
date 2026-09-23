#pragma once

#include "debug.h"
#include "robot.h"
#include "task.h"
#include "worldbuilder.h"
#include <algorithm>
#include <box2d/b2_math.h>
#include <dirent.h>
#include <memory>
#include <ncurses.h>
#include <optional>
#include <sys/stat.h>
#include <vector>

/** 
 * Cognitive map node.
 */
struct State
{
    std::vector<std::shared_ptr<State> > children;
    std::shared_ptr<AbstractTask> task;
};

class Configurator
{
  public:
    Configurator () = default;

    /**
     * Needs to react to the LIDAR world
     */
    virtual void onLIDARworld (std::shared_ptr<b2World> world, WorldBuilder::SpeedResult sr)
    {
        currentTask->onLIDARworld (world, std::make_shared<Robot> (world));
    }

    /**
     * Task needs to incorporate new target angle info from the camera
     */
    virtual void onTargetDetected (float r, float phi)
    {
        currentTask->onTargetDetected (r, phi);
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
    }

    std::shared_ptr<AbstractTask> getCurrentTask () { return currentTask; }

    /**
     * Registers the motor event callback which sets the wheel speeds of the
     * real robot.
     */
    void registerMotorEvent (AbstractTask::MotorEvent me) { motorEvent = me; }

    void setSimulationStep (float f) { simulationStep = f; }

    void register_logger (std::shared_ptr<Logger> l) { logger = l; }

    class Simulator
    {
      public:
        struct Result
        {
            AbstractTask::TerminationMessage terminationMessage;
            b2Transform endPose = b2Transform (b2Vec2 (0.0, 0.0), b2Rot (0));
            int step = 0;
            Result () = default;
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

        void setTarget (float r, float phi) {
            b2Vec2 t;
            t.x = r * cos(phi);
            t.y = r * sin(phi);
            targetPos = std::optional<b2Vec2>(t);
        }

        Configurator::Simulator::Result
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
    }

    AbstractTask::MotorEvent motorEvent;
    std::shared_ptr<AbstractTask> currentTask = std::make_shared<StopTask> ();
    std::shared_ptr<Logger> logger;
    float simulationStep = 2 * std::max (ROBOT_HALFLENGTH, ROBOT_HALFWIDTH);
    std::chrono::high_resolution_clock::time_point previousTimeScan;
};
