#include "configurator.h"
#include "debug.h"
#include "lib/const.h"
#include "lib/robot.h"
#include "task.h"
#include <memory>

void BaseConfigurator::TaskSimulator::checkTarget (
    Robot &robot, std::shared_ptr<AbstractTask> task)
{
    if (!targetPos->IsValid ())
        return;
    b2Vec2 robotPos = robot.getPosition ();
    b2Vec2 delta = targetPos.value () - robotPos;
    // Transform world delta vector into robot's local space
    b2Vec2 localDelta = b2MulT (robot.getTransform ().q, delta);

    // Calculate relative angle (-PI to +PI). 0 is dead ahead.
    float relativeTargetAngle = b2Atan2 (localDelta.y, localDelta.x);
    bratlogger.log ("SIM: Relative target angle: %f", relativeTargetAngle);
    if (fabs (relativeTargetAngle) > cameraAngle)
    {
        bratlogger.log ("SIM: target out of sight: %f > %f",
                        relativeTargetAngle, cameraAngle);
        return;
    }
    bratlogger.log ("SIM: Target det at r=%f, phi=%f\n", localDelta.Length (),
                    relativeTargetAngle);
    task->onTargetDetected (delta.Length (), relativeTargetAngle);
}

AbstractTask::TerminationMessage
BaseConfigurator::TaskSimulator::run (std::shared_ptr<AbstractTask> task,
                                      const char *plan_file)
{
    FILE *robotPath = nullptr;
    Robot robot (world, start);
    theta = start.q.GetAngle ();
    AbstractTask::TerminationMessage simResult;
    bool running = true;
    task->registerTerminated ([&] (AbstractTask::TerminationMessage tm) {
        running = false;
        simResult = tm;
        bratlogger.log (
            "BaseConfigurator::TaskSimulator::run Simulation terminating.");
    });
    if (plan_file)
    {
        robotPath = fopen (plan_file, "wt");
    }
    // needs to be called before it can be used. Sets for example the wheel speed.
    task->onStart ();
    bool out = false;
    int stepb2d = 0;
    for (; running && (!out); stepb2d++)
    {
        task->onLIDARworld (world, robot);
        task->onGyroTurn (task->getangularVelocity () / HZ);
        checkTarget (robot, task);
        b2Vec2 instVelocity{ task->getLinearSpeed () * cos (theta),
                             task->getLinearSpeed () * sin (theta) };
        robot.setLinearVelocity (instVelocity);
        robot.setAngularVelocity (task->getangularVelocity ());
        robot.setAngle (theta);
        if (robotPath)
        {
            fprintf (robotPath, "%f\t%f\t%f\t%f\t%f\t%f\n",
                     robot.getPosition ().x, robot.getPosition ().y,
                     instVelocity.x, instVelocity.y, theta,
                     task->getangularVelocity () / HZ); //save predictions/
        }
        bool out_x = fabs (robot.getTransform ().p.x) >= (BOX2DRANGE - 0.001);
        bool out_y = fabs (robot.getTransform ().p.y) >= (BOX2DRANGE - 0.001);
        out = (out_x || out_y);
        world->Step (
            1.0f / HZ, 3,
            8); //time step 100 ms which also is zetabot callback time, possibly put it higher in the future if fast
        theta += task->getangularVelocity () / HZ; //= omega *t
    }
    bratlogger.log ("BaseConfigurator::TaskSimulator::run: Termination status:"
                    " step2d = %d, running = %d, out = %d",
                    stepb2d, running, out);
    if (robotPath)
    {
        fclose (robotPath);
    }
    start = robot.getTransform ();
    robot.destruct ();
    return simResult;
}
