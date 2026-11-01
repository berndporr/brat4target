#include "configurator.h"
#include "debug.h"
#include "lib/const.h"
#include "task.h"
#include <memory>

void BaseConfigurator::TaskSimulator::checkTarget (
    std::shared_ptr<AbstractTask> task)
{
    if (!targetPos->IsValid ())
        return;
    b2Vec2 robotPos = robot->body ()->GetPosition ();
    b2Vec2 delta = targetPos.value () - robotPos;
    // Transform world delta vector into robot's local space
    b2Vec2 localDelta = b2MulT (robot->body ()->GetTransform ().q, delta);

    // Calculate relative angle (-PI to +PI). 0 is dead ahead.
    float relativeTargetAngle = b2Atan2 (localDelta.y, localDelta.x);
    if (DEBUG)
    {
        fprintf (stderr, "Relative target angle: %f\n", relativeTargetAngle);
    }
    if (fabs (relativeTargetAngle) > cameraAngle)
    {
        if (DEBUG)
        {
            fprintf (stderr, "SIM: target out of sight: %f > %f\n",
                     relativeTargetAngle, cameraAngle);
        }
        return;
    }
    if (DEBUG)
    {
        fprintf (stderr, "SIM: Target det at r=%f, phi=%f\n",
                 localDelta.Length (), relativeTargetAngle);
    }
    task->onTargetDetected (localDelta.Length (), relativeTargetAngle);
}

AbstractTask::TerminationMessage
BaseConfigurator::TaskSimulator::run (std::shared_ptr<AbstractTask> task,
                                      const char *plan_file)
{
    FILE *robotPath = nullptr;
    AbstractTask::TerminationMessage simResult;
    bool running = true;
    task->registerTerminated ([&] (AbstractTask::TerminationMessage tm) {
        running = false;
        simResult = tm;
        bratlogger.log ("BaseConfigurator::TaskSimulator::run Simulation terminating.");
    });
    if (plan_file)
    {
        robotPath = fopen (plan_file, "wt");
    }
    // needs to be called before it can be used. Sets for example the wheel speed.
    task->onStart ();
    bool out = false;
    for (stepb2d = 0; running && (!out); stepb2d++)
    {
        task->onLIDARworld (world, robot);
        task->onGyroTurn (task->getangularVelocity () / HZ);
        checkTarget (task);
        instVelocity.x = task->getLinearSpeed () * cos (theta);
        instVelocity.y = task->getLinearSpeed () * sin (theta);
        robot->body ()->SetLinearVelocity (instVelocity);
        robot->body ()->SetAngularVelocity (task->getangularVelocity ());
        robot->body ()->SetTransform (robot->body ()->GetPosition (), theta);
        if (robotPath)
        {
            fprintf (robotPath, "%f\t%f\t%f\t%f\t%f\t%f\n",
                     robot->body ()->GetPosition ().x,
                     robot->body ()->GetPosition ().y, instVelocity.x,
                     instVelocity.y, theta,
                     task->getangularVelocity () / HZ); //save predictions/
        }
        bool out_x = fabs (robot->body ()->GetTransform ().p.x)
                     >= (BOX2DRANGE - 0.001);
        bool out_y = fabs (robot->body ()->GetTransform ().p.y)
                     >= (BOX2DRANGE - 0.001);
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
    return simResult;
}
