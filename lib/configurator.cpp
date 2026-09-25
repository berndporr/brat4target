#include "configurator.h"
#include "debug.h"
#include "task.h"
#include "worldbuilder.h"
#include <memory>

void Configurator::Simulator::checkTarget (std::shared_ptr<AbstractTask> task)
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

Configurator::Simulator::Result
Configurator::Simulator::run (std::shared_ptr<AbstractTask> task,
                              const char *plan_file)
{
    FILE *robotPath = nullptr;
    Result simResult;
    simResult.endPose = start;
    float remaining = remainingSimulationTime (task);
    bool running = true;
    task->registerTerminated ([&] (AbstractTask::TerminationMessage tm) {
        running = false;
        simResult.terminationMessage = tm;
    });
    if (plan_file)
    {
        robotPath = fopen (plan_file, "wt");
    }
    bool out = false;
    for (stepb2d = 0; (stepb2d < (HZ * remaining)) && running && (!out);
         stepb2d++)
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
    simResult.endPose = robot->body ()->GetTransform ();
    simResult.step = stepb2d;
    if (robotPath)
    {
        fclose (robotPath);
    }
    return simResult;
}

float Configurator::Simulator::remainingSimulationTime (
    std::shared_ptr<AbstractTask> task)
{
    float distance = BOX2DRANGE;
    return distance / task->getLinearSpeed ();
}
