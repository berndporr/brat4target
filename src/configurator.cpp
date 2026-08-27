#include "configurator.h"
#include "task.h"
#include "worldbuilder.h"
#include <memory>

Configurator::Simulator::Result
Configurator::Simulator::run (std::shared_ptr<AbstractTask> task,
                                   const char *plan_file)
{
    char collisionFile[50]; //debug
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
