#include "const.h"
#include "robot.h"
#include "task.h"
#include "worldbuilder.h"
#include <memory>

constexpr float target_angle = 0.45;

CoordinateContainer loadLIDARdata (const char *filename)
{
    FILE *f = fopen (filename, "rt");
    CoordinateContainer cc;
    float x, y, q;
    while (fscanf (f, "%f\t%f\t%f\n", &x, &y, &q) > 1)
    {
        cc.push_back ({ x, y });
    };
    fclose (f);
    return cc;
}

void test_target_task_close ()
{
    printf ("Target close test.\n");
    TargetTask task;
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG ("/tmp/test_targettask_close.svg");
    task.registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    task.registerTerminated ([&] (AbstractTask::TerminationMessage tm) {
        printf ("Task terminated. Reason: %d\n", tm.terminationReason);
    });
    std::shared_ptr<Robot> robot = std::make_shared<Robot> (world);
    task.onLIDARworld (world, robot);
    printf ("\n");
}

void test_avoid_task_close ()
{
    printf ("Avoid test close.\n");
    AvoidTask task;
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG ("/tmp/close_obst_far_target.svg");
    task.registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    task.registerTerminated ([&] (AbstractTask::TerminationMessage tm) {
        printf ("Task terminated. Reason: %d\n", tm.terminationReason);
    });
    std::shared_ptr<Robot> robot = std::make_shared<Robot> (world);
    task.setLinearSpeed (0.5);
    task.onLIDARworld (world, robot);
    printf ("\n");
}

void test_target_task_far ()
{
    printf ("Target far test.\n");
    TargetTask task;
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG ("/tmp/test_targettask_far.svg");
    task.registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    task.registerTerminated ([&] (AbstractTask::TerminationMessage tm) {
        printf ("Task terminated. Reason: %d\n", tm.terminationReason);
    });
    std::shared_ptr<Robot> robot = std::make_shared<Robot> (world);
    task.setLinearSpeed (0.5);
    task.onLIDARworld (world, robot);
    task.onTargetAngle (target_angle);
    printf ("\n");
}

int main (int, char **)
{
    test_target_task_close ();
    test_target_task_far ();
    test_avoid_task_close ();
}
