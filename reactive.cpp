#include "src/configurator.h"
#include "src/const.h"
#include "src/task.h"
#include <memory>

constexpr float target_angle = 0.44;
constexpr float target_distance = 0.93;

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

void drive_to_target ()
{
    printf ("Drive to target:\n");
    auto task = std::make_shared<TargetTask> ();
    task->setLinearSpeed (0.5);
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG ("/tmp/test_configur_far_target_bodies.svg");
    worldBuilder.bodies_dump ("/tmp/test_configur_far_target_bodies.tsv");
    task->registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    Configurator::Simulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    simulator.run (task, "/tmp/test_configur_far_target_sim.tsv");
    printf ("\n");
}

void avoid_obstacle ()
{
    printf ("Avoid obstacle:\n");
    auto task = std::make_shared<AvoidTask> ();
    task->setLinearSpeed (0.5);
    CoordinateContainer coords
        = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG (
        "/tmp/test_configur_close_obst_far_target_bodies.svg");
    worldBuilder.bodies_dump (
        "/tmp/test_configur_close_obst_far_target_bodies.tsv");
    task->registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    Configurator::Simulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    simulator.run (task, "/tmp/test_configur_close_obst_far_target_sim.tsv");
    printf ("\n");
}

void avoid_obstacle_drive_to_arget ()
{
    printf ("->>> Avoid obstacle & drive to target:\n");
    auto taskAvoid = std::make_shared<AvoidTask> ();
    taskAvoid->setLinearSpeed (0.5);
    CoordinateContainer coords
        = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG (
        "/tmp/test_configur_close_obst_far_target_bodies2.svg");
    worldBuilder.bodies_dump (
        "/tmp/test_configur_close_obst_far_target_bodies2.tsv");
    taskAvoid->registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    Configurator::Simulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    printf ("->Avoid obstacle:\n");
    simulator.run (taskAvoid,
                   "/tmp/test_configur_close_obst_far_target_sim2avoid.tsv");
    auto taskTarget = std::make_shared<TargetTask> ();
    taskTarget->init (taskAvoid);
    printf ("->>> Drive to target:\n");
    simulator.run (taskTarget,
                   "/tmp/test_configur_close_obst_far_target_sim2target.tsv");
    printf ("\n");
}

int main (int, char **)
{
    drive_to_target ();
    avoid_obstacle ();
    avoid_obstacle_drive_to_arget ();
}
