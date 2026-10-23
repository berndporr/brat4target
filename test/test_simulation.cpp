#include "configurator.h"
#include "const.h"
#include "task.h"
#include "worldbuilder.h"
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
    BaseConfigurator::Simulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    simulator.run (task, "/tmp/test_configur_far_target_sim.tsv");
    printf ("\n");
}

void avoid_obstacle (AbstractTask::AvoidDirecton ad)
{
    printf ("Avoid obstacle:\n");
    auto task = std::make_shared<AvoidTask> ();
    task->setLinearSpeed (0.5);
    task->setAvoidDirection (ad);
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
    BaseConfigurator::Simulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    char tmp[256];
    sprintf (tmp, "/tmp/test_configur_close_obst_far_target_sim_dir_ad%d.tsv",
             ad);
    simulator.run (task, tmp);
    printf ("\n");
}

void avoid_obstacle_drive_to_arget (AbstractTask::AvoidDirecton ad)
{
    printf ("->>> Avoid obstacle & drive to target:\n");
    auto taskAvoid = std::make_shared<AvoidTask> ();
    taskAvoid->setLinearSpeed (0.5);
    taskAvoid->setAvoidDirection (ad);
    CoordinateContainer coords
        = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG (
        "/tmp/test_configur_close_obst_far_target_bodies_seq.svg");
    worldBuilder.bodies_dump (
        "/tmp/test_configur_close_obst_far_target_bodies_seq.tsv");
    taskAvoid->registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    BaseConfigurator::Simulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    printf ("->Avoid obstacle:\n");
    char tmp[256];
    sprintf (tmp,
             "/tmp/test_configur_close_obst_far_target_sim_ad%d_seq_avoid.tsv",
             ad);
    simulator.run (taskAvoid, tmp);
    auto taskTarget = std::make_shared<TargetTask> ();
    taskTarget->init (taskAvoid);
    printf ("->>> Drive to target:\n");
    sprintf (
        tmp,
        "/tmp/test_configur_close_obst_far_target_sim_ad%d_seq_target.tsv",
        ad);
    simulator.run (taskTarget, tmp);
    printf ("\n");
}

int main (int, char **)
{
    drive_to_target ();
    avoid_obstacle (AbstractTask::LEFT);
    avoid_obstacle (AbstractTask::RIGHT);
    avoid_obstacle_drive_to_arget (AbstractTask::LEFT);
    avoid_obstacle_drive_to_arget (AbstractTask::RIGHT);
}
