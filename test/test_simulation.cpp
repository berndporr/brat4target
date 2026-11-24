#include "configurator.h"
#include "const.h"
#include "debug.h"
#include "task.h"
#include "worldbuilder.h"
#include <format>
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
    bratlogger.start ("/tmp", "sim", "drive2target.txt");
    auto task = std::make_shared<TargetTask> ();
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG ("/tmp/test_configur_far_target_bodies.svg");
    worldBuilder.bodies_dump ("/tmp/test_configur_far_target_bodies.tsv");
    task->registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    BaseConfigurator::TaskSimulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    simulator.run (task, "/tmp/test_configur_far_target_sim.tsv");
    printf ("\n");
}

void avoid_obstacle (AvoidTask::AvoidDirecton ad)
{
    printf ("Avoid obstacle:\n");
    auto filename = std::format ("avoid_obstacle{}.txt", (int)ad);
    bratlogger.start ("/tmp", "sim", filename);
    auto task = std::make_shared<AvoidTask> (ad);
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
    BaseConfigurator::TaskSimulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    filename = std::format (
        "/tmp/test_configur_close_obst_far_target_sim_dir_ad{}.tsv", (int)ad);
    simulator.run (task, filename.c_str());
    printf ("\n");
}

void avoid_obstacle_drive_to_arget (AvoidTask::AvoidDirecton ad)
{
    printf ("->>> Avoid obstacle & drive to target:\n");
    auto filename
        = std::format ("avoid_obstacle_drive_to_arget{}.txt", (int)ad);
    bratlogger.start ("/tmp/", "sim", filename);
    auto taskAvoid = std::make_shared<AvoidTask> (ad);
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
    BaseConfigurator::TaskSimulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    printf ("->Avoid obstacle:\n");
    filename = std::format (
        "/tmp/test_configur_close_obst_far_target_sim_ad{}_seq_avoid.tsv",
        (int)ad);
    simulator.run (taskAvoid, filename.c_str ());
    auto taskTarget = std::make_shared<TargetTask> (taskAvoid);
    printf ("->>> Drive to target:\n");
    filename = std::string (
        "/tmp/test_configur_close_obst_far_target_sim_ad{}_seq_target.tsv",
        (int)ad);
    simulator.run (taskTarget, filename.c_str ());
    printf ("\n");
}

int main (int, char **)
{
    drive_to_target ();
    avoid_obstacle (AvoidTask::LEFT);
    avoid_obstacle (AvoidTask::RIGHT);
    avoid_obstacle_drive_to_arget (AvoidTask::LEFT);
    avoid_obstacle_drive_to_arget (AvoidTask::RIGHT);
}
