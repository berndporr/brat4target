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

void avoid_obstacle_drive_to_arget ()
{
    printf ("->>> Avoid obstacle & drive to target:\n");
    auto task = std::make_shared<StopTask> ();
    task->setLinearSpeed (0.5);
    CoordinateContainer coords
        = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG (
        "/tmp/test_configur_close_obst_far_target_bodies_seq.svg");
    worldBuilder.bodies_dump (
        "/tmp/test_configur_close_obst_far_target_bodies_seq.tsv");
    task->registerMotorEvent ([&] (const float l, const float r) {
        printf ("Motor event: %f,%f\n", l, r);
    });
    ReactiveConfigurator reactiveConfigurator;
    reactiveConfigurator.planner (world, target_distance, target_angle);
}

int main (int, char **) { avoid_obstacle_drive_to_arget (); }
