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
    printf ("->>> Reactive configurator test:\n");
    CoordinateContainer coords
        = loadLIDARdata ("lidar_scans/close_obst_far_target.tsv");
    WorldClusterBuilder worldBuilder;
    std::shared_ptr<b2World> world
        = worldBuilder.buildWorld (coords, b2Transform_zero);
    worldBuilder.exportWorldToSVG (
        "/tmp/test_configur_close_obst_far_target_bodies_seq.svg");
    worldBuilder.bodies_dump (
        "/tmp/test_configur_close_obst_far_target_bodies_seq.tsv");
    BaseConfigurator::TaskSimulator simulator (world);
    simulator.setTarget (target_distance, target_angle);
    std::shared_ptr<AbstractTask> task = std::make_shared<StopTask> ();
    task->registerMotorEvent (
        [&] (const float l, const float r) {
            printf ("Motor event: %f,%f\n", l, r);
        });
    for (int i = 0; i < 3; i++)
    {
        char tmp[256];
        sprintf (tmp, "/tmp/test_reactive_configur_simseq%d_target.tsv", i);
        auto msg = simulator.run (task, tmp);
        task = ReactiveConfigurator::terminationMessage2task (msg);
    }
}

int main (int, char **) { avoid_obstacle_drive_to_arget (); }
