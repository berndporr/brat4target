#include "const.h"
#include "worldbuilder.h"
#include <memory>
#include <unistd.h>

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

void test_worldbuilder_sync (std::shared_ptr<WorldBuilder> worldbuilder,
                             const char *bodiesFilename,
                             const char *objectsFilename,
                             const char *svgfilename)
{
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/far_target.tsv");

    b2Transform start = b2Transform_zero;

    std::shared_ptr<b2World> world = worldbuilder->buildWorld (coords, start);

    worldbuilder->bodies_dump (bodiesFilename);
    worldbuilder->object_dump (objectsFilename);
    worldbuilder->exportWorldToSVG (svgfilename);
}

void test_worldbuilder_async (std::shared_ptr<WorldBuilder> worldbuilder,
                              const char *bodiesFilename,
                              const char *svgfilename)
{
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/far_target.tsv");

    b2Transform start = b2Transform_zero;

    int n = 0;

    worldbuilder->registerWorldReadyCallback (
        [&] (std::shared_ptr<b2World> world, WorldBuilder::SpeedResult sr) {
            WorldBuilder::bodies_dump (world, bodiesFilename);
            WorldBuilder::exportWorldToSVG (world, svgfilename);
            n++;
            printf ("World %d built.\n",n);
        });
    for (int i = 0; i < 10; i++)
    {
        worldbuilder->doAsyncBuildWorld (coords, b2Transform_zero);
        usleep (1e3);
    }
    n = 0;
    for (int i = 0; i < 10; i++)
    {
        worldbuilder->doAsyncBuildWorld (coords, b2Transform_zero);
        usleep (100e3);
    }
    worldbuilder->wait4Async ();
}

int main (int, char **)
{
    test_worldbuilder_sync (
        std::make_shared<WorldBuilder> (), "/tmp/world_default_bodies.tsv",
        "/tmp/world_default_obj.tsv", "/tmp/world_default.svg");
    test_worldbuilder_sync (std::make_shared<WorldClusterBuilder> (),
                            "/tmp/world_cluster_bodies.tsv",
                            "/tmp/world_cluster_obj.tsv",
                            "/tmp/world_cluster.svg");
    test_worldbuilder_sync (
        std::make_shared<FocusedBuilder> (), "/tmp/world_focused_bodies.tsv",
        "/tmp/world_focused_obj.tsv", "/tmp/world_focused.svg");
    test_worldbuilder_async (std::make_shared<WorldClusterBuilder> (),
                             "/tmp/async_world_cluster_bodies.tsv",
                             "/tmp/async_world_cluster_bodies.svg");
}
