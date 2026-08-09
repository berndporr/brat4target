#include "worldbuilder.h"
#include <memory>

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

void test_worldbuilder (std::shared_ptr<WorldBuilder> worldbuilder,
                        const char *bodiesFilename,
                        const char *objectsFilename, const char *svgfilename)
{
    CoordinateContainer coords = loadLIDARdata ("lidar_scans/far_target.tsv");

    b2Transform start = b2Transform_zero;

    std::shared_ptr<b2World> world = worldbuilder->buildWorld (coords, start);

    worldbuilder->bodies_dump (bodiesFilename);
    worldbuilder->object_dump (objectsFilename);
    worldbuilder->exportWorldToSVG (svgfilename);
}

int main (int, char **)
{
    test_worldbuilder (std::make_shared<WorldBuilder> (),
                       "/tmp/world_default_bodies.tsv",
                       "/tmp/world_default_obj.tsv", "/tmp/world_default.svg");
    test_worldbuilder (std::make_shared<WorldClusterBuilder> (),
                       "/tmp/world_cluster_bodies.tsv",
                       "/tmp/world_cluster_obj.tsv", "/tmp/world_cluster.svg");
    test_worldbuilder (std::make_shared<FocusedBuilder> (),
                       "/tmp/world_focused_bodies.tsv",
                       "/tmp/world_focused_obj.tsv", "/tmp/world_focused.svg");
}
