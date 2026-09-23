#include "c1lidarrpi.h"
#include "lib/configurator.h"
#include "lib/const.h"
#include "lib/targetloc.h"
#include "lib/worldbuilder.h"
#include "zetabot.h"
#include <iostream>

int main (int, char **)
{
    printf ("->>> Avoid obstacle & drive to target:\n");

    C1Lidar lidar;
    TargetLoc targetLoc;
    Configurator configurator;
    ZetaBot zetabot;

    configurator.registerMotorEvent ([&] (float l, float r) {
       zetabot.setLeftWheelSpeed (l);
       zetabot.setRightWheelSpeed (r); 
    });

    targetLoc.registerDetectionEvent ([&] (const float r, const float phi) {
        configurator.onTargetDetected (r, phi);
    });

    WorldClusterBuilder worldBuilder;
    worldBuilder.registerWorldReadyCallback (
        [&] (std::shared_ptr<b2World> world, WorldBuilder::SpeedResult sr) {
            configurator.onLIDARworld (world);
            worldBuilder.exportWorldToSVG (
                "/tmp/test_configur_close_obst_far_target_bodies2.svg");
            worldBuilder.bodies_dump (
                "/tmp/test_configur_close_obst_far_target_bodies2.tsv");
        });

    lidar.registerCallbackFunction (
        [&] (C1LidarData (&data)[C1Lidar::nDistance]) {
            CoordinateContainer coords;
            for (const C1LidarData &d : data)
            {
                if (d.valid)
                { // Only process valid data
                    coords.push_back ({ d.x, d.y });
                }
            }
            worldBuilder.doAsyncBuildWorld (coords, b2Transform_zero);
        });

    try
    {
        zetabot.start ();
    }
    catch (const char *tmp)
    {
        fprintf (stderr, "\nZetabot error: %s\n", tmp);
        return 1;
    }

    try
    {
        lidar.start (C1Lidar::RPI_SERIAL_DEV); // Start the LIDAR
    }
    catch (const char *msg)
    {
        std::cerr << "ERROR: " << msg << std::endl;
        lidar.stop (); // Make sure motor and scan stop
        zetabot.stop();
        return 1;
    }

    targetLoc.start ();

    printf ("->>> Drive to target:\n");
    printf ("\n");

    // waiting for a keypress
    getchar ();

    std::cerr << "\nStopped cleanly." << std::endl;
    lidar.stop (); // Stop the scanning and motor
    return 0;
}
