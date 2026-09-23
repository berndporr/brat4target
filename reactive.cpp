#include "c1lidarrpi.h"
#include "lib/configurator.h"
#include "lib/const.h"
#include "lib/debug.h"
#include "lib/targetloc.h"
#include "lib/worldbuilder.h"
#include "zetabot.h"
#include <iostream>

int main (int nargs, char **argv)
{
    printf ("->>> Reactive.\n");

    if (nargs < 2)
    {
        fprintf (stderr, "%s 0=no motor ctrl, 1=motor ctrl\n", argv[0]);
        return 1;
    }
    bool motorOutput = atoi (argv[1]) > 0;

    C1Lidar lidar;
    TargetLoc targetLoc;
    Configurator configurator;
    ZetaBot zetabot;

    long int nWorld = 0;

    configurator.registerMotorEvent ([&] (float l, float r) {
        if (motorOutput)
        {
            zetabot.setLeftWheelSpeed (l);
            zetabot.setRightWheelSpeed (r);
        }
        if (DEBUG)
            fprintf (stderr, "Motor out: %f,%f\n", l, r);
    });

    targetLoc.registerDetectionEvent ([&] (const float r, const float phi) {
        if (DEBUG)
            fprintf (stderr, "Detection: %f,%f\n", r, phi);
        configurator.onTargetDetected (r, phi);
    });

    WorldClusterBuilder worldBuilder;
    worldBuilder.registerWorldReadyCallback (
        [&] (std::shared_ptr<b2World> world, WorldBuilder::SpeedResult sr) {
            configurator.onLIDARworld (world, sr);
            char tmp[256];
            sprintf (tmp, "/tmp/reactive_bodies%05ld.svg", nWorld);
            worldBuilder.exportWorldToSVG (tmp);
            sprintf (tmp, "/tmp/reactive_bodies%05ld.tsv", nWorld);
            worldBuilder.bodies_dump (tmp);
            nWorld++;
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
        zetabot.stop ();
        return 1;
    }

    targetLoc.start ();

    printf ("Up and running.\n");

    // waiting for a keypress
    getchar ();

    printf ("Stopping.\n");

    zetabot.stop();
    targetLoc.stop();
    lidar.stop ();
    
    return 0;
}
