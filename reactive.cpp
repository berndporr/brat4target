#include "LSM6DSOX.h"
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
        fprintf (stderr, "%s [number of tasks generated]\n", argv[0]);
        return -1;
    }
    const int nTasks = atoi (argv[1]);

    bratlogger.start ("/tmp", "reactive");

    C1Lidar lidar;
    TargetLoc targetLoc;
    ReactiveConfigurator configurator;
    ZetaBot zetabot;
    LSM6DSOX lsm6dS0x;
    lsm6dS0x.setGyroSamplingRate (GyroSettings::GYRO_ODR_12_5);
    const float gyrofs = 12.5;

    lsm6dS0x.registerCallback ([&] (const LSM6DSOXSample &s) {
        configurator.onGyroTurn (-s.gz / gyrofs);
    });

    configurator.registerMotorEvent ([&] (float l, float r) {
        zetabot.setLeftWheelSpeed (l);
        zetabot.setRightWheelSpeed (r);
        bratlogger.log ("Motor out: %f,%f\n", l, r);
    });

    configurator.setMaxTasks (nTasks);

    targetLoc.registerDetectionEvent ([&] (const float r, const float phi) {
        bratlogger.log ("Target detected:%f,%f\n", r, phi);
        configurator.onTargetDetected (r, phi);
    });

    WorldClusterBuilder worldBuilder;
    long int nWorld = 0;
    worldBuilder.registerWorldReadyCallback (
        [&] (std::shared_ptr<b2World> world) {
            bratlogger.log ("nWorld = %ld", nWorld);
            configurator.onLIDARworld (world);
            char tmp[256];
            sprintf (tmp, "/tmp/reactive_bodies%05ld.svg", nWorld);
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
    lsm6dS0x.start ();

    // We are starting from the stoptask
    configurator.setCurrentTask (std::make_shared<StopTask> ());

    printf ("Up and running.\n");

    // waiting for a keypress
    getchar ();

    printf ("Stopping.\n");

    lsm6dS0x.stop ();
    targetLoc.stop ();
    lidar.stop ();
    worldBuilder.stop ();
    zetabot.stop ();

    bratlogger.log ("END");

    return 0;
}
