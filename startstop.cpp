#include "LSM6DSOX.h"
#include "c1lidarrpi.h"
#include "lib/configurator.h"
#include "lib/targetloc.h"
#include "lib/worldbuilder.h"
#include "zetabot.h"
#include <iostream>
#include <unistd.h>

int main (int nargs, char **argv)
{
    printf ("->>> Start&stop.\n");

    C1Lidar lidar;
    TargetLoc targetLoc;
    Configurator configurator;
    ZetaBot zetabot;
    LSM6DSOX lsm6dS0x;

    try
    {
        zetabot.start ();
    }
    catch (const char *tmp)
    {
        fprintf (stderr, "\n!!!! Zetabot error: %s\n", tmp);
        return 1;
    }

    try
    {
        lidar.start (C1Lidar::RPI_SERIAL_DEV); // Start the LIDAR
    }
    catch (const char *msg)
    {
        std::cerr << "LIDAR ERROR: " << msg << std::endl;
        lidar.stop (); // Make sure motor and scan stop
        zetabot.stop ();
        return 1;
    }

    targetLoc.start ();
    lsm6dS0x.start ();

    printf ("Up and running.\n");

    sleep(1);

    printf ("Stopping.\n");

    lsm6dS0x.stop ();
    targetLoc.stop ();
    lidar.stop ();
    zetabot.stop ();

    return 0;
}
