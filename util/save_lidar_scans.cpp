#include "c1lidarrpi.h"
#include <iostream>
#include <chrono>   // For timing

class DataPrinter {
  int scanNo = 0;
public:
  void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]) {
    char tmp[256];
    sprintf(tmp,"/tmp/scan%05d.tsv",scanNo);
    FILE* datafile = fopen(tmp,"wt");
    for (C1LidarData &d : data) {
      if (d.valid) {  // Only process valid data
	fprintf(datafile,
		"%f\t%f\t%f\n",
		d.x,d.y,d.signal_strength);
      }
    }
    fclose(datafile);
    scanNo++;
    fprintf(stderr,".");
    fflush(stderr);
  }
};

int main(int, char **) {
    fprintf(stderr, "Data format: x <tab> y <tab> strength\n");

    C1Lidar lidar;
    DataPrinter dataPrinter;
    lidar.registerCallbackFunction([&](C1LidarData (&data)[C1Lidar::nDistance]){dataPrinter.newScanAvail(data);});

    try {
        lidar.start(C1Lidar::RPI_SERIAL_DEV);  // Start the LIDAR
    } catch (const char* msg) {
        std::cerr << "ERROR: " << msg << std::endl;
        lidar.stop(); // Make sure motor and scan stop
        return 1;
    }

    fprintf(stderr, "Press enter to stop.\n");

    // waiting for a keypress
    getchar();
    
    lidar.stop();  // Stop the scanning and motor

    std::cerr << "\nStopped cleanly." << std::endl;
    return 0;
}
