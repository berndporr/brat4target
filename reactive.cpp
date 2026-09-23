#include "c1lidarrpi.h"
#include <iostream>
#include <chrono>   // For timing

class Reactive {
public:
	void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]) {
	    for (C1LidarData &d : data) {
		if (d.valid) {  // Only process valid data
		    std::cout << d.x << "\t" << d.y 
			      << "\t" << d.r << "\t" << d.phi 
			      << "\t" << d.signal_strength << std::endl;
		}
	    }
	}
};

int main(int, char **) {
    C1Lidar lidar;
    Reactive reactive;
    lidar.registerCallbackFunction([&](C1LidarData (&data)[C1Lidar::nDistance]){reactive.newScanAvail(data);});

    try {
        lidar.start(C1Lidar::RPI_SERIAL_DEV);  // Start the LIDAR
    } catch (const char* msg) {
        std::cerr << "ERROR: " << msg << std::endl;
        lidar.stop(); // Make sure motor and scan stop
        return 1;
    }

    // waiting for a keypress
    getchar();
    
    std::cerr << "\nStopped cleanly." << std::endl;
    lidar.stop();  // Stop the scanning and motor
    return 0;
}
