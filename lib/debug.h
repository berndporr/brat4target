#pragma once
#include <bits/stdc++.h>
#include <box2d/b2_body.h>
#include <box2d/b2_math.h>
#include <chrono>
#include <cstddef>
#include <dirent.h>
#include <opencv2/core/mat.hpp>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

static constexpr bool DEBUG = true;

/**
 * @brief Class used to load data from the configurator
 * 
 */
class Logger
{
  public:
    Logger () = default;

    /**
	 * @brief Construct a new Logger object
	 * 
	 * @param _dir directory containing new_folder
	 * @param _newFolder folder where files will be dumped (no / at the end)
	 * @param _filename the filename of the log file
	 */
    void start (const std::string _dir = "/tmp", std::string _newFolder = {},
                const std::string _filename = dateTime () + ".txt");

    void stop ();

    ~Logger () { stop (); }

    /**
	  * @brief 
	  * 
	  * @param format printf style e.g. "hello%s"
	  * @param ... other parameters
	  */
    void log (const char *format, ...);

    // If we log a variable then we end the line of the logfile with "<<<VAR"
    const char *variableindicator = "<<<VAR";

    // Logging an integer variable
    void logVariable (const char *variable, int val)
    {
        log ("%s\t%d%s", variable, val, variableindicator);
    }

    // Logging a float variable
    void logVariable (const char *variable, float val)
    {
        log ("%s\t%f%s", variable, val, variableindicator);
    }

    const std::string get_fileName () const { return fileName; };

    /**
	  * @brief Returns a string with system architecture
	  */
    static const std::string getSystemArchitecture ();

    /**
	 * @brief Creates filename with today's date and time, name in format customdmy_hm.txt
	 * 
	 * @param custom custom
	 * @param name empty char array
	 */
    static const std::string dateTime ();

  private:
    // ms since the start of the program
    std::string get_elapsed_ms ();
    std::string fileName;
    FILE *f = NULL;
    const std::chrono::time_point<std::chrono::steady_clock> program_start
        = std::chrono::steady_clock::now ();
};

// global logger
inline Logger bratlogger;
