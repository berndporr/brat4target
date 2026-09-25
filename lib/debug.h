#pragma once
#include <bits/stdc++.h>
#include <box2d/b2_body.h>
#include <box2d/b2_math.h>
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
  protected:
    char fileName[60];
    FILE *f = NULL;
    int fileCount = 0; //files with the same name

  public:
    Logger () = default;

    /**
	 * @brief Construct a new Logger object
	 * 
	 * @param new_folder folder where files will be dumped (no / at the end)
	 * @param _dir directory containing new_folder
	 * @param customName file prefix (/ must be at the beginning)
	 * @param dateOn whether to add today's date and time to file name
	 */
    Logger (const char *new_folder, const char *_dir = "/tmp",
            const char *customName = "/stats", bool dateOn = true)
    {
        init (new_folder, _dir, customName, dateOn);
    }

    ~Logger ()
    {
        if (NULL != f)
        {
            fclose (f);
        }
        f = NULL;
    }

    /**
	 * @brief 
	 * 
	 * @param format printf style e.g. "hello%s"
	 * @param ... other parameters
	 */
    bool log (const char *format, ...);

    const char *get_fileName () { return fileName; }

    /**
	 * @brief Returns a string with system architecture
	 */
    static const char *getSystemArchitecture ();

  protected:
    /**
	 * @brief Creates filename with today's date and time, name in format customdmy_hm.txt
	 * 
	 * @param custom custom
	 * @param name empty char array
	 */
    std::string file_dateTime (const char *custom, char name[80]);

    /**
	 * @see Logger
	 */
    void init (const char *new_folder, const char *_dir = NULL,
               const char *customName = "/stats", bool dateOn = false);
};

namespace debug
{

b2Vec2 GetWorldPoints (b2Body *, b2Vec2);

void print_pose (const b2Transform &p, const char *msg = NULL);

void print_matrix (const cv::Mat &);

std::vector<b2Vec2> GetBodies (b2World *);

}
