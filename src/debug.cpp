#include "debug.h"
#include <box2d/b2_math.h>
#include <box2d/b2_world.h>
#include <opencv2/core/mat.hpp>

std::string Logger::file_dateTime (const char *custom, char name[80])
{
    time_t now = time (0);
    tm *ltm = localtime (&now);
    int y, m, d, h, min;
    y = ltm->tm_year - 100;
    m = ltm->tm_mon + 1;
    d = ltm->tm_mday;
    h = ltm->tm_hour;
    min = ltm->tm_min;
    //	struct stat buffer;

    sprintf (name, "%s_%02i%02i%02i_%02i%02i.txt", custom, d, m, y, h, min);
    // int count=0;
    // while (stat (fileName, &buffer)==0){
    // 	count++;
    // }
    return std::string (name);
}

bool Logger::log (const char *format, ...)
{
    try
    {
        va_list args;
        va_start (args, format);
        vfprintf (f, format, args);
        va_end (args);
        fflush (f);
        return true;
    }
    catch (std::exception &e)
    {
        return false;
    }
}

void Logger::init (const char *new_folder, const char *_dir,
                   const char *customName, bool dateOn)
{
    std::string dirName = _dir;
    if (!opendir (dirName.c_str ()))
    {
        mkdir (dirName.c_str (), 0777);
    }

    std::string new_path = dirName + "/" + new_folder;
    if (!opendir (new_path.c_str ()))
    {
        mkdir (new_path.c_str (), 0777); //""
    }
    std::string customfile = new_path + customName;
    if (dateOn)
    {
        file_dateTime (customfile.c_str (), fileName);
    }
    else
    {
        sprintf (fileName, "%s.txt", customfile.c_str ());
    }
    f = fopen (fileName, "a+");
    if (!f)
    {
        f = fopen (fileName, "w+");
    }
    if (!f)
    {
        std::cerr << "cannot open file " << fileName << std::endl;
        throw;
    }
}

const char *Logger::getSystemArchitecture ()
{
#if defined(__x86_64__) || defined(_M_X64)
    return "x86_64";
#elif defined(__i386__) || defined(_M_IX86)
    return "x86";
#elif defined(__aarch64__) || defined(_M_ARM64)
    return "ARM64";
#elif defined(__arm__) || defined(_M_ARM)
    return "ARM";
#elif defined(__ppc64__)
    return "PowerPC64";
#elif defined(__ppc__)
    return "PowerPC";
#else
    return "UnknownArchitecture";
#endif
}

b2Vec2 GetWorldPoints (b2Body *b, b2Vec2 v)
{
    b2Vec2 wp = b->GetWorldPoint (v);
    printf ("x=%f, y=%f\t", wp.x, wp.y);
    return wp;
}

