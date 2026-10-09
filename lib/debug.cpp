#include "debug.h"
#include <box2d/b2_math.h>
#include <box2d/b2_world.h>
#include <cstdio>
#include <opencv2/core/mat.hpp>

const std::string Logger::dateTime ()
{
    time_t now = time (0);
    tm *ltm = localtime (&now);
    int y, m, d, h, min;
    y = ltm->tm_year - 100;
    m = ltm->tm_mon + 1;
    d = ltm->tm_mday;
    h = ltm->tm_hour;
    min = ltm->tm_min;
    char name[256];
    sprintf (name, "%02i%02i%02i_%02i%02i", d, m, y, h, min);
    return std::string (name);
}

void Logger::log (const char *format, ...)
{
    try
    {
        if (DEBUG)
        {
            va_list args;
            va_start (args, format);
            vfprintf (stderr, format, args);
            va_end (args);
            fprintf (stderr, "\n");
            fflush (stderr);
        }
        if (f)
        {
            va_list args;
            va_start (args, format);
            vfprintf (f, format, args);
            va_end (args);
            fprintf (f, "\n");
            fflush (f);
        }
    }
    catch (std::exception &e)
    {
        f = nullptr;
        fprintf(stderr,"Logger exeption: %s\n",e.what());
    }
}

void Logger::start (const std::string _dir, std::string _newFolder,
                    const std::string _filename)
{
    if (!opendir (_dir.c_str ()))
    {
        mkdir (_dir.c_str (), 0777);
    }
    if (_newFolder.empty ())
    {
        fileName = _dir + "/" + _filename;
    }
    else
    {
        std::string new_path = _dir + "/" + _newFolder;
        if (!opendir (new_path.c_str ()))
        {
            mkdir (new_path.c_str (), 0777);
        }
        fileName = new_path + "/" + _filename;
    }
    f = fopen (fileName.c_str (), "wt");
    if (!f)
    {
        std::cerr << "cannot open file " << fileName << std::endl;
        throw;
    }
}

void Logger::stop ()
{
    if (NULL != f)
    {
        fclose (f);
    }
    f = NULL;
}

const std::string Logger::getSystemArchitecture ()
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
