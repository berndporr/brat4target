#include "debug.h"

int main (int, char **)
{
    bratlogger.start();
    bratlogger.log("This is a log test with the number %d.",42);
    bratlogger.log("This is another log test without a number.");
}
