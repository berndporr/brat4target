#include "debug.h"

int main (int, char **)
{
    logger.start();
    logger.log("This is a log test with the number %d.",42);
    logger.log("This is another log test without a number.");
}
