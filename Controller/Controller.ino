// #define OPTIMIZED
#define DEBUG
#include "Internals.h"

void setup()
{
	initialize();
    calibrateIMU();
}

void loop()
{
	// if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // debug(imuVal());
    // debug("\n");
}	
