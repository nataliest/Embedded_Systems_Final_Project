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
	while (!mpuInterrupt && fifoCount < packetSize);
	
	uint16_t photo = photoVal();
	float imu = imuVal();

	debug(photo);
	debug("\t");
	debug(imu);
	debug("\t");
	mapAndSendData(photo, imu);
	debugln();
}
