#define DEBUG
#include "Internals.h"

void setup()
{
	initializeSerial();
	initializePhoto();
	calibratePhoto();
	initializeOthers();
	calibrateIMU();
	init_radio_controller();
}

void loop()
{
	uint16_t photo = photoVal();
	float imu = imuVal();

	debug(photo);
	debug("\t");
	debugln(imu);
	mapAndSendData(photo, imu);
}
