#define DEBUG
#include "Internals.h"
uint16_t data = 0;

void setup()
{
	initialize();
	calibrateIMU();
//Serial.begin(115200);
//init_radio_controller();
}

void loop()
{
//Serial.println(++data);
//radio.write(&data, sizeof(uint16_t));


  
	uint16_t photo = photoVal();
	float imu = imuVal();

	debug(photo);
	debug("\t");
	debug(imu);
	debug("\t");
	//mapAndSendData(photo, imu);
	debugln();
}
