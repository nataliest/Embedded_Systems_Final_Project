#define DEBUG
#include "Internals.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup()
{
	initializeSerial();
	initializePhoto();
	calibratePhoto();
	mag.begin();
	sensor_t sensor;
	mag.getSensor(&sensor);
	init_radio_controller();

	debugln("Calibrating magnetometer center. Place in center.");
	delay(250);
	debug("Calibrating in 3... ");
	delay(1000);
	debug("2... ");
	delay(1000);
	debugln("1...");
	delay(1000);
	debugln("Calibrating...");
	calib_offset_x = getOffset(mag);
	debug("offset x ");
	debugln(calib_offset_x);
	debugln();

	debugln("Calibrating magnetometer left bound. Place on left.");
	delay(250);
	debug("Calibrating in 3... ");
	delay(1000);
	debug("2... ");
	delay(1000);
	debugln("1...");
	delay(1000);
	debugln("Calibrating...");
	calib_x_min = getOffset(mag);
	debug("x left ");
	debugln(calib_x_min);
	debugln();

	debugln("Calibrating magnetometer right bound. Place on right.");
	delay(250);
	debug("Calibrating in 3... ");
	delay(1000);
	debug("2... ");
	delay(1000);
	debugln("1...");
	delay(1000);
	debugln("Calibrating...");
	calib_x_max = getOffset(mag);
	debug("x right ");
	debugln(calib_x_max);
	debugln("... Done calibrating.");
}


void loop()
{
	sensors_event_t event; 
	mag.getEvent(&event);
	auto turn = event.magnetic.x;
	uint16_t photo = photoVal();
	mapAndSendData(photo, turn);
}
