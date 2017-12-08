#define DEBUG
#include "Internals.h"

void setup()
{
	Serial.begin(115200);
	// initialize();
	// calibrateIMU();
	init_radio_controller();
}

void printBinary(uint16_t data)
{
	for(int i = 15; i >= 0; --i)
	{
		Serial.print(((data >> i) << 15) >> 15);
	}
	Serial.println();
}

long data[2];
const char* prompt[] = {"TRN:", "SPD:"};
bool prompted = false;
bool i;
uint16_t squished;

void loop()
{
	if ( Serial.available() )
	{
		data[i] = Serial.parseInt();
		i = !i;
		prompted = false;
		if(i == 0)
		{
			int8_t t = data[0];
			uint8_t p = data[1];
			squished = t << 8 | p;
			debug(squished);
			debug("\t");
			printBinary(squished);
			radio.write(&squished, sizeof(uint16_t));
		}
	}
	else if(!prompted)
	{
		debugln();
		debugln();
		debugln();
		debugln(prompt[i]);
		prompted = true;
	}
	// // if programming failed, don't try to do anything
	// if (!dmpReady) return;

	// // wait for MPU interrupt or extra packet(s) available
	// while (!mpuInterrupt && fifoCount < packetSize);
	
	// uint16_t photo = photoVal();
	// float imu = imuVal();

	// debug(photo);
	// debug("\t");
	// debug(imu);
	// debug("\t");
	// mapAndSendData(photo, imu);
	// debugln();
}
