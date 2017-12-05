#define EASYMODE
#include "Internals.h"

void setup()
{
	initialize();
}

void loop()
{
	Serial.println(magnetVal());
}	