// #define OPTIMIZED
#define DEBUG
#include "Internals.h"

void setup()
{
	initialize();
}

void loop()
{
	debug(magnetVal());
	debug("\n");
}	