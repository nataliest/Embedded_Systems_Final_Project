#include <Arduino.h>

#ifdef EASYMODE
#include <Wire.h>
#else
#endif

#define MAGNET_ADDR 0x1E

void initialize()
{
	Serial.begin(9600);

#ifdef EASYMODE
	Wire.begin();
	Wire.beginTransmission(MAGNET_ADDR); //open communication with HMC5883
	Wire.write(0x2); //select mode register
	Wire.write(0x0); //continuous measurement mode
	Wire.endTransmission();
#else
	//ADMUX
	//	7-6: Reference Voltage = Vcc
	//	  5: Analog Input Data will be right-aligned
	//	  4: Unused
	//	3-0: Analog Input 0 is used
	ADMUX = 0b01000000;

	//ADCSRA
	//	  7: Enable Analog-DC Conversion
	//	  6: Start Analog-DC Conversion
	//	  5: Automatically Start Conversion on postive edge of trigger signal
	//	  4: Disable Interrupt Flag
	//	  3: Disable Interrupts
	//	2-0: Prescale Input clock by factor of 2 (minimum)
	ADCSRA = 0b11100000;

	//ADCSRB
	//	  7: Unused
	//	  6: ADC will never be switched off, so this is N/A
	//	5-3: Unused
	//	2-0: Free Running Mode
	ADCSRB = 0b00000000;
#endif
}

unsigned short photoVal()
{
#ifdef EASYMODE
	return analogRead(A0);
#else
	return *((unsigned short *)0x78) & 0x3ff;
#endif
}

short magnetVal()
{
	static short lastValX = 0;
	static short lastValY = 0;
	static short lastValZ = 0;
#ifdef EASYMODE
	Wire.beginTransmission(MAGNET_ADDR); //open communication with HMC5883
	Wire.write(0x3); //select register 3, X MSB register
	Wire.endTransmission();
	Wire.requestFrom(MAGNET_ADDR, 6); //Request 2 bytes at register 3
	if(Wire.available() >= 6)
	{
		lastValX = Wire.read() << 8;
		lastValX |= Wire.read();
		lastValY = Wire.read() << 8;
		lastValY |= Wire.read();
		lastValZ = Wire.read() << 8;
		lastValZ |= Wire.read();
	}
	return lastValX;
#else
#endif
}