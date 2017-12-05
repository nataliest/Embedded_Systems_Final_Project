////////////////////////
//INCLUDES
////////////////////////
#ifdef DEBUG
#include <Arduino.h>
#endif

#ifdef OPTIMIZED
#include <cstdint>
#else
#include <Arduino.h>
#endif

#include <Wire.h>
////////////////////////



////////////////////////
//TYPE / COMPILER DEFS
////////////////////////
#ifdef DEBUG
#define debug(t) Serial.print(t)
#else
#define debug(t) 
#endif

#ifdef OPTIMIZED
typedef uint16_t Angle
#else
typedef float Angle
#endif

#define MAGNET_ADDR 0x1E
////////////////////////



////////////////////////
//GLOBALS
////////////////////////
#ifdef OPTIMIZED
const Angle ANGLE_MIN = 0;
const Angle ANGLE_MAX = 0;
#else
const Angle ANGLE_MIN = -1.047f;
const Angle ANGLE_MAX =  1.047f;
#endif

uint16_t DIST_MIN = 0;
uint16_t DIST_MAX = 0;

const int8_t FMT_MIN = -127;
const int8_t FMT_MAX = 127;
////////////////////////



template<class F, class T = F> map(const F& value, const F& fromLow, const F& fromHigh, const T& toLow, const T& toHigh)
{
	return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + fromLow;
}



void initialize()
{
#ifdef DEBUG
	Serial.begin(9600);
#endif

#ifdef OPTIMIZED
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
#else
	Wire.begin();
	Wire.beginTransmission(MAGNET_ADDR); //open communication with HMC5883
	Wire.write(0x2); //select mode register
	Wire.write(0x0); //continuous measurement mode
	Wire.endTransmission();
#endif
}

uint16_t photoVal()
{
#ifdef OPTIMIZED
	return *((uint16_t*)0x78) & 0x3ff;
#else
	return analogRead(A0);
#endif
}

short magnetVal()
{
	static short lastValX = 0;
	static short lastValY = 0;
	static short lastValZ = 0;
#ifdef OPTIMIZED
#else
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
#endif
}

Angle imuVal()
{
#ifdef OPTIMIZED
#else
	static uint8_t fifoBuffer[64]; // FIFO storage buffer
	static Quaternion q;           // [w, x, y, z]         quaternion container
	static VectorFloat gravity;    // [x, y, z]            gravity vector

	static Angle ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
	static Angle curr_x;
	static Angle prev_x;
	Angle calib_offset;

	static long init_time;
	short CALIB_TIME = 20000;

	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	curr_x = ypr[0];
	
	if ((millis() - init_time) > CALIB_TIME)
	{
	 	if (prev_x < 0)
	 	{
			calib_offset = prev_x * -1;
		}
		else
		{
			calib_offset = prev_x;
		}
		debug("Calibrated value\t");
	  
	}
	else
	{
		debug("x axis\t");
		prev_x = curr_x;
	} 
	if ((ypr[0] - calib_offset) < 0)
	{
		Angle output = max(ANGLE_MIN, (ypr[0] - calib_offset));
		debug(output);
		return output;
	}
	else
	{
		Angle output = min(ANGLE_MAX, (ypr[0] - calib_offset));
		debug(output);
		return output;
	}
#endif
}

void mapAndSendData(const short& photo_value, const Angle& imu_value)
{
	int8_t angle = map(imu_value, ANGLE_MIN, ANGLE_MAX, FMT_MIN, FMT_MAX);
	uint8_t photo_value = map(photo_value, DIST_MIN, DIST_MAX, uint8_t(0), uint8_t(255));
	
}