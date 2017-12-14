////////////////////////
//INCLUDES
////////////////////////
#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>
#include <math.h>
#include <SPI.h>
#include "RF24.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
////////////////////////



////////////////////////
//TYPE / COMPILER DEFS
////////////////////////
#ifdef DEBUG
#define debug(t) Serial.print(t)
#define debugln(t) Serial.println(t)
#else
#define debug(t) 
#define debugln(t) 
#endif
////////////////////////

// Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


////////////////////////
//GLOBALS
////////////////////////
uint8_t RadioId[] = {0xc8, 0x1a, 0x23, 0xd1, 0xbe}; //chosen at random to be different from other radios

const float ANGLE_MIN = -1.05;
const float ANGLE_MAX =  1.05;

uint16_t DIST_MIN = 99999;
uint16_t DIST_MAX = 0;

const int8_t TRN_MIN = -127;
const int8_t TRN_MAX = 127;

const uint8_t SPD_MIN = 0;
const uint8_t SPD_MAX = 255;

float calib_offset;

int16_t calib_offset_optimized_x, calib_offset_optimized_z;

float calib_offset_x;
float calib_x_min;
float calib_x_max;

RF24 radio(9,10);

MPU6050 mpu;

const int MPU_addr=0x68;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
////////////////////////

void dmpDataReady();
void initialize();
template<class F, class T, class I = T> T map_t(const F&, const F&, const F&, const T&, const T&);
template<class F, class T, class I = T> T map_c_t(F, const F&, const F&, const T&, const T&);
uint16_t photoVal();
float getIMUX();
void calibrateIMU();
float imuVal();
void mapAndSendData();
void init_radio_controller();
void init_radio_car();
int16_t arctan(int16_t, int16_t);
float getOffset(const Adafruit_HMC5883_Unified&);


void dmpDataReady() { mpuInterrupt = true; }

template<class F, class T, class I = T> T map_t(const F& value, const F& fromLow, const F& fromHigh, const T& toLow, const T& toHigh)
{

	I a = value - fromLow;
	I b = toHigh - toLow;
	I c = fromHigh - fromLow;
	// debug("MAPPING "); debug(value); debug(" from ["); debug(fromLow); debug(", "); debug(fromHigh); debug("] to ["); debug(toLow); debug(", "); debug(toHigh); debugln("]");
	// debug("("); debug(a); debug(" * "); debug(b); debug(") / "); debug(c); debug(" + "); debug(toLow);
	// debug(" = "); debug(a * b); debug(" / "); debug(c); debug(" + "); debugln(toLow);
	return T((a * b) / c + toLow);
}
template<class F, class T, class I = T> T map_c_t(F value, const F& fromLow, const F& fromHigh, const T& toLow, const T& toHigh)
{
	return map_t<F, T, I>((value < fromLow) ? fromLow : ((value > fromHigh) ? fromHigh : value), fromLow, fromHigh, toLow, toHigh);
}



void initializeSerial()
{
#ifdef DEBUG
	Serial.begin(115200);
#endif
}

void initializePhoto()
{
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
	//	2-0: Prescale Input clock by factor of 4 
	ADCSRA = 0b11100010;

	//ADCSRB
	//	  7: Unused
	//	  6: ADC will never be switched off, so this is N/A
	//	5-3: Unused
	//	2-0: Free Running Mode
	ADCSRB = 0b00000000;
}

void initializeIMU()
{
	Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

	// initialize device
	debugln("Initializing I2C devices...");
	mpu.initialize();

	// verify connection
	debugln("Testing device connections...");
	debugln(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// load and configure the DMP
	debugln("Initializing DMP...");
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	// make sure it worked (returns 0 if so)
	if (devStatus == 0)
	{
		// turn on the DMP, now that it's ready
		debugln("Enabling DMP...");
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		debugln("Enabling interrupt detection (Arduino external interrupt 0)...");
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		debugln("DMP ready! Waiting for first interrupt...");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		debug("DMP Initialization failed (code ");
		debug(devStatus);
		debugln(")");
	}
}

void initializeOptimizedIMU()
{
	Wire.begin();
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B);  // PWR_MGMT_1 register
	Wire.write(0);     // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
}

void calibrateOptimizedIMU()
{
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr,6,true);

	calib_offset_optimized_x=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  	Wire.read();Wire.read();
 	calib_offset_optimized_z=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

uint16_t photoVal()
{
	return *((uint16_t*)0x78) & 0x3ff;
}

float getIMUX()
{
	static uint8_t fifoBuffer[64]; // FIFO storage buffer
	static Quaternion q;           // [w, x, y, z]         quaternion container
	static VectorFloat gravity;    // [x, y, z]            gravity vector

	static float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		debugln("FIFO overflow!");
		return NAN;
	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02)
	{
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);
		
		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;
//		mpu.dmpGetQuaternion(&q, fifoBuffer);
//		mpu.dmpGetGravity(&gravity, &q);
//		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//		return ypr[0];
	}
	
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	return ypr[0];
}

void calibratePhoto()
{
	static const long CALIB_TIME = 2500;
	long init_time = millis();
	debugln("Calibrating Photoresistor...");
	debugln("Leave photoresistor unobscured.");
	while(millis() - init_time < CALIB_TIME)
	{
		DIST_MAX = max(DIST_MAX, photoVal());
	}
	debugln("Cover photoresistor.");
	while(millis() - init_time < CALIB_TIME * 2)
	{
		DIST_MIN = min(DIST_MIN, photoVal());
	}
	debugln("... Done.");
}

void calibrateIMU()
{
	static const uint16_t CALIB_TIME = 5000;
	long init_time = millis();
	float x_prev = 0;
	float x_curr;
 	bool calibrated = false;
	do
	{
		// if programming failed, don't try to do anything
		if (!dmpReady) return;

		// wait for MPU interrupt or extra packet(s) available
		while (!mpuInterrupt && fifoCount < packetSize);
		
		if(millis() - init_time >= CALIB_TIME)
		{
			x_prev = x_curr;
			init_time = millis();
		}
		float x_tmp = getIMUX();
		if(x_tmp == NAN) continue;
		x_curr = x_tmp;
		debug("curr\t");
		debugln(x_curr);
		debug("prev\t");
		debugln(x_prev);

	}
	while(x_curr != x_prev);
	
	debug("Calibrated!");
	calib_offset = x_curr;
	debugln(x_curr);
}

float imuVal()
{
    // if programming failed, don't try to do anything
  if (!dmpReady) return 0;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize);
	float x = getIMUX();
	if(x == NAN) return NAN;
	float output = ((x-calib_offset < 0) ? max(ANGLE_MIN, x - calib_offset) : min(ANGLE_MAX, x - calib_offset));


 if (calib_offset < 0) {
    output *= -1;  
  } 

	return output;
}

uint16_t imuValOptimized()
{
	int16_t x, z;

	Wire.beginTransmission(MPU_addr);
	Wire.write(0x43);  // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr,6,true);

	x=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  	Wire.read();Wire.read();
 	z=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

 	debug("x: ");
 	debug(x);
 	debug("\tz: ");
 	debugln(z);
 	return arctan(x, z);
}

void mapAndSendData(const uint16_t& photo_value, const float& imu_value)
{
	int8_t angle = map_c_t<float, int8_t, float>(imu_value, calib_x_min, calib_x_max, TRN_MIN, TRN_MAX);
	uint8_t dist = map_c_t<uint16_t, uint8_t, float>(photo_value, DIST_MIN, DIST_MAX, SPD_MIN, SPD_MAX);
	uint16_t squished = angle << 8 | dist;
	debug("Sending data: ");
	debug(int(angle));
	debug(" ");
	debugln((unsigned int)(dist));
	radio.write(&squished, sizeof(uint16_t));
	debug("Sent: ");
	debugln(squished);
}

void init_radio_controller()
{
	radio.begin();
	radio.setPALevel(RF24_PA_LOW);
	radio.openWritingPipe(RadioId);
	radio.stopListening();
}

void init_radio_car()
{
	radio.begin();
	radio.setPALevel(RF24_PA_LOW);
	radio.openReadingPipe(1, RadioId);
	radio.startListening();
}

int16_t arctan(int16_t opp, int16_t adj)
{
	static int16_t buckets [] = 
	{
		0,
		11,
		21,
		30,
		38,
		45,
		50,
		54,
		57,
		60
	};
	int16_t result = adj ? constrain((opp*5)/adj, 0, 9) : 0;
	int8_t sign = (result < 0) ? -1 : 1;
	result *= sign;
	return buckets[result] * sign;
}

float getOffset(const Adafruit_HMC5883_Unified& mag){
	long init_time = millis();
	short calib_time = 3000;
	float sum=0;
	short numvalues=0;


	while (millis()-init_time<=calib_time){
		sensors_event_t event; 
		mag.getEvent(&event);
		auto x = event.magnetic.x;
		sum+= x;
		numvalues+=1;

	}
	return sum/numvalues;
}