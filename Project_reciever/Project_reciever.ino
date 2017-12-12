#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <stdint.h>
//#define DEBUG
//#include "Internals.h"

//Globals as set up in the "gettingstarted" radio program
RF24 radio (9,10); //SPI pin bus 11,12 (MOSI/MISO) may need to be changed
uint8_t RadioId[] = {0xc8, 0x1a, 0x23, 0xd1, 0xbe}; //chosen at random to be different from other radios


void setup() {
  // put your setup code here, to run once:
  TCCR0A = 0b10100011;
  TCCR0B = 0b00000101;
  DDRD = 0b01100000; //pwm will run off D pins (timer should be connected to D and B ports)
  PORTD = 0b01100000;
  //DDRB = 0b00111111;
  //PORTB = 0b00101110; //SCK, MOSI, CSN, CE, and IRQ are set to output here
  
  radio.begin(); //turns radio on

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
 /*
  * I am leaving this set to low for the inital test, we may need to chane go high after testing
  */
  radio.setPALevel(RF24_PA_LOW);
  
  radio.openReadingPipe(1, RadioId);
  
  // Start the radio listening for data
  radio.startListening();
  Serial.begin (115200);


  
}
  uint16_t data=255;
  uint8_t* total = (uint8_t*)&data; //from accel
  uint8_t spd;
  static int8_t* turn = (((int8_t*)&data) + 1);  //from steering mechanism
  uint8_t left;  //holds value to reduce left motor
  uint8_t right; //hold value to reduce right motor
  uint8_t totalright;  //hold total motor speed value
  uint8_t totalleft;
void loop() {
  // put your main code here, to run repeatedly

  /* 
   *  OCR0A and OCR0B will be a value between a max value of 31
   *  and a min value of 24, this will be sent via hex (we might have to make this int
   *  I assume that
   *  Pin 5 will be left (OCRB), and Pin 6 will be right (OCRA) motors
  */
   
  //static uint16_t data;
  if(radio.available())
  {
    while (radio.available()) {
      radio.read(&data, sizeof(uint16_t));
      //Serial.println(data);
    }

}
 //Serial.println(*total); 
spd = map(*total, 0 , 255, 31,23 );
  totalright = spd;
  totalleft = spd;

  if ( *turn == 0){ //assuming that we are going straight
    left = 0;
    right = 0;
  }

  else if ( *turn < 0){  //we want to turn right
    left = map (*turn, 0, 127, 0 , 7); //sets left as a value
    right = 0;
  }

  else if (*turn > 0){  //we want ot turn left
    right = map (*turn, 0, -127, 0, 7);
    left = 0;
  }
  
  OCR0A = max (23,(totalright - right)); //if right is <0 this will reduce the right wheel speed
  OCR0B = max (23, (totalleft - left)); //if left is <0 this will reduce left wheel speed
  
  //Serial.println(OCR0A);
  //Serial.println();
  //Serial.println(OCR0B);
 // Serial.println();
  //Serial.println(*total);
  //Serial.println(spd);
  //Serial.println();
  }

