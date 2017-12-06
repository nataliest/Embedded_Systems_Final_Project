#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>

//Globals as set up in the "gettingstarted" radio program
bool radioNumber = 0; //random number tbd actual assuming even number is reciever
RF24 radio(11,12); //SPI pin bus 11,12 (MOSI/MISO) may need to be changed
byte addresses[][6] = {"1Node","2Node"}; //this is left as is from code may need change



void setup() {
  // put your setup code here, to run once:
  //TCCR1A = 0b11110011;  //maybe 11110011
  //TCCR1B = 0b00001100;
  TCCR0A=0b10100011;
  TCCR0B=0b00000101;
  /*setting timer as 10bit with 256prescaler this will give us
   * 61Hz and will allow for 20+ speed options
   */
  DDRD = 0b01100000; //pwm will run off D pins (timer should be connected to D and B ports)
  PORTD = 0b01100000;
  DDRB=0b00111111;
  PORTB=0b00101110; //SCK, MOSI, CSN, CE, and IRQ are set to output here
  
  radio.begin(); //turns radio on

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
 /*
  * I am leaving this set to low for the inital test, we may need to chane go high after testing
  */
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(!radioNumber){ //this should always be true, just modified from origional, below is the "else" portion from that code
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  
  // Start the radio listening for data
  radio.startListening();


  
}

void loop() {
  // put your main code here, to run repeatedly
  int total=255;
  int totalforward=255; //from accel
  int totalback = 255;
  int turn=0;  //from steering mechanism
  int left;  //holds value to reduce left motor
  int right; //hold value to reduce right motor
  /* 
   *  OCR0A and OCR0B will be a value between a max value of 125
   *  and a min value of 97, this will be sent via hex (we might have to make this int
   *  I assume that
   *  Pin 5 will be left (OCRB), and Pin 6 will be right (OCRA) motors
  */

  /*
   * if (radio.available()){
   *  while (radio.available()){
   *    radio.read(&total, sizeof(int));
   *    radio.read(&turn, sizeof(int));
   *    
   *  }
   * 
   */


  /* while (!radio.avialable()){}
   *  
   * if(radio.available()){
   *  radio.read(&total, sizeof(int));
   * }
   *    
   * while (!radio.avialable()){}
   * 
   * if (radio.available()){
   *  radio.read(&turn, sizeof(int))
   *  }
   *  
   *  radio.stopListening();
   */

  totalforward = map(total, 0 , 255, 24, 31); //sets the overall motorspeed to a value between 97-125 which are min and max
  totalback = map(total, 0, 255, 15, 23);

  if ( turn == 0){ //assuming that we are going straight
    left = 0;
    right = 0;
  }

  else if ( turn < 0){  //we want to turn left
    left = map (turn, 0, -127, 0 , 7); //sets left as a value
    right = 0;
  }

  else if (turn > 0){  //we want ot turn right
    right = map (turn, 0, 127, 0, 7);
    left = 0;
  }
  
  OCR0A = (totalforward - right); //if right is <0 this will reduce the right wheel speed
  OCR0B = (totalback + left); //if left is <0 this will reduce left wheel speed

}
