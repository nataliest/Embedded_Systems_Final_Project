
#include <SPI.h>
//#include "nRF24L01.h"
#include "RF24.h"
//#include "printf.h"
#include <stdint.h>

RF24 radio(9,10);

uint8_t RadioId[] = {0xc8, 0x1a, 0x23, 0xd1, 0xbe}; //chosen at random to be different from other radios


void setup() {
  
  // put your setup code here, to run once:
    
    radio.begin();
    radio.setPALevel(RF24_PA_LOW);
    radio.openReadingPipe(1, RadioId);
    radio.startListening();
    Serial.begin (57600);
    
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t data;
  if(radio.available())
  {
    while (radio.available()) {
      radio.read(&data, sizeof(uint16_t));
      Serial.println(data);
    }

  }
}
