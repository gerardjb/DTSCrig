//Making sure new rotary encoder-type works as expected
#include "Arduino.h"
#include <Wire.h>
#include <Encoder.h> // http://www.pjrc.com/teensy/td_libs_Encoder.html from PaulStoffregen

//Initializing the roatary encoder structure
struct rotaryencoder
{
  int pinA = 2; // use pin 2
  int pinB = 3; // use pin 3
  float pos = 0; //setting initial position of encoder
  float count = 0; //setting up to query the position on specified interval
};
typedef struct rotaryencoder RotaryEncoder;
RotaryEncoder rotaryencoder;
boolean justOut = true;

//Stepper and Encoder objects defined per relevant libraries
Encoder myEncoder(rotaryencoder.pinA, rotaryencoder.pinB);

void setup() {
  //rotary encoder pin definitions
  rotaryencoder.pinA = 3;
  rotaryencoder.pinB = 2;
  //

  //Initialize serial
  Serial.begin(115200);//high baud rate for rapid writing out
  Serial.println("Starting");
}

//Outputting info over the serial port
void serialOut(unsigned long now, signed long dist) {
  Serial.println(String(now) + ",rot," + String(dist));
  //Serial.println(dist);
};

//Updating the position read off of the rotary encoder, dumping
//difference to file if during a trial and changed after >100msec
void updateEncoder(unsigned long now) {
  float posNow = myEncoder.read();

  if (justOut){
    rotaryencoder.count = now;
    rotaryencoder.pos = posNow;
    justOut = false;
  };
    
    float diff = now - rotaryencoder.count;
    
    if (diff>=10000){
      float dist =  rotaryencoder.pos - posNow;
      serialOut(now, dist);
      rotaryencoder.count = now;
      rotaryencoder.pos = posNow;
      justOut = true;
    };
  
}

void loop() {
  //Counting millis
  unsigned long now = millis();

  //Function to how much the encoder has moved
  updateEncoder(now);

}
