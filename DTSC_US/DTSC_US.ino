/*
 * Author: Joey Broussard
 * PNI, 20200812
 * 
 * Control of the stepper motor performed on dedicated arduino
 * for the Delayed Tactile Startle Conditioning paradigm. Designed for 
 * master Arduino to activate interrupt input on the Arduino running
 * this sketch to generate the US. No specialized libraries are 
 * required as the motion of the US is controlled by an EasyDriver.
 * 
 * todo: Make it possible to change "target" based on if mouse moves 
 * back during CS presentation, can use additional interrupt pin (not 
 * ideal) or maybe some I2C comm?
 */

#include "Arduino.h"

//Pins
const int intPin = 2;//interrupt pin
const int M1pin = 4;//micro stepper pin 1
const int M2pin = 5;//micro stepper pin 2
const int slpPin = 9;//sleep pin, LOW = motor free
const int stepPin = 11;//step pin
const int dirPin = 12;//direction pin

//Cues for movement/direction
const long target = 10;
volatile long steps = 0;
volatile byte dirState = HIGH;

void setup()
{ 
  //Setup interrupt
  pinMode(intPin,INPUT);
  attachInterrupt(digitalPinToInterrupt(intPin),US,RISING);
  
  //Setup microstepping
  pinMode(M1pin,OUTPUT);
  digitalWrite(M1pin,LOW);
  pinMode(M2pin,OUTPUT);
  digitalWrite(M2pin,HIGH);
  
  //Setup pin for deactivating the motor
  pinMode(slpPin,OUTPUT);
  digitalWrite(slpPin,LOW);//LOW = Motor in free run until receives commands

  //Direction pin changes direction of running
  pinMode(dirPin,OUTPUT);
  digitalWrite(dirPin,dirState);

  //Step pin
  pinMode(stepPin,OUTPUT);
  digitalWrite(stepPin,LOW);

  //Report initial state
  Serial.begin(9600);
  Serial.println("Started, motor free");
}

void SerialIn(String str){
  if (str.length()==0){
    return;
  }
  if (str == "free"){
    digitalWrite(slpPin,LOW);
    Serial.println("Motor free");
  }else if (str == "active"){
    digitalWrite(slpPin,HIGH);
    Serial.println("Motor active");
  }
}
/*US occurs on rising interrupt pin*/
void US()
{ 

  while (steps<target){
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(1000);
    steps++;
  };
  steps = 0;
  digitalWrite(dirPin,!digitalRead(dirPin));
  digitalWrite(M1pin,HIGH);
  digitalWrite(M2pin,HIGH);
  while (steps<target*5.2){
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(1000);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(1000);
    steps++;
  };
  steps = 0;
  digitalWrite(dirPin,!digitalRead(dirPin));
  digitalWrite(M1pin,LOW);
  digitalWrite(M2pin,HIGH);
  Serial.println("US done" + String(dirState));
  
}

void loop(){
  //Wait for serial commands to dictate motor state
  if (Serial.available() > 0) {
    String inString = Serial.readStringUntil('\n');
    inString.replace("\n","");
    inString.replace("\r","");
    SerialIn(inString);
  }
}
