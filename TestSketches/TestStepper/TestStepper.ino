// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $
#include <AccelStepper.h>

//Pins
const int intPin = 2;
const int M1pin = 4;
const int M2pin = 5;
const int slpPin = 9;
const int stepPin = 11;
const int dirPin = 12;

//For cue to change direction
long steps = 0;

//Cues for movement
long target = 20;
long velocity = 400;
long stepWait = 1e6*1/velocity;
boolean moving = false;



void setup()
{ 
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
  digitalWrite(dirPin,LOW);

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

void loop()
{ 

  //Wait for serial commands to dictate motor state
  if (Serial.available() > 0) {
    String inString = Serial.readStringUntil('\n');
    inString.replace("\n","");
    inString.replace("\r","");
    SerialIn(inString);
  }

  digitalWrite(stepPin,HIGH);
  delayMicroseconds(1000);
  digitalWrite(stepPin,LOW);
  delayMicroseconds(1000);
  
  
}
