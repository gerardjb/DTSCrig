/*
 * Author: Joey Broussard
 * PNI, 20200820
 * DTSC rig master arduino controller sketch
 * 
 * V1.0 - rewritten from eyeblink3_4 to interface with DTSC_US and
 *  eyeblink_app.py
 * V1.1 - added pre-CS backing catch, inter-trial waits on motion
 * todo: -
 */
 
#include "Arduino.h"
#include <Wire.h>
#include <Encoder.h> // http://www.pjrc.com/teensy/td_libs_Encoder.html

/////////////////////////////////////////////////////////////
//////Making sure puffPin is set LOW as it's not used here
int puffPin = 5;
/*Structure and state definitions*/
//Defining trial structure with associated attributes
struct trial
{
  //Session timing and numbering
  boolean sessionIsRunning;//flag for starting and stopping session
  int sessionNumber;//correspnds to session number
  unsigned long sessionStartMillis;//ms time at which session starts
  unsigned long sessionDur; //ms, numEpoch*epochDur
  //Trial timing and numbering
  boolean trialIsRunning;
  int currentTrial;
  unsigned long trialDur;//ms trial duration
  unsigned long numTrial;//number of trials we want
  unsigned long trialStartMillis; //ms time trial starts
  unsigned long interTrialIntervalLow; //ms lowest inter-trial interval
  unsigned long interTrialIntervalHigh; //ms highest inter-trial interval  
  unsigned long ITIstartMillis;//ms time at which interTrialInterval starts
  //Trial pin
  boolean pinOnOff;//controls transitioning pin state
  int trialPin;//pin for projecting current trial state
  //CS and US
  unsigned long CSstartMillis; //millis at start of currentPulse
  unsigned long  preCSdur; //ms time in trial before CS
  int CSdur; //ms CS duration
  unsigned long USdur;//ms
  unsigned long CS_USinterval;//ms
  unsigned int percentUS;//percent trials user wants to be US only trials
  unsigned int percentCS;//percent trials user wants to be CS only
  //motor
  int useMotor;//{motorOn,motorLocked,motorFree}
  unsigned long motorSpeed; //rev/sec

  
};

//Handling in-trial and inter-trial event timing
unsigned long msIntoSession;
unsigned long msIntoTrial;
unsigned long interTrialInterval;//ms
unsigned long sumITI;//ms holds sum of all ITIs to calculate when to end session
unsigned long motionCatchDur = 100;//millis to track motion prior to CS
boolean inMotionCatch;//are we in time to check animal for backing up pre-CS
boolean inCS_USint;//let us know if we should sum rotary to decide attack size
boolean transmitMotion = false;//let us know if animal moved in specified way during motion catch
volatile boolean transmitAttack = false;//wire out boolean for big or small attack
int tmpTrial;
String stimPairType;// rng used to determine CS_US, CS, or US trial type


struct rotaryencoder
{
  int pinA = 2; // use pin 2
  int pinB = 3; // use pin 3
  float pos = 0; //setting initial position of encoder
  float timer = 0; //setting up to query the position only once every specified ms
	float motionStart = 0; //position at start of motion catch
	float sumMotion = 0;// sum of rotary encoder during motion catch
  float CS_USposStart = 0;//position at start of CS
  float sumCS_US = 0;//sum of roary encoder during CS-US interval
};

struct ledCS
{
  boolean isOnLED; // use to toggle on and off
  int ledPin; // pin for the CS
};

struct DTSC_US
{
  boolean isOnDTSC;// use to toggel on and off
  int DTSCPin;// pin for the US
};


//Version, defining structures and aliases
String versionStr = "DTSC1_0.cpp";
typedef struct trial Trial;
typedef struct rotaryencoder RotaryEncoder;
typedef struct ledCS LedCS;
typedef struct DTSC_US dtsc_US;

//Instances of all hardware-associated structures
Trial trial;
RotaryEncoder rotaryencoder;
LedCS ledCS;
dtsc_US DTSC_US;

//Stepper and Encoder objects defined per relevant libraries
Encoder myEncoder(rotaryencoder.pinA, rotaryencoder.pinB);

/////////////////////////////////////////////////////////////
/*Setup, mostly declaring default structure values*/
void setup()
{
  //trial
  trial.sessionIsRunning = false;
  trial.sessionNumber = 0;
  trial.sessionStartMillis = 0;

  trial.trialIsRunning = false;
  trial.trialDur = 1000; // epoch has to be >= (preDur + xxx + postDur)
  trial.numTrial = 1;
  
  trial.sessionDur = (trial.numTrial*trial.trialDur); //

  trial.useMotor = 0; //0 = motorOn, 1 = motorLocked, 2 = motorFree
  trial.motorSpeed = 500; //step/sec

  trial.preCSdur = 3000;
  trial.CSdur = 350;
  trial.USdur = 50;
  trial.CS_USinterval = trial.CSdur - trial.USdur;
  trial.interTrialIntervalLow = 5000;//ms
  trial.interTrialIntervalHigh = 20000;//ms
  trial.ITIstartMillis = 0;//ms
  trial.percentUS = 0;//percent US only trials
  trial.percentCS = 10;//percent CS only trials

  trial.trialPin = 7;//pin for conveying trial state
  trial.pinOnOff = false;//trial didn't just end
  pinMode(trial.trialPin, OUTPUT);
  digitalWrite(trial.trialPin, LOW);
  
  sumITI = 0;
  //motor.resetPin = xxx;
  //
  //rotary encoder
  rotaryencoder.pinA = 3;
  rotaryencoder.pinB = 2;
  //
  
  //Activate pin 13 for testing
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);

  //CS/US structure and pin settings, intially at Arduino grnd
  pinMode(puffPin,OUTPUT);
  digitalWrite(puffPin,LOW);
  ledCS.ledPin = 4;
  ledCS.isOnLED = false;
  pinMode(ledCS.ledPin,OUTPUT);
  digitalWrite(ledCS.ledPin,LOW);
  DTSC_US.DTSCPin = 12;
  DTSC_US.isOnDTSC = false;
  pinMode(DTSC_US.DTSCPin,OUTPUT);
  digitalWrite(DTSC_US.DTSCPin,LOW);
  
  //Initialize serial
  Serial.begin(115200);

  //Initialize as I2C master
  Wire.begin();//join I2C bus with no given address you master, you
 
}

/////////////////////////////////////////////////////////////
/*Starting and ending Trials (sessions)*/
//Start session
void startSession(unsigned long now) {
  if (trial.trialIsRunning==false) {
    //I2C to inactivate slp pin DTSC wheel
    wireOut(0,0);
    trial.sessionNumber += 1;
    
    trial.sessionStartMillis = now;
    trial.trialStartMillis = now;

    trial.sessionDur = trial.trialDur * trial.numTrial;
    serialOut(now, "sessionDur",trial.sessionDur);
    serialOut(now, "numTrial", trial.numTrial);
    serialOut(now, "trialDur", trial.trialDur);
    trial.currentTrial = 0;
    
    serialOut(now, "startSession", trial.sessionNumber);
    digitalWrite(trial.trialPin,HIGH);
    serialOut(now, "startTrial", trial.currentTrial);

    trial.sessionIsRunning = true;
    trial.trialIsRunning = true;

    //Calculate trial type
    unsigned int RNG = random(1,101);
    if (RNG <= trial.percentUS){
       stimPairType = "US";
    } else if (RNG > trial.percentUS && RNG <= (trial.percentUS + trial.percentCS)){
      stimPairType = "CS";
    } else if (RNG > (trial.percentUS + trial.percentCS)){
      stimPairType = "CS_US";
    }
    serialOut(now,stimPairType,trial.currentTrial);
    //scanImageStart_(now);
    
  }
}

//Start trial
void startTrial(unsigned long now){
  if (trial.trialIsRunning==false){
    trial.currentTrial += 1;

    trial.trialStartMillis = now;
    digitalWrite(trial.trialPin,HIGH);
    serialOut(now,"startTrial",trial.currentTrial);

    trial.trialIsRunning = true;

    //Calculate trial type
    unsigned int RNG = random(1,101);
    if (RNG <= trial.percentUS){
       stimPairType = "US";
    } else if (RNG > trial.percentUS && RNG <= (trial.percentUS + trial.percentCS)){
      stimPairType = "CS";
    } else if (RNG > (trial.percentUS + trial.percentCS)){
      stimPairType = "CS_US";
    }
    serialOut(now,stimPairType,trial.currentTrial);
  }
}

//End trial
void stopTrial(unsigned long now) {
  //If this is the last trial, end session
  if (trial.currentTrial == trial.numTrial) {
  stopSession(now);
  }
  trial.trialIsRunning = false;
  digitalWrite(trial.trialPin,LOW);
  serialOut(now, "stopTrial", trial.currentTrial);
  //Set time to wait until next trial starts
  interTrialInterval = random(trial.interTrialIntervalLow,trial.interTrialIntervalHigh);
  trial.ITIstartMillis = now;
  //sum ITIs so they can be counted towards total session time
  sumITI = sumITI + interTrialInterval;
  
}

//End Session
void stopSession(unsigned long now) {
    if (trial.trialIsRunning){
      trial.trialIsRunning = false;
      serialOut(now,"stopTrial",trial.currentTrial);
    }
  digitalWrite(trial.trialPin,LOW);
  serialOut(now,"stopSession",trial.sessionNumber);
  //reset states
  trial.sessionIsRunning = false;
  trial.trialIsRunning = false;
  digitalWrite(trial.trialPin,LOW);
  trial.sessionNumber += 1;
	trial.currentTrial = 0;
  
  
  //I currently don't expect to have to change DTSC wheel, 
  //but check this


}
/////////////////////////////////////////////////////////////
/*Communication via serial port or I2C*/
//Sending info to slave Arduino over I2C
void wireOut(int state2change,int newStateVal){
  Wire.beginTransmission(8);
  Wire.write(state2change);//0 = active/free; 1 = bigAttack boolean
  Wire.write(newStateVal);//0 = free/bigAttack false; 1 = active/bigAttack true
  Wire.endTransmission();
}

//Outputting info over the serial port
void serialOut(unsigned long now, String str, signed long val) {
  Serial.println(String(now) + "," + str + "," + String(val));
}

//Respond to incoming commands over serial
void SerialIn(unsigned long now, String str) {
  String delimStr = ",";
    
  if (str.length()==0) {
    return;
  }
  if (str == "version") {
    Serial.println("version=" + versionStr);
  } else if (str == "startSession") {
    startSession(now);
  }
  else if (str == "stopSession") {
    stopSession(now);
  }
  else if (str.startsWith("getState")) {
    GetState();
  }
  else if (str.startsWith("settrial")) {
    //set is {set,name,value}
    int firstComma = str.indexOf(delimStr,0);
    int secondComma = str.indexOf(delimStr,firstComma+1);
    String nameStr = str.substring(firstComma+1,secondComma); //first is inclusive, second is exclusive
    String valueStr = str.substring(secondComma+1,str.length());
    SetTrial(nameStr, valueStr);
  }
  else {
    Serial.println("SerialIn() did not handle: '" + str + "'");
  }
  trial.CS_USinterval = trial.CSdur - trial.USdur;
    
}

//Get the current experiment parameters;
//This generates the headers for the output files
void GetState() {
  //trial
  Serial.println("sessionNumber=" + String(trial.sessionNumber));
  Serial.println("sessionDur=" + String(trial.sessionDur));

  Serial.println("numTrial=" + String(trial.numTrial));
  Serial.println("trialDur=" + String(trial.trialDur));
  Serial.println("interTrialInteval=" + String(trial.interTrialIntervalLow) + String(trial.interTrialIntervalHigh)); 

  Serial.println("preCSdur=" + String(trial.preCSdur));
  Serial.println("CSdur=" + String(trial.CSdur));
  Serial.println("USdur=" + String(trial.USdur));
  Serial.println("CS_USinterval=" + String(trial.CS_USinterval));
  Serial.println("percentUS=" + String(trial.percentUS));
  Serial.println("percentCS=" + String(trial.percentCS));

  Serial.println("useMotor=" + String(trial.useMotor));
  Serial.println("motorSpeed=" + String(trial.motorSpeed));
  
  Serial.println("versionStr=" + String(versionStr));

}

//Setting experiment parameters
void SetTrial(String name, String strValue) {
  float value = strValue.toFloat();

  //trial
  if (name == "numTrial") {
    trial.numTrial = value;
    Serial.println("trial.numTrial=" + String(trial.numTrial));
  } else if (name=="trialDur") {
    trial.trialDur = value;
    Serial.println("trial.trialDur=" + String(trial.trialDur));
    
  } else if (name=="interTrialIntervalHigh") {
    trial.interTrialIntervalHigh = value;
    Serial.println("trial.interTrialIntervalHigh=" + String(trial.interTrialIntervalHigh));
  } else if (name=="interTrialIntervalLow") {
    trial.interTrialIntervalLow = value;
    Serial.println("trial.interTrialIntervalLow=" + String(trial.interTrialIntervalLow));
    
  } else if (name=="preCSdur") {
    trial.preCSdur = value;
    Serial.println("trial.preCSdur=" + String(trial.preCSdur));
    
  } else if (name=="CSdur") {
    trial.CSdur = value;
    Serial.println("trial.CSdur=" + String(trial.CSdur));
  } else if (name=="USdur") {
    trial.USdur = value;
    Serial.println("trial.USdur=" + String(trial.USdur));
    
  } else if (name=="percentCS") {
    trial.percentCS = value;
    Serial.println("trial.percentCS=" + String(trial.percentCS));
  } else if (name=="percentUS") {
    trial.percentUS = value;
    Serial.println("trial.percentUS=" + String(trial.percentUS));
    
  } else if (name=="useMotor") {
    if (strValue=="motorOn") {//0 for forced run, 1 for locked, 2 for free run
      trial.useMotor = 0;
    } else if (strValue=="motorLocked"){
      trial.useMotor = 1;
    } else if (strValue=="motorFree"){
      trial.useMotor = 2;
    }
    Serial.println("trial.useMotor=" + String(strValue));
    /*I2C-directed*///0 for change motor sleep state
    wireOut(0,trial.useMotor);
  } else if (name=="motorSpeed") {
    trial.motorSpeed = value;
    Serial.println("trial.motorSpeed=" + String(trial.motorSpeed));
    /*I2C-directed*/
    wireOut(1,trial.motorSpeed);
  }else {
    Serial.println("SetValue() did not handle '" + name + "'");
  }
  
}
/////////////////////////////////////////////////////////////
/*Interacting with hardware components*/
//Rotary encoder
//Updating the position read off of the rotary encoder, dumping
//difference to file if during a trial and changed after >specidied msec
void updateEncoder(unsigned long now,bool inMotionCatch,bool inCS_USint) {
  float posNow = myEncoder.read();
  
  if (trial.trialIsRunning && (now-trial.trialStartMillis<3)){
    rotaryencoder.timer = now;
    rotaryencoder.pos = posNow;
  }
  
  if (trial.trialIsRunning){  
    float diff = now - rotaryencoder.timer;
    
		//update encoder output when specified difference reached
    if (diff>=20){
      float dist =  rotaryencoder.pos - posNow;
      serialOut(now, "rotary", dist);
      rotaryencoder.timer = now;
      rotaryencoder.pos = posNow;
    }
		
		//determine if animal moves back before CS start as pre-CS backing catch
		if (inMotionCatch && !transmitMotion){
			rotaryencoder.motionStart = posNow;
			transmitMotion = true;
		}else if(!inMotionCatch && transmitMotion){
			rotaryencoder.sumMotion = rotaryencoder.motionStart - posNow;
			transmitMotion = false;
		}
		
		//Sum movement during CS_US interval then send attack
		if (inCS_USint && !transmitAttack && (stimPairType=="US"||stimPairType=="CS_US")){
      rotaryencoder.CS_USposStart =  posNow;
      transmitAttack = true;
    }else if (!inCS_USint && transmitAttack && (stimPairType=="US"||stimPairType=="CS_US")){
      rotaryencoder.sumCS_US = rotaryencoder.CS_USposStart - posNow;
      if (rotaryencoder.sumCS_US>=-1 || rotaryencoder.sumMotion<-1){
        wireOut(1,1);//send big attack if animal didn't move back
				DTSC_US.isOnDTSC = true;
				serialOut(now,"bigUSon",trial.currentTrial);
				digitalWrite(DTSC_US.DTSCPin,HIGH);
				
      }else{
        wireOut(1,0);//send little attack if animal moved back
				DTSC_US.isOnDTSC = true;
				serialOut(now,"smallUSon",trial.currentTrial);
				digitalWrite(DTSC_US.DTSCPin,HIGH);
      }
      transmitAttack = false;
			rotaryencoder.sumCS_US = 0;
    }
  }
  
}

//Triggering the LED CS
void updateLED(unsigned long now){
  if (trial.trialIsRunning && (stimPairType=="CS"||stimPairType=="CS_US")){
    //Turning CS on and off while correct trial type running
    unsigned long ledStart = trial.trialStartMillis + trial.preCSdur;
    unsigned long ledStop = ledStart + trial.CSdur;
    if (!ledCS.isOnLED && now >= ledStart && now <= ledStop){
      ledCS.isOnLED = true;
      trial.CSstartMillis = now;
      serialOut(now,"ledCSon",trial.currentTrial);
      digitalWrite(ledCS.ledPin,HIGH);
    } else if(ledCS.isOnLED && now>ledStop){
      ledCS.isOnLED = false;
      serialOut(now,"ledCSoff",trial.currentTrial);
      digitalWrite(ledCS.ledPin,LOW);
    }
  }
}

//Triggering the DTSC US
void updateDTSC(unsigned long now){
  if (trial.trialIsRunning && (stimPairType=="US"||stimPairType=="CS_US")){
    //Turning US off while correct trial type is running
    unsigned long DTSCStart = trial.trialStartMillis + trial.preCSdur + trial.CS_USinterval;
    unsigned long DTSCStop = DTSCStart + trial.USdur;
    if (DTSC_US.isOnDTSC && now > DTSCStop){
      DTSC_US.isOnDTSC = false;
      serialOut(now,"DTSCUSoff",trial.currentTrial);
      digitalWrite(DTSC_US.DTSCPin,LOW);
    }
  }
}


/*Loop*/
void loop()
{
  //Counting for each session/trial
  unsigned long now = millis();
  msIntoSession = now-trial.sessionStartMillis;
  msIntoTrial = now-trial.trialStartMillis;
  
  //Booleans for if we're in the pre-CS time or interval between CS and US
	inMotionCatch = (msIntoTrial > (trial.preCSdur - motionCatchDur)) && (msIntoTrial < trial.preCSdur);
  inCS_USint = (msIntoTrial > trial.preCSdur) && (msIntoTrial < (trial.preCSdur + trial.CS_USinterval));
  
  if (Serial.available() > 0) {
    String inString = Serial.readStringUntil('\n');
    inString.replace("\n","");
    inString.replace("\r","");
    SerialIn(now, inString);
  }

  //Stop at end of trialDur if trialIsRunning
  if (now > (trial.trialStartMillis + trial.trialDur) && trial.trialIsRunning && trial.sessionIsRunning){
    stopTrial(now);
    //we set ITI inside stopTrial function
  }

  //Start a trial at the end of the ITI period
  if (now>trial.ITIstartMillis + interTrialInterval && !trial.trialIsRunning && trial.sessionIsRunning){
    startTrial(now);
  }
  

  //Updating all hardware components
  updateEncoder(now,inMotionCatch,inCS_USint);

  updateLED(now);

  updateDTSC(now);

//  updateTrialPin(now);
  
  delayMicroseconds(300); //ms

}
