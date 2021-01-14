/*
 * Author: Joey Broussard
 * PNI, 20200820
 * DTSC rig master arduino controller sketch
 * 
 * V1.0 - rewritten from eyeblink3_4 to interface with DTSC_US and
 *  eyeblink_app.py
 * V1.1 - added pre-CS backing catch
 * V1.2
 * V1.3 - Harmonized to Bonni lab approach including
 *		- Inter-trial quiescence period required for trial to start with timeout
 *		- Only collecting motion during 50ms before US
 * 		- Use of interrupt attachment for motion-detection functions. Note that to	
 *			run in parallel with Encoder library, need Arduino Due
 * V2.0 - Broke out trial and 2P acquisition pins to allow asynchronous 
 *				activation to allow baseline acquisition during uncertain trial starts
 *		- Allowed full write of rotary encoder data during ITIs
 *		- Reworked the ISI motion-catch variables into relevnat structures to
 *			allow in-loop updates to states including in ITIs
 *		- Explicitly defined a structure and pin for 2P acquisition
 *		- Clarified loop logic by incorporating all unstructured variables
 *			into appropriate structure and removing all non-loop delays
 *		- Routed all rotary encoder detection through distinct (timing and variable)
 *			processes, all of which now poll the Encoder cache in the 
 *			update encoder function
 * V2.1 - cleaned up 2P variables
 *		- Changed from 1 to 2 pins for 2P control
 *		- 2P now runs throughout session, with new files intialized where
 *			acq stop/starts were in V2.0
 *    - Automatically activate and free motor at session beginning and end
 *    - Implemented tiered US -> bigger CR = smaller US
 */
 
#include "Arduino.h"
#include <Wire.h>
#include <Encoder.h> // http://www.pjrc.com/teensy/td_libs_Encoder.html

/////////////////////////////////////////////////////////////
/*Structure and state definitions*/
//Defining trial structure with associated attributes
struct trial
{
  //Session timing and numbering
  boolean sessionIsRunning;//flag for starting and stopping session
  int sessionNumber;//correspnds to session number
  unsigned long sessionStartMillis;//ms time at which session starts
  unsigned long sessionDur; //ms, numEpoch*epochDur
	unsigned long msIntoSession;
  //Trial timing and numbering
  boolean trialIsRunning;
  int currentTrial;
  unsigned long trialDur;//ms trial duration
  unsigned long numTrial;//number of trials we want
  unsigned long trialStartMillis; //ms time trial starts
	unsigned long msIntoTrial;
  unsigned long ITIlow; //ms lowest inter-trial interval
  unsigned long ITIhigh; //ms highest inter-trial interval  
  unsigned long ITIstartMillis;//ms time at which interTrialInterval starts
	unsigned long msIntoITI;//ms since ITI began
	unsigned long ITIstillStartMillis; // time at which animal became still
	unsigned long msIntoStillITI; // ms since animal last moved
	
  //Trial pin
  boolean pinOnOff;//controls transitioning pin state
  int trialPin;//pin for projecting current trial state
  //CS and US
	String stimPairType;// rng used to determine CS_US, CS, or US trial type
  unsigned long CSstartMillis; //millis at start of currentPulse
  unsigned long  preCSdur; //ms time in trial before CS
  unsigned long CSdur; //ms CS duration
	boolean inCS; // flag for CS timing
  unsigned long CRcountDur; // legnth of time to detect CR
  boolean inCRcount; // flag for when to detect CR motion
  unsigned long USdur;//ms
	boolean inUS; //flag for US timing
  unsigned long CS_USinterval;//ms
  unsigned int percentUS;//percent trials user wants to be US only trials
  unsigned int percentCS;//percent trials user wants to be CS only
  //motor
  int useMotor;//{motorOn,motorLocked,motorFree}
  unsigned long motorSpeed; //rev/sec
  
};


struct rotaryencoder
{
  int pinA = 3; // use pin 2 for A output
  int pinB = 2; // use pin 3 for B output
	//reading encoder
	unsigned long time = 0; // keeps current time at milli precision to poll once per milli
	long currentPos = 0; // instantaneous readout of encoder position
	long diffPos = 0; //difference between this and last poll position
	long lastPos = 0; // holds previous polled value
	
	//for printing distance traveled at regular intervals
	long printVal = 0; // value pronted ot csv
	long printTime = 0; //counter for when to make next print
	long printInterval = 14; //length of time between prints
	
	//Track CR motion
	boolean isOnCRcount = false; // on during CR detection
	long CRcount = 0; // tracks wheel motion during CR detection
	long CRthresh1 = -4; // sensitivity level for CR detection, lower number less sensitive
  long CRthresh2 = -10; // sensitivity level for the very small bop
	//Track ITI motion
	boolean notStill = false; // true if animal breaks fixation too frquently in ITI
	boolean still = false; // true if animal keeps fixation during ITI
	boolean isOnMotionCount = false; // true when traking ITI motion
  boolean resetMotionCount = true; // true if animal moved or we're starting motion tracking
	const int lenDetect = 20; // ms rolling interval over which we track motion
	long motionArr[20]; // array to hold detected motion over last lenDtect time bins
	long sumMotion = 0; // sum of motionArr
	long motionThresh = 8; // threshold for saying an animal moved
	
};

struct twoP
{
	boolean isOnTwoP = false; //
	boolean toggleState = false;//
	int twoPpin = 52; //pin for 2P activation
	int fileChangePin = 51; //pin to change the file
	boolean fileChangeOn = false; // signal to make a file break
	boolean changeFile = false; // goes true when rotary encoder motion conditions met
  boolean reportNew = false; //
	unsigned long fileChangeStart = 0; //minimum time between activation of 2P
	unsigned long fileChangeInt = 20; //counting until minimum inter-2P activation interval reached
  boolean runTilTrial = false; // if true, don't shut off 2P until after trial
	unsigned long preTrialImgDur = 500; // pre-trial time to collect 2P data
};

struct ledCS
{
  boolean isOnLED; // use to toggle on and off
  int ledPin; // pin for the CS
};

struct DTSC_US
{
	boolean armedDTSC = false; // encoder sets true when motion conditions met
  boolean isOnDTSC = false;// use to toggel on and off DTSC trigger signal
  int DTSCPin;// pin for the US
};


//Version, defining structures and aliases
String versionStr = "DTSC2_1.cpp";
typedef struct trial Trial;
typedef struct rotaryencoder RotaryEncoder;
typedef struct twoP TwoP;
typedef struct ledCS LedCS;
typedef struct DTSC_US dtsc_US;

//Instances of all hardware-associated structures
Trial trial;
RotaryEncoder rotaryencoder;
TwoP twoP;
LedCS ledCS;
dtsc_US DTSC_US;

//Stepper and Encoder objects defined per relevant libraries
Encoder myEncoder(rotaryencoder.pinA, rotaryencoder.pinB);

/////////////////////////////////////////////////////////////
/*Setup, mostly declaring default structure values*/
void setup()
{ 
  //Initialize serial
  Serial.begin(115200);
  while(!Serial);
  
  //trial
  trial.sessionIsRunning = false;
  trial.sessionNumber = 0;
  trial.sessionStartMillis = 0;

  trial.trialIsRunning = false;
  trial.trialDur = 3000; // epoch has to be >= (preDur + xxx + postDur)
  trial.numTrial = 1;
  
  trial.sessionDur = (trial.numTrial*trial.trialDur); //

  trial.useMotor = 0; //0 = motorOn, 1 = motorLocked, 2 = motorFree
  trial.motorSpeed = 500; //step/sec

  trial.preCSdur = 1000;
  trial.CSdur = 250;
  trial.USdur = 50;
  trial.CS_USinterval = trial.CSdur - trial.USdur;
  trial.CRcountDur = 50;
  trial.ITIlow = 1000;//ms
  trial.ITIhigh = 3000;//ms
  trial.ITIstartMillis = 0;//ms
  trial.percentUS = 0;//percent US only trials
  trial.percentCS = 10;//percent CS only trials

  trial.trialPin = 7;//pin for conveying trial state
  trial.pinOnOff = false;//trial didn't just end
  pinMode(trial.trialPin, OUTPUT);
  digitalWrite(trial.trialPin, LOW);
  
  //rotary encoder
  rotaryencoder.pinA = 3;
  rotaryencoder.pinB = 2;
	//rotaryencoder.motionArr
  //

  //CS/US structure and pin settings, intially at Arduino grnd
  ledCS.ledPin = 4;
  ledCS.isOnLED = false;
  pinMode(ledCS.ledPin,OUTPUT);
  digitalWrite(ledCS.ledPin,LOW);
  DTSC_US.DTSCPin = 5;
  DTSC_US.isOnDTSC = false;
  pinMode(DTSC_US.DTSCPin,OUTPUT);
  digitalWrite(DTSC_US.DTSCPin,LOW);
  pinMode(twoP.twoPpin,OUTPUT);
  digitalWrite(twoP.twoPpin,LOW);
  pinMode(twoP.fileChangePin,OUTPUT);
  digitalWrite(twoP.fileChangePin,LOW);

  //give random seed to random number generator from analog 0
  randomSeed(analogRead(0));

  //Initialize as I2C master
  Wire.begin();//join I2C bus with no given address you master, you
	
	//Set any pins used in other setups low
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
}

/////////////////////////////////////////////////////////////
/*Starting and ending Trials (sessions)*/
//Start session
void startSession(unsigned long now) {
  if (trial.trialIsRunning==false) {
		trial.sessionIsRunning = true;
		
    //I2C to inactivate slp pin DTSC wheel
    wireOut(0,0);
		
		//Activate 2P for pre-trial interval
		serialOut(now,"2Pon",-1);
		digitalWrite(twoP.twoPpin,HIGH);
    twoP.isOnTwoP = true;
		delay(twoP.preTrialImgDur);
		now = millis();
		
		//Session start stuff
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

    trial.trialIsRunning = true;

    //Calculate trial type
    unsigned int RNG = random(1,101);
    if (RNG <= trial.percentUS){
       trial.stimPairType = "US";
    } else if (RNG > trial.percentUS && RNG <= (trial.percentUS + trial.percentCS)){
      trial.stimPairType = "CS";
    } else if (RNG > (trial.percentUS + trial.percentCS)){
      trial.stimPairType = "CS_US";
    }
    serialOut(now,trial.stimPairType,trial.currentTrial);
    
    
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
      trial.stimPairType = "US";
    } else if (RNG > trial.percentUS && RNG <= (trial.percentUS + trial.percentCS)){
      trial.stimPairType = "CS";
    } else if (RNG > (trial.percentUS + trial.percentCS)){
      trial.stimPairType = "CS_US";
    }
    serialOut(now,trial.stimPairType,trial.currentTrial);
  }
}

//End trial
void stopTrial(unsigned long now) {
  //If this is the last trial, end session
  if (trial.currentTrial == trial.numTrial) {
    stopSession(now);
    return;
  }
  trial.trialIsRunning = false;
  digitalWrite(trial.trialPin,LOW);
  serialOut(now, "stopTrial", trial.currentTrial);

  //Reset the 2P
  twoP.changeFile = true;
  twoP.reportNew = true;
  
  //Set time to wait until next trial starts
  trial.ITIstartMillis = now;
  trial.ITIstillStartMillis = now;
	
  
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
	twoP.isOnTwoP = false;
	digitalWrite(twoP.twoPpin,LOW);
	serialOut(now,"2Poff",trial.currentTrial);
  trial.sessionNumber += 1;
	trial.currentTrial = 0;

  //I2C to inactivate slp pin DTSC wheel
  wireOut(0,1);
  Serial.println("Motor free");
  

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
    Serial.println("Got start!");
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
  Serial.println("interTrialInteval=" + String(trial.ITIlow) + String(trial.ITIhigh)); 

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
    trial.ITIhigh = value;
    Serial.println("trial.interTrialIntervalHigh=" + String(trial.ITIhigh));
  } else if (name=="interTrialIntervalLow") {
    trial.ITIlow = value;
    Serial.println("trial.interTrialIntervalLow=" + String(trial.ITIlow));
    
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
void updateEncoder(unsigned long now) {

	//Poll once every milli
	if (rotaryencoder.time<now&&trial.sessionIsRunning){
		rotaryencoder.time = now;
		//update encoder position
		rotaryencoder.currentPos = myEncoder.read();
		rotaryencoder.diffPos = rotaryencoder.currentPos - rotaryencoder.lastPos;
		rotaryencoder.lastPos = rotaryencoder.currentPos;
		//update print value
		rotaryencoder.printVal += rotaryencoder.diffPos;
		
		//print out position at specified intervals
		if(now - rotaryencoder.printTime > rotaryencoder.printInterval){
			serialOut(now,"rotary",rotaryencoder.printVal);
			rotaryencoder.printVal = 0;
			rotaryencoder.printTime = now;
		}
		
		//sum encoder output during CR detection period, update US flag
		if(trial.inCRcount && !rotaryencoder.isOnCRcount){
			rotaryencoder.isOnCRcount = true;
			rotaryencoder.CRcount = 0;
			serialOut(now,"CRcountOn",trial.currentTrial);
		}else if(trial.inCRcount&&rotaryencoder.isOnCRcount){
			rotaryencoder.CRcount += rotaryencoder.diffPos;
		}else if(!trial.inCRcount&&rotaryencoder.isOnCRcount){
			rotaryencoder.isOnCRcount = false;
			serialOut(now,"CRcount",rotaryencoder.CRcount);
			if((trial.stimPairType=="US"||trial.stimPairType=="CS_US")){
				DTSC_US.armedDTSC = true;
				if (rotaryencoder.CRcount >= rotaryencoder.CRthresh1){
					wireOut(1,2);//send big attack if animal didn't move back
					serialOut(now,"bigUSon",rotaryencoder.CRcount);				
				}else if(rotaryencoder.CRcount >= rotaryencoder.CRthresh1){
          wireOut(1,1);//send med attack if animal moved back a little
          serialOut(now,"medUSon",rotaryencoder.CRcount); 
				}else{
					wireOut(1,0);//send little attack if animal moved back
					serialOut(now,"smallUSon",rotaryencoder.CRcount);
				}
			}
		}
		
		//Motion detection during ITI
		//If animal moves too much during ITI
		if(!trial.trialIsRunning){
			if(trial.msIntoITI>trial.ITIhigh - trial.ITIlow){	
				if(!rotaryencoder.notStill){
          //serialOut(now,"notStillFlag",trial.currentTrial);
					rotaryencoder.notStill = true;
					rotaryencoder.isOnMotionCount = false;
          twoP.reportNew = false;
          trial.ITIstillStartMillis = now;
					
				//Reset 2P if we've reached that interval after animal hasn't been still; we'll turn off at stopTrial
				}else if(rotaryencoder.notStill && trial.ITIhigh - trial.msIntoITI < twoP.preTrialImgDur && !twoP.changeFile && !twoP.runTilTrial){
					//serialOut(now,"notStillNewFile",trial.currentTrial);
					twoP.changeFile = true;
          twoP.runTilTrial = true;
          twoP.reportNew = true;

				}
				
			//Initialize motion detection	
			}else if(rotaryencoder.resetMotionCount){
        //serialOut(now,"motionDetectOn",1);
				rotaryencoder.isOnMotionCount = true;
				rotaryencoder.resetMotionCount = false;
				trial.ITIstillStartMillis = now;
        rotaryencoder.sumMotion = 0;
				//Initialize motion detection as 0's array
				for(int i=0;i<rotaryencoder.lenDetect;i++){rotaryencoder.motionArr[i]=0;}
			
			//Update the count based on how much mouse has moved since last poll	
			}else if(rotaryencoder.isOnMotionCount){
				//circular permute array by 1 to right, then add latest motion
				for(int i = rotaryencoder.lenDetect - 1;i>=0;i--){
					if(i>0){rotaryencoder.motionArr[i] = rotaryencoder.motionArr[i-1];
					}else{rotaryencoder.motionArr[i] = abs(rotaryencoder.diffPos);//random(180)*0.01;
					}
					rotaryencoder.sumMotion += rotaryencoder.motionArr[i];
				}
        //serialOut(now,"sumMotion",rotaryencoder.sumMotion);
        
				//If motion above threshold, reset motion array
				if(rotaryencoder.sumMotion>rotaryencoder.motionThresh){
          //serialOut(now,"Moved",rotaryencoder.sumMotion);
					rotaryencoder.resetMotionCount = true;
					twoP.changeFile = true;
          twoP.reportNew = false;
          rotaryencoder.still = false;
          
          
				//Otherwise start recording 2P when we reach ITIlow - preTrialImgDur
				}else if(trial.ITIlow - trial.msIntoStillITI < twoP.preTrialImgDur && !twoP.changeFile){
					serialOut(now,"stillFlag",trial.currentTrial);
					twoP.changeFile = true;
          twoP.reportNew = true;
					rotaryencoder.isOnMotionCount = false;
					rotaryencoder.resetMotionCount = false;
          rotaryencoder.still = true;

				}
        rotaryencoder.sumMotion = 0;
			}
		//Clear flags at the end of the ITI
		}else if(trial.trialIsRunning && (rotaryencoder.notStill || rotaryencoder.still || !rotaryencoder.resetMotionCount)){
      //serialOut(now,"flag6",1);
      //serialOut(now,"flag2crit",trial.ITIhigh - trial.msIntoITI);
			if(rotaryencoder.notStill){
				serialOut(now,"NotStill",trial.currentTrial);
			}else if(rotaryencoder.still){
				serialOut(now,"Still",trial.currentTrial);
			}
			rotaryencoder.notStill = false;
      rotaryencoder.still = false;
			rotaryencoder.isOnMotionCount = false;
			rotaryencoder.resetMotionCount = true;
      twoP.runTilTrial = false;
		}
		
	}
 
}


//Triggering the LED CS
void updateLED(unsigned long now){
  if (trial.trialIsRunning && (trial.stimPairType=="CS"||trial.stimPairType=="CS_US")){
    
		if(trial.inCS && !ledCS.isOnLED){
			ledCS.isOnLED = true;
			serialOut(now,"ledCSon",trial.currentTrial);
      digitalWrite(ledCS.ledPin,HIGH);
		}else if(!trial.inCS && ledCS.isOnLED){
			ledCS.isOnLED = false;
      serialOut(now,"ledCSoff",trial.currentTrial);
      digitalWrite(ledCS.ledPin,LOW);
		}
  }
}

//Triggering the DTSC US
void updateDTSC(unsigned long now){
  if (trial.trialIsRunning  && (trial.stimPairType=="US"||trial.stimPairType=="CS_US")){
    if(trial.inUS && !DTSC_US.isOnDTSC && DTSC_US.armedDTSC){
			DTSC_US.isOnDTSC = true;
			digitalWrite(DTSC_US.DTSCPin,HIGH);
		}
		//Turning US off while correct trial type is running
    if (!trial.inUS && DTSC_US.isOnDTSC && DTSC_US.armedDTSC){
      DTSC_US.isOnDTSC = false;
			DTSC_US.armedDTSC = false;
      serialOut(now,"DTSCUSoff",trial.currentTrial);
      digitalWrite(DTSC_US.DTSCPin,LOW);
    }
  }
}

//Triggering 2P acquisitions
void update2P(unsigned long now){
	//Turn 2P scanning on and off
	if(!twoP.isOnTwoP && twoP.toggleState && trial.sessionIsRunning){
		twoP.isOnTwoP = true;
		digitalWrite(twoP.twoPpin,HIGH);
		serialOut(now,"2Pon",trial.currentTrial);
		twoP.toggleState = false;
	}else if(twoP.isOnTwoP && twoP.toggleState && trial.sessionIsRunning){
		twoP.isOnTwoP = false;
		digitalWrite(twoP.twoPpin,LOW);
		serialOut(now,"2Poff",trial.currentTrial);
		twoP.toggleState = false;
	}
	//Make a new file
	if(twoP.isOnTwoP && twoP.changeFile && twoP.reportNew && trial.sessionIsRunning){
		twoP.changeFile = false;
		digitalWrite(twoP.fileChangePin,HIGH);
		serialOut(now,"newFile",trial.currentTrial);
		twoP.fileChangeStart = now;
	}else if(twoP.isOnTwoP && twoP.changeFile && !twoP.reportNew && trial.sessionIsRunning){
    twoP.changeFile = false;
    digitalWrite(twoP.fileChangePin,HIGH);
    twoP.fileChangeStart = now;
	}else if(twoP.isOnTwoP && now - twoP.fileChangeStart>twoP.fileChangeInt && trial.sessionIsRunning){
		digitalWrite(twoP.fileChangePin,LOW);
	}
	
}


/*Loop*/
void loop()
{
  //Counting for each session/trial/ITI
  unsigned long now = millis();
  trial.msIntoSession = now-trial.sessionStartMillis;
  trial.msIntoTrial = now-trial.trialStartMillis;
	trial.msIntoITI = now - trial.ITIstartMillis;
	trial.msIntoStillITI = now - trial.ITIstillStartMillis;
  
  //Booleans for state controller
	trial.inCRcount = (trial.msIntoTrial > (trial.preCSdur + trial.CS_USinterval - trial.CRcountDur)) && (trial.msIntoTrial < (trial.preCSdur + trial.CS_USinterval));//interval to deterimine if animal makes CR
	trial.inCS = (trial.msIntoTrial > trial.preCSdur) && (trial.msIntoTrial < trial.preCSdur + trial.CSdur);
  trial.inUS = (trial.msIntoTrial > trial.preCSdur + trial.CS_USinterval) && (trial.msIntoTrial < (trial.preCSdur + trial.CS_USinterval + trial.USdur));

  //Start a trial at the end of the ITI period
  if (!trial.trialIsRunning && trial.sessionIsRunning  && (trial.msIntoITI>trial.ITIhigh || trial.msIntoStillITI>trial.ITIlow)){
    startTrial(now);
  }

  //Updating all hardware components
  updateEncoder(now);
  updateLED(now);
  updateDTSC(now);
	update2P(now);

  //Stop at end of trialDur if trialIsRunning
  if (now > (trial.trialStartMillis + trial.trialDur) && trial.trialIsRunning && trial.sessionIsRunning){
    stopTrial(now);
    //we set ITI inside stopTrial function
  }
  
  if (Serial.available() > 0) {
    String inString = Serial.readStringUntil('\n');
    inString.replace("\n","");
    inString.replace("\r","");
    SerialIn(now, inString);
  }
	
  delayMicroseconds(50); //us

}
