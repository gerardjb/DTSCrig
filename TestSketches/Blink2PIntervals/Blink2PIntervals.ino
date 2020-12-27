/*
 * Author: Joey Broussard
 * PNI, 20201224
 * Blink2PIntervals
 * 
 * Purpose: minimal sketch to test proper performance of the DTSC2_0 interaction
 * with scanimage user functions. Should cause following behavior on scanimage interface:
 * 1. Trial triggering on loop will only save and increment for trials in the timing
 *    range specified in the scanimage user function "funOff.mat". Other lengths save
 *    and overwrite by decrementing the Acquisition number (i.e. hSI.hScanners{i}.logFileCounter)
 * 2. Only trials beyond upper trial timing range cause laser to set to last focus value.
 *    Other lengths cause reset to 0.1%
 */

int tPin = 13; //current scanimage trigger pin
int ct = 0; //limit total number of loops
int timer = 100; //rest between scanimage acquisition loops
long trialLen[] = {3000,9000,100,9000}; //array of TTL lengths

void setup() {
  // Set current scanimage interaction pin
  pinMode(tPin,OUTPUT);
  digitalWrite(tPin,LOW);

  //Serial
  Serial.begin(9600);
  delay(1000);
}

void SerialIn(String str){
  if (str.length()==0){
    return;
  }
  if (str == "go"){
    ct = 5;
  }else if (str == "stop"){
    ct = 0;
  }
}

void loop() {
  //Set of two of each trialLen per ct
  if (Serial.available()>0){
    String inString = Serial.readStringUntil('\n');
    inString.replace("\n","");
    inString.replace("\r","");
    SerialIn(inString);
  }
  while(ct>0){
    for(int iLen = 0;iLen<sizeof(trialLen)/sizeof(trialLen[0]);iLen++){
      Serial.println("length =" + String(trialLen[iLen]) + "count = " + String(ct));
      digitalWrite(tPin,HIGH);
      delay(trialLen[iLen]);
      digitalWrite(tPin,LOW);
      delay(timer);
      if (Serial.available()>0){
        String inString = Serial.readStringUntil('\n');
        inString.replace("\n","");
        inString.replace("\r","");
        SerialIn(inString);
      }
    }
    ct--;
    
  }
  
}
