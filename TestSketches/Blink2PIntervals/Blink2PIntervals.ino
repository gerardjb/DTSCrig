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
int ct = 5; //limit total number of loops
int reps = 2; //number of repititions per trialLen in each ct
int timer = 100; //rest between scanimage acquisition loops
long trialLen[] = {100,3000,9000};

void setup() {
  // Set current scanimage interaction pin
  pinMode(tPin,OUTPUT);
  digitalWrite(tPin,LOW);

  //Serial
  Serial.begin(9600);
}

void loop() {
  //Set of two of each trialLen per ct
  delay(3000);
  while(ct>0){
    for(int iLen = 0;iLen<sizeof(trialLen)/sizeof(trialLen[0]);iLen++){
      for(int rep = 0;rep<reps;rep++){
        Serial.println("length =" + String(trialLen[iLen]) + ", rep = " + String(rep));
        Serial.println("count = " + String(ct));
        digitalWrite(tPin,HIGH);
        delay(trialLen[iLen]);
        digitalWrite(tPin,LOW);
        delay(timer);
      }
    }
    ct--;
  }
  
}
