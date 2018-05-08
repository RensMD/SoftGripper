#include <Wire.h>
#include "TCS34725_quickswitch.h"

#define TCAADDR 0x70

const int SDApin = 13;
const int SCLpin = 12;

const int integrationTime = 700;
const byte tcsUsed = 5;

TCS34725 tcs0 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs1 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs2 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs3 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs4 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcsArray [5] = {tcs0,tcs1,tcs2,tcs3,tcs4};

uint16_t clear_tcs0, clear_tcs1, clear_tcs2, clear_tcs3, clear_tcs4;
uint16_t red_tcs0, red_tcs1, red_tcs2, red_tcs3, red_tcs4;
uint16_t green_tcs0, green_tcs1, green_tcs2, green_tcs3, green_tcs4;
uint16_t blue_tcs0, blue_tcs1, blue_tcs2, blue_tcs3, blue_tcs4;

uint16_t clearArray [5] = {clear_tcs0, clear_tcs1, clear_tcs2, clear_tcs3, clear_tcs4};
uint16_t redArray [5] = {red_tcs0, red_tcs1, red_tcs2, red_tcs3, red_tcs4};
uint16_t greenArray [5] = {green_tcs0, green_tcs1, green_tcs2, green_tcs3, green_tcs4};
uint16_t blueArray [5] = {blue_tcs0, blue_tcs1, blue_tcs2, blue_tcs3, blue_tcs4};

void tcaselect(uint8_t i){
   if (i > 7) return;

   Wire.beginTransmission(TCAADDR);
   Wire.write(1 << i);
   Wire.endTransmission();
}

void printValues(byte n){
   Serial.print(n);
   Serial.print(",");
   Serial.print(redArray[n]);
   Serial.print(",");
   Serial.print(greenArray[n]);
   Serial.print(",");
   Serial.print(blueArray[n]);
   Serial.print(",");
   Serial.println(clearArray[n]);
}

void setup() {
   Wire.begin(SDApin,SCLpin);
   Serial.begin(9600);
   
   /* Initialise Sensors */
   for(byte n = 0; n < tcsUsed; n++){
      tcaselect(n);
      if(!tcsArray[n].begin())  Serial.println(n);
   }
}

void loop(){
  
  delay(integrationTime);
//  delay(500);
  while (Serial.available()) {
    Serial.read();
  }
  
  while (Serial.available() <= 0) {
    delay(10);
  }

//  Serial.print("Data Time:");
//  unsigned long timeBefore = millis();
  
  for(byte n = 0; n < tcsUsed; n++){
     tcaselect(n);
     tcsArray[n].getRawData(&redArray[n], &greenArray[n], &blueArray[n], &clearArray[n]);
  }
  
//  Serial.println(millis() - timeBefore);
  
  for(byte n = 0; n < tcsUsed; n++){
    printValues(n);
  }
}

