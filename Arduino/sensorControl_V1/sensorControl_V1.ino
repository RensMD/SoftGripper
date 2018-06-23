// TODO: Receive number of sensors used from matlab

/* Libraries */
/* ********* */
#include <Wire.h>
#include "TCS34725_quickswitch.h"

/* Settings */
/* ******** */
// Sensors
const byte tcsUsed = 4;
const int integrationTime = 60; // Should be somewhat higher than integration time defined in sensor objects

// TCA
const int SDApin = SDA;
const int SCLpin = SCL;
#define TCAADDR 0x70

// Debugging
//#define TIME_MODE // Uncomment to enable

/* Variables */
/* ********* */
// Create sensor objects
TCS34725 tcs0 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs1 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs2 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs3 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcs4 = TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_60X);
TCS34725 tcsArray [5] = {tcs0,tcs1,tcs2,tcs3,tcs4};

// Create color variables
uint16_t clear_tcs0, clear_tcs1, clear_tcs2, clear_tcs3, clear_tcs4;
uint16_t red_tcs0, red_tcs1, red_tcs2, red_tcs3, red_tcs4;
uint16_t green_tcs0, green_tcs1, green_tcs2, green_tcs3, green_tcs4;
uint16_t blue_tcs0, blue_tcs1, blue_tcs2, blue_tcs3, blue_tcs4;

// Create Arrays of color variables
uint16_t clearArray [5] = {clear_tcs0, clear_tcs1, clear_tcs2, clear_tcs3, clear_tcs4};
uint16_t redArray [5] = {red_tcs0, red_tcs1, red_tcs2, red_tcs3, red_tcs4};
uint16_t greenArray [5] = {green_tcs0, green_tcs1, green_tcs2, green_tcs3, green_tcs4};
uint16_t blueArray [5] = {blue_tcs0, blue_tcs1, blue_tcs2, blue_tcs3, blue_tcs4};

/* Functions */
/* ********* */
// TCA switch select
void tcaselect(uint8_t i){
   if (i > 7) return;

   Wire.beginTransmission(TCAADDR);
   Wire.write(1 << i);
   Wire.endTransmission();
}

// Print values over serial connection
void printValues(byte n){
   Serial.print(redArray[n]);
   Serial.print(",");
   Serial.print(greenArray[n]);
   Serial.print(",");
   Serial.print(blueArray[n]);
   Serial.print(",");
   Serial.println(clearArray[n]);
}

/* Setup */
/* ***** */
void setup() {
   Wire.begin(SDApin,SCLpin);
   Serial.begin(9600);
   
   /* Initialise Sensors */
   for(byte n = 0; n < tcsUsed; n++){
      tcaselect(n);
      if(!tcsArray[n].begin())  Serial.println(n);
   }
}

/* Loop */
/* **** */
void loop(){
  
  delay(integrationTime);

  // Wait for incoming command
  while (Serial.available()) {
    Serial.read();
  }
  while (Serial.available() <= 0) {
    delay(10);
  }

  // If in time mode, set time before reading sensors
  #ifdef TIME_MODE
    Serial.print("Data Time:");
    unsigned long timeBefore = millis();
  #endif

  // Get data from all sensors
  for(byte n = 0; n < tcsUsed; n++){
     tcaselect(n);
     tcsArray[n].getRawData(&redArray[n], &greenArray[n], &blueArray[n], &clearArray[n]);
  }

  // If in time mode, print duration of reading sensors
  #ifdef TIME_MODE
    Serial.println(millis() - timeBefore);
  #endif

  // Send sensordata over serial connection
  for(byte n = 0; n < tcsUsed; n++){
    printValues(n);
  }
}
