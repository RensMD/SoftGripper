#include <Wire.h>
#include "Adafruit_TCS34725_Adap.h"

#define TCAADDR 0x70

const int SDApin = 12;
const int SCLpin = 13;
const int PWMpin = 14;

Adafruit_TCS34725 tcs0 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs3 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs4 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs5 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs6 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs7 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcsArray [8] = {tcs0,tcs1,tcs2,tcs3,tcs4,tcs5,tcs6,tcs7};

byte tcsUsed = 1;

uint16_t clear, red, green, blue;
float combinedColors;

int pwmOutput = 0; 


void setup(){
  
   Wire.begin(SDApin,SCLpin);
   Serial.begin(9600);
   Serial.println();

   pinMode(PWMpin, OUTPUT);

   /* Initialise Sensors */
   for(int n = 0; n < tcsUsed; n++){
      tcaselect(n);
      if(!tcsArray[n].begin())  Serial.println(n);
   }
   
}


void loop(){
  
  while (Serial.available() <= 0) {
    delay(300);
  }
  
  char rx_byte = ' ';
  String rx_str = "";
  while(rx_byte != '\n'){
    rx_byte = Serial.read();
    rx_str += rx_byte;
  }

  pwmOutput = rx_str.toInt();
  analogWrite(PWMpin, pwmOutput);

  // TODO determine delay for letting actuator settle
  delay(1000);
  
  for(int n = 0; n < tcsUsed; n++){
    tcaselect(n);
    sendColor(n);
    delay(10);
  }
  
}

void sendColor(int n){
  
  tcsArray[n].getRawData(&red, &green, &blue, &clear);
  
  Serial.print(n);
  Serial.print(",");
  Serial.print(red);
  Serial.print(",");
  Serial.print(green);
  Serial.print(",");
  Serial.print(blue);
  Serial.print(",");
  Serial.println(clear);

}

void tcaselect(uint8_t i){
  
  if (i > 7) return;
  
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
  
}
