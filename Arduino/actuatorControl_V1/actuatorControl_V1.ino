///////////////////////////////////////////////////////////////
//by Evelijn Verboom, Ines Arnaiz, Myron Wouts and Flip Colin//
///////////////mechatronics course IO3050 2018/////////////////
///////////////////////////////////////////////////////////////

#include <Arduino.h>
#include "DRV8825.h"
#include <PID_v1.h>
#include <AccelStepper.h>

// using a 200-step motor (most common)
#define MOTOR_STEPS 200

// configure the pins connected
#define DIR 8
#define STEP 9
#define MS1 10
#define MS2 11
#define MS3 7

A4988 stepperGripper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

// stepper library for pump stepper
AccelStepper stepperPump(AccelStepper::DRIVER, 3, 2);

//tuning parameters//
/////////////////////
double Kp = 400;
double Ki = 10;
double Kd = 6;

int timePumpOn = 6000; //how long the pump is turned on in milisec

float Ang = 0;      //starting angle
float maxAng = 250; //set maximum angle

float maxPres = 180; //set maximum pressure
float minPres = 47; //set minimum pressure

int val = 0; //commands for serial communication

//pins connected//
//////////////////
int stepPin = 3; // Easy driver Step to pin 3 of the arduino
int directionPin = 2; // Easy driver dir to pin 2 of the arduino

int solPinVac = 4;
int pumpPinVac = 10;
int pumpPin = 6;

int endswitch1Pin = 12;//switch at the high pressure end (srynge empty)
int endswitch2Pin = 13;//switch at the low pressure end (srynge full)

int pSensor = A0; //pressure sensor

//Initialisation other variables//
//////////////////////////////////

//PRESSURE
double measVoltage; // voltage meassured
double measPressure; //measured pressure  = InputPID
double desPressure;  //desired pressure = SetpointPID

//PUMP
long timePutOn = 0; // the moment the pump has been turned on (does not have to be set)
long timePutOn2 = 0; // the moment the vacuum pump has been turned on (does not have to be set)
boolean pumpOn = false; //if the pump is turned on
boolean pumpOn2 = false; //if the vacuum pump is turned on

//PID
double outputPID;
PID myPID(&measPressure, &outputPID, &desPressure, Kp, Ki, Kd, DIRECT);

//for debug print
double lastPrintTime = 0;

//////////////
// Set Up  //
////////////
void setup() {
  
  // STEPPERPINS
  pinMode (stepPin, OUTPUT);
  pinMode (directionPin, OUTPUT);

  // PRESSURE PIN
  pinMode(pSensor, INPUT);

  //PUMPS
  pinMode(pumpPin, OUTPUT);
  pinMode(pumpPinVac, OUTPUT);
  pinMode(solPinVac, OUTPUT);

  //SWITCHES
  pinMode(endswitch1Pin, INPUT);
  pinMode(endswitch2Pin, INPUT);

  digitalWrite(solPinVac, LOW);  // Serial Monitor
  Serial.begin(9600);

  //turn on PID and set output limits
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100000, 100000);
  desPressure = minPres; //set starting pressure

  //huge acceleration for pump stepper
  stepperPump.setAcceleration(20000000);

  stepperGripper.begin(50, 10);
  stepperGripper.rotate(30);
  stepperGripper.rotate(-30);
}

////////////////
// Main Loop//
//////////////
void loop() {
  while (Serial.available() == 0){
    
    measPressure = analogRead(pSensor);

    // PID
    myPID.Compute();
    moveStepper(outputPID);

    // Turn pump on and off
    // TURN PUMP ON IF SWITCH AT HIGH PRESSURE END IS TOUCHED
    if (digitalRead(endswitch1Pin) == LOW) {
      analogWrite(pumpPin, 50);
      timePutOn =  millis();
      pumpOn = true;
    }
    // TURN PUMP OFF AFTER THE TIMEPOMPON HAS PASSED
    if ((millis() - timePutOn > timePumpOn || digitalRead(endswitch2Pin) == LOW) && pumpOn) {
      digitalWrite(pumpPin, LOW);
      timePutOn = 0;
      pumpOn = false;
    }

    //TURN VACUUM PUMP ON IF SWITCH AT LOW PRESSURE END IS TOUCHED (AND OPEN SOLENOID)
    if (digitalRead(endswitch2Pin) == LOW) {
      analogWrite(pumpPinVac, 100);
      digitalWrite(solPinVac, HIGH);
      timePutOn2 =  millis();
      pumpOn2 = true;
    }
    //TURN VACUUM PUMP OFF AFTER THE TIMEPOMPON HAS PASSED
    if ((millis() - timePutOn2 > timePumpOn || digitalRead(endswitch1Pin) == LOW) && pumpOn2) {
      digitalWrite(solPinVac, LOW);
      digitalWrite(pumpPinVac, LOW);
      timePutOn = 0;
      pumpOn2 = false;
    }

//    //show pressures
//    if (millis() - lastPrintTime > 200) {
//      printDebug();
//      lastPrintTime = millis();
//    }
  }

  if (Serial.available() > 0){
    val = Serial.read();
    if (val == 'R') {
//      Ang = random(0, maxAng); //make Ang a random value between 0 and maximum steps
      Ang = readValue();
      stepperGripper.rotate(Ang);
//      Serial.println(Ang);
    }
    else if (val == 'S') {
      stepperGripper.rotate(-Ang);
    }
    else if (val == 'T') {
      desPressure = readValue();
//      desPressure = random(minPres, maxPres); //set desired pressure
//      Serial.println(desPressure);
    }
    else if (val == 'U') {
      desPressure = minPres;
    }
  }
}

int readValue() {
  delay(10);
  char inChar = ' ';
  String inString = " ";
  while (Serial.available() > 0) {
    inChar = Serial.read();
    inString += String(inChar);
  }
  return inString.toInt();
}

//Step function//
/////////////////
void moveStepper(double stepperMove) {
  int moveDir = 0;

  if (stepperMove > 0) {
    moveDir = -20000;
  }
  if (stepperMove <= 0) {
    moveDir = 20000;
  }
  
  stepperPump.move(moveDir);
  stepperPump.setMaxSpeed(abs(stepperMove));
  stepperPump.runSpeed();
}

void printDebug() {
  Serial.print(measPressure);
  Serial.print(",  ");
  Serial.print(desPressure);
  Serial.print(",  ");
  Serial.print(digitalRead(endswitch1Pin));
  Serial.print(",  ");
  Serial.println(digitalRead(endswitch2Pin));
}
