// Uncomment to enable:
//#define DEBUG_MODE

/* LIBRARIES */
#include <Arduino.h>
#include "DRV8825.h"
#include <PID_v1.h>
#include <AccelStepper.h>

/* PINS */
const int solPinVac   = 4;
const int pumpPin     = 6;
const int pumpPinVac  = 10;

const int endswitch1Pin = 12; // Switch at the high pressure end (srynge empty)
const int endswitch2Pin = 13; // Switch at the low pressure end (srynge full)
const int pSensor = A0; //pressure sensor

#define GRIP_DIR   8
#define GRIP_STEP  9
#define GRIP_MS1   10
#define GRIP_MS2   11
#define GRIP_MS3   7

#define PUMP_STEP  3
#define PUMP_DIR   2

/* STEPPER MOTORS */
#define GRIP_MOTOR_STEPS 200 // Using a 200-step motor (most common)

A4988 stepperGripper(GRIP_MOTOR_STEPS, GRIP_DIR, GRIP_STEP, GRIP_MS1, GRIP_MS2, GRIP_MS3);
AccelStepper stepperPump(AccelStepper::DRIVER, PUMP_STEP, PUMP_DIR);

double Ang = 0.0; // Starting angle
double minPres = 49.0;
double desPressure = 0.0;  // Desired pressure = SetpointPID
double measPressure = 0.0;
double range = 3.0;

/* PID CONTROL */
// Tuning parameters
double Kp = 400;
double Ki = 10;
double Kd = 6;

double outputPID;
PID myPID(&measPressure, &outputPID, &desPressure, Kp, Ki, Kd, DIRECT);

/* PUMPS */
int timePumpOn = 6000; //how long the pump is turned on in milisec
int timePumpOn2 = 2000; //how long the pump is turned on in milisec

long timePutOn = 0; // The moment the pump has been turned on (does not have to be set)
long timePutOn2 = 0; // The moment the vacuum pump has been turned on (does not have to be set)
boolean pumpOn = false; // If the pump is turned on
boolean pumpOn2 = false; // If the vacuum pump is turned on

//for debug print
#ifdef DEBUG_MODE
  double lastPrintTime = 0;
#endif

////////////
// Set Up //
////////////
void setup() {
  /* SETUP PINS */
  pinMode(pSensor, INPUT);
  pinMode(endswitch1Pin, INPUT);
  pinMode(endswitch2Pin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(pumpPinVac, OUTPUT);
  pinMode(solPinVac, OUTPUT);

  digitalWrite(solPinVac, LOW);

  /* SETUP COMMUNICATION */
  Serial.begin(9600);

  /* SETUP PID */
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100000, 100000);
  desPressure = minPres;

  /* SETUP STEPPERS */
  stepperPump.setAcceleration(20000000);
  stepperGripper.begin(50, 10);
  
  stepperGripper.rotate(30);
  stepperGripper.rotate(-30);
}

///////////////
// Main Loop //
///////////////
void loop() {
  while (Serial.available() == 0){
    measPressure = analogRead(pSensor);

    // PID
    myPID.Compute();
    moveStepper(outputPID);

    controlPump();

    #ifdef DEBUG_MODE
      if (millis() - lastPrintTime > 200) {
        printDebug();
        lastPrintTime = millis();
      }
    #endif
  }
  if (Serial.available() > 0){
    checkSerial();
  }
}

///////////////
// FUNCTIONS //
///////////////
/* Control the pumps */
void controlPump(){
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
  if ((millis() - timePutOn2 > timePumpOn2 || digitalRead(endswitch1Pin) == LOW) && pumpOn2) {
    digitalWrite(solPinVac, LOW);
    digitalWrite(pumpPinVac, LOW);
    timePutOn2 = 0;
    pumpOn2 = false;
  }
}

/* INITIALIZE PUMP AFTER COMMAND */
void initializePump(){
  
  analogWrite(pumpPin, 200);
  while (digitalRead(endswitch2Pin) == HIGH){    
    measPressure = analogRead(pSensor);
    myPID.Compute();
    moveStepper(outputPID);
  }
  digitalWrite(pumpPin, LOW);

  analogWrite(pumpPinVac, 100);
  digitalWrite(solPinVac, HIGH);
  unsigned long timeBefore = millis();
  while (millis() - timeBefore < 4000){    
    measPressure = analogRead(pSensor);
    myPID.Compute();
    moveStepper(outputPID);
  }
  digitalWrite(solPinVac, LOW);
  digitalWrite(pumpPinVac, LOW);
  
  unsigned long rangeTimer = millis();
  bool inRange = false;
  while(!inRange){
    measPressure = analogRead(pSensor);
      
    if(measPressure <= (desPressure + range) && measPressure >= (desPressure - range)){
      if(millis() - rangeTimer > 1000) inRange = true;
    }
    else{
      rangeTimer = millis();
    }

    myPID.Compute();
    moveStepper(outputPID);
  }
  
  Serial.println("I");
   
}

/* Move the pump stepper */
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

/* Serial */
// Check the incoming command
void checkSerial(){
  int command = Serial.read();
  
  if (command == 'R') {
    Ang = readValue();
    stepperGripper.rotate(Ang);
    #ifdef DEBUG_MODE
      Serial.print("Angle: ");
      Serial.println(Ang);
    #endif
  }
  else if (command == 'T') {
    desPressure = readValue();
    initializePump();
    #ifdef DEBUG_MODE
      Serial.print("Pressure: ");
      Serial.println(desPressure);
    #endif
  }
  else if (command == 'S') stepperGripper.rotate(-Ang);
  else if (command == 'U') desPressure = minPres;
}

// Read value of incoming number
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

// Send Serial Values
#ifdef DEBUG_MODE
  void printDebug() {
    Serial.print(measPressure);
    Serial.print(",  ");
    Serial.print(desPressure);
    Serial.print(",  ");
    Serial.print(digitalRead(endswitch1Pin));
    Serial.print(",  ");
    Serial.println(digitalRead(endswitch2Pin));
  }
#endif
