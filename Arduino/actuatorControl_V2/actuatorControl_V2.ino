// Uncomment to enable:
//#define DEBUG_MODE

/* LIBRARIES */
#include <Arduino.h>
#include "DRV8825.h"
#include <PID_v1.h>
#include <AccelStepper.h>

/* PINS */
const int pinVacSolenoid  = 4;
const int pinPresPump     = 6;
const int pinVacPump      = 10;

const int pinEndSwitchHigh  = 12; // Switch at the high pressure end (srynge empty)
const int pinEndSwitchLow   = 13; // Switch at the low pressure end (srynge full)
const int pinPresSensor     = A0; // Pressure sensor

#define OBST_DIR   8
#define OBST_STEP  9
#define OBST_MS1   10
#define OBST_MS2   11
#define OBST_MS3   7

#define PUMP_STEP  3
#define PUMP_DIR   2

/* STEPPER MOTORS */
#define OBST_MOTOR_STEPS 200 // Using a 200-step motor (most common)

A4988 obstacleStepper(OBST_MOTOR_STEPS, OBST_DIR, OBST_STEP, OBST_MS1, OBST_MS2, OBST_MS3);
AccelStepper pumpStepper(AccelStepper::DRIVER, PUMP_STEP, PUMP_DIR);

double obstacleAngle = 0.0;
double pressureMin = 49.0;
double pressureDesired = 0.0;  // Desired pressure = SetpointPID
double pressureMeasured = 0.0;
double pressureRange = 4.0;

/* PID CONTROL */
// Tuning parameters
double Kp = 400;
double Ki = 10;
double Kd = 6;

double outputPID;
PID myPID(&pressureMeasured, &outputPID, &pressureDesired, Kp, Ki, Kd, DIRECT);

/* PUMPS */
int durationPres = 4000; // How long the pressure pump is turned on in milisec
int durationVac = 2000; // How long the vacuum pump is turned on in milisec

long timePresOn = 0; // The moment the pump has been turned on (does not have to be set)
long timeVacOn = 0; // The moment the vacuum pump has been turned on (does not have to be set)
boolean presOn = false; // If the pump is turned on
boolean VacOn = false; // If the vacuum pump is turned on

//for debug print
#ifdef DEBUG_MODE
  double timePrintPrevious = 0;
#endif

////////////
// Set Up //
////////////
void setup() {
  /* SETUP PINS */
  pinMode(pinPresSensor, INPUT);
  pinMode(pinEndSwitchHigh, INPUT);
  pinMode(pinEndSwitchLow, INPUT);
  pinMode(pinPresPump, OUTPUT);
  pinMode(pinVacPump, OUTPUT);
  pinMode(pinVacSolenoid, OUTPUT);

  digitalWrite(pinVacSolenoid, LOW);

  /* SETUP COMMUNICATION */
  Serial.begin(9600);

  /* SETUP PID */
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100000, 100000);
  pressureDesired = pressureMin;

  /* SETUP STEPPERS */
  pumpStepper.setAcceleration(20000000);
  obstacleStepper.begin(15, 1);
  obstacleStepper.rotate(30);
  obstacleStepper.rotate(-30);
}

///////////////
// Main Loop //
///////////////
void loop() {
  if(!Serial.available()){
    pressureMeasured = analogRead(pinPresSensor);

    // PID
    myPID.Compute();
    moveStepper(outputPID);

    controlPump();

    #ifdef DEBUG_MODE
      if (millis() - timePrintPrevious > 200) {
        printDebug();
        timePrintPrevious = millis();
      }
    #endif
  }
  else  checkSerial();
}

///////////////
// FUNCTIONS //
///////////////
/* Control the pumps */
void controlPump(){
  // TURN PUMP ON IF SWITCH AT HIGH PRESSURE END IS TOUCHED
  if (digitalRead(pinEndSwitchHigh) == LOW) {
    analogWrite(pinPresPump, 100);
    timePresOn =  millis();
    presOn = true;
  }
  // TURN PUMP OFF AFTER THE TIMEPOMPON HAS PASSED
  if ((millis() - timePresOn > durationPres || digitalRead(pinEndSwitchLow) == LOW) && presOn) {
    digitalWrite(pinPresPump, LOW);
    presOn = false;
  }

  //TURN VACUUM PUMP ON IF SWITCH AT LOW PRESSURE END IS TOUCHED (AND OPEN SOLENOID)
  if (digitalRead(pinEndSwitchLow) == LOW) {
    analogWrite(pinVacPump, 100);
    digitalWrite(pinVacSolenoid, HIGH);
    timeVacOn =  millis();
    VacOn = true;
  }
  //TURN VACUUM PUMP OFF AFTER THE TIMEPOMPON HAS PASSED
  if ((millis() - timeVacOn > durationVac || digitalRead(pinEndSwitchHigh) == LOW) && VacOn) {
    digitalWrite(pinVacSolenoid, LOW);
    digitalWrite(pinVacPump, LOW);
    VacOn = false;
  }
}

/* INITIALIZE PUMP AFTER COMMAND */
void initializePump(){
  if(presOn)
  if(VacOn)
  
  analogWrite(pinPresPump, 150);
  while (digitalRead(pinEndSwitchLow) == HIGH){    
    pressureMeasured = analogRead(pinPresSensor);
    myPID.Compute();
    moveStepper(outputPID);
  }
  digitalWrite(pinPresPump, LOW);

  analogWrite(pinVacPump, 100);
  digitalWrite(pinVacSolenoid, HIGH);
  unsigned long timeBefore = millis();
  while (millis() - timeBefore < 5000){    
    pressureMeasured = analogRead(pinPresSensor);
    myPID.Compute();
    moveStepper(outputPID);
  }
  digitalWrite(pinVacSolenoid, LOW);
  digitalWrite(pinVacPump, LOW);
  
  unsigned long pressureRangeTimer = millis();
  bool inRange = false;
  while(!inRange){
    pressureMeasured = analogRead(pinPresSensor);
      
    if(pressureMeasured <= (pressureDesired + pressureRange) && pressureMeasured >= (pressureDesired - pressureRange)){
      if(millis() - pressureRangeTimer > 1000) inRange = true;
    }
    else{
      pressureRangeTimer = millis();
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
  
  pumpStepper.move(moveDir);
  pumpStepper.setMaxSpeed(abs(stepperMove));
  pumpStepper.runSpeed();
}

/* Serial */
// Check the incoming command
void checkSerial(){
  int command = Serial.read();
  
  if (command == 'A') {
    // Set Angle
    obstacleAngle = readValue();
    obstacleStepper.rotate(obstacleAngle);
    #ifdef DEBUG_MODE
      Serial.print("Angle: ");
      Serial.println(obstacleAngle);
    #endif
  }
  else if (command == 'P') {
    // Set Pressure
    pressureDesired = readValue();
    initializePump();
    #ifdef DEBUG_MODE
      Serial.print("Pressure: ");
      Serial.println(pressureDesired);
    #endif
  }
  else if (command == 'R'){
    // Reset positions 
    obstacleStepper.rotate(-obstacleAngle);
    pressureDesired = pressureMin;
  }
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
    Serial.print(pressureMeasured);
    Serial.print(",  ");
    Serial.print(pressureDesired);
    Serial.print(",  ");
    Serial.print(digitalRead(pinEndSwitchHigh));
    Serial.print(",  ");
    Serial.println(digitalRead(pinEndSwitchLow));
  }
#endif
