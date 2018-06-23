//////////////////
// Declarations //
//////////////////

/* CODE SETTINGS */
// Uncomment to enable:
//#define DEBUG_MODE

/* LIBRARIES */
#include <Arduino.h>
#include <AccelStepper.h>
#include "DRV8825.h"
#include <PID_v1.h>

/* PINS */
const int pinVacSolenoid  = 4;
const int pinPresPump     = 6;
const int pinVacPump      = 10;

const int pinPresSensor     = A0;
const int pinEndSwitchHigh  = 12; // Switch at the high pressure end (syringe empty)
const int pinEndSwitchLow   = 13; // Switch at the low pressure end (syringe full)

#define PUMP_DIR   2
#define PUMP_STEP  3

#define OBST_DIR   8
#define OBST_STEP  9
#define OBST_MS1   10
#define OBST_MS2   11
#define OBST_MS3   7

/* STEPPER MOTORS */
#define OBST_MOTOR_STEPS    200 // Using a 200-step motor (most common)
#define OBST_MOTOR_SPEED    15  // 15 RPM
#define OBST_MOTOR_STEPSIZE 1   // 1 step

AccelStepper pumpStepper(AccelStepper::DRIVER, PUMP_STEP, PUMP_DIR);
A4988 obstacleStepper(OBST_MOTOR_STEPS, OBST_DIR, OBST_STEP, OBST_MS1, OBST_MS2, OBST_MS3);

double obstacleAngle = 0.0;

/* PRESSURE SENSOR */
double pressureNeutral =  49.0; // Neutral pressure
double pressureDesired =  0.0;  // Desired pressure = SetpointPID
double pressureMeasured = 0.0;  // Measured pressure

double pressureRange =    4.0;  // Acceptable range for stable pressure
const int durationRange = 1000; // How long the vacuum pump will be on during initialization


/* PID CONTROL */
// Tuning parameters
double Kp = 400;
double Ki = 10;
double Kd = 6;
double outputPID;

PID myPID(&pressureMeasured, &outputPID, &pressureDesired, Kp, Ki, Kd, DIRECT);

/* PUMPS */
const int durationVacInit = 4500; // How long the vacuum pump will be on during initialization
const int durationPres =    3500; // How long the pressure pump is turned on in milisec
const int durationVac =     2000; // How long the vacuum pump is turned on in milisec
const int presPumpPower =   100;  // PWM Power of pressure pump
const int vacPumpPower =    100;  // PWM Power of pressure pump

unsigned long timerPresOn = 0;  // The moment the pump has been turned on (does not have to be set)
unsigned long timervacOn =  0; // The moment the vacuum pump has been turned on (does not have to be set)

bool presOn =       false;  // If the pressure pump is turned on
bool presDoubleOn = false;  // If the pressure pump is turned on at double power
bool vacOn =        false;  // If the vacuum pump is turned on
bool vacDoubleOn =  false;  // If the vacuum pump is turned on at double power
bool leaking =      false;  // If the below has been determined to be leaking


// Debug print time
#ifdef DEBUG_MODE
  double timerPrintPrevious = 0;
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
  pressureDesired = pressureNeutral;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100000, 100000);

  /* SETUP STEPPERS */
  pumpStepper.setAcceleration(20000000);
  obstacleStepper.begin(OBST_MOTOR_SPEED, OBST_MOTOR_STEPSIZE);
  obstacleStepper.rotate(30);
  obstacleStepper.rotate(-30);
  
}


///////////////
// Main Loop //
///////////////
void loop() {
  
  if(!Serial.available()){
    if(!leaking){
      pressureMeasured = analogRead(pinPresSensor);
      myPID.Compute();
      moveStepper(outputPID);

      controlPumpLimits();
    }
    else delay(1000);
    
    #ifdef DEBUG_MODE
      if (millis() - timerPrintPrevious > 200) {
        printDebug();
        timerPrintPrevious = millis();
      }
    #endif
    
  }
  else  checkSerial();
  
}


///////////////
// FUNCTIONS //
///////////////
/* LIMIT PUMPS CONTROL */
void controlPumpLimits(){
  
  // Turn pressure pump on if high Pressure end of syringe has been reached
  if (digitalRead(pinEndSwitchHigh) == LOW && !presOn && !presDoubleOn) {
    analogWrite(pinPresPump, presPumpPower);
    timerPresOn =  millis();
    presOn = true;
  }
  // Increase pump power if current power isn't enough
  if (millis() - timerPresOn > durationPres && digitalRead(pinEndSwitchHigh) == LOW  && presOn){
    analogWrite(pinPresPump, 200);
    timerPresOn =  millis();
    presOn = false;
    presDoubleOn = true;
  }
  // Turn of pumps if even double power isn't enough
  if (millis() - timerPresOn > durationPres && digitalRead(pinEndSwitchHigh) == LOW && presDoubleOn){
    moveStepper(0);
    digitalWrite(pinPresPump, LOW);
    leaking = true;
    presDoubleOn = false;
    Serial.print("L");
    return;
  }
  // Turn pressure pump off after timer has passed or other side has been reached
  if ((millis() - timerPresOn > durationPres || digitalRead(pinEndSwitchLow) == LOW) && (presOn || presDoubleOn)) {
    digitalWrite(pinPresPump, LOW);
    presOn = false;
    presDoubleOn = false;
  }

  // Turn vacuum pump on if low Pressure end of syringe has been reached
  if (digitalRead(pinEndSwitchLow) == LOW && !vacOn && !vacDoubleOn) {
    analogWrite(pinVacPump, vacPumpPower);
    digitalWrite(pinVacSolenoid, HIGH);
    timervacOn =  millis();
    vacOn = true;
  }
  // Increase vacuum pump power if current power isn't enough
  if (millis() - timervacOn > durationVac && digitalRead(pinEndSwitchLow) == LOW  && vacOn){
    analogWrite(pinVacPump, 200);
    timervacOn =  millis();
    vacOn = false;
    vacDoubleOn = true;
  }
  // Turn vacuum pump off after timer has passed or other side has been reached
  if ((millis() - timervacOn > durationVac || digitalRead(pinEndSwitchHigh) == LOW) && (vacOn || vacDoubleOn)) {
    digitalWrite(pinVacSolenoid, LOW);
    digitalWrite(pinVacPump, LOW);
    vacOn = false;
    vacDoubleOn = false;
  }
  
}

/* INITIALIZE SETUP AFTER INCOMING COMMAND */
void initializePump(){

  // Check if limit pumps were on
  if(presOn) presOn = false;
  if(vacOn){
    digitalWrite(pinVacSolenoid, LOW);
    digitalWrite(pinVacPump, LOW);
    vacOn = false;
  }

  // Fill pump with air untill switch is reached
  analogWrite(pinPresPump, presPumpPower);
  unsigned long timerBeforePressure = millis();
  while (digitalRead(pinEndSwitchLow) == HIGH){    
    pressureMeasured = analogRead(pinPresSensor);
    myPID.Compute();
    moveStepper(outputPID);
    
    if(millis() - timerBeforePressure > 6000 && digitalRead(pinEndSwitchHigh) == LOW){
      moveStepper(0);
      digitalWrite(pinPresPump, LOW);
      leaking = true;
      Serial.print("L");
      return;
    }
    if(millis() - timerBeforePressure > 3000 && digitalRead(pinEndSwitchHigh) == LOW){
      analogWrite(pinPresPump, 200);
    }
    
  }
  digitalWrite(pinPresPump, LOW);

  // Empty pump for a couple of seconds
  analogWrite(pinVacPump, vacPumpPower);
  digitalWrite(pinVacSolenoid, HIGH);
  unsigned long timerBeforeVacuum = millis();
  while (millis() - timerBeforeVacuum < durationVacInit){    
    pressureMeasured = analogRead(pinPresSensor);
    myPID.Compute();
    moveStepper(outputPID);
  }
  digitalWrite(pinVacSolenoid, LOW);
  digitalWrite(pinVacPump, LOW);

  // Wait till the current pressure is within desired range for at least desired duration
  unsigned long timerPressureRange = millis();
  bool inRange = false;
  while(!inRange){
    pressureMeasured = analogRead(pinPresSensor);
      
    if(pressureMeasured <= (pressureDesired + pressureRange) && pressureMeasured >= (pressureDesired - pressureRange)){
      if(millis() - timerPressureRange > 1000){
        inRange = true;
      }
    }
    else timerPressureRange = millis();

    myPID.Compute();
    moveStepper(outputPID);
  }

  // Send command after pressure has set within range
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
  if(!leaking){
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
      pressureDesired = pressureNeutral;
    }
  }
  else{
    if(command == 'F'){
      leaking = false;
    }
    else Serial.print("L");
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

    if(leaking) Serial.print("Leaking");
    else{
      Serial.print(pressureMeasured);
      Serial.print(",  ");
      Serial.print(pressureDesired);
      Serial.print(",  ");
      Serial.print(digitalRead(pinEndSwitchHigh));
      Serial.print(",  ");
      Serial.println(digitalRead(pinEndSwitchLow));
    }
    
  }
#endif
