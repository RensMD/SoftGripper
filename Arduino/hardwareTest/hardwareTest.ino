const int endswitchPin = 12; // Switch at the low pressure end (srynge full)
const int endswitch2Pin = 13; // Switch at the low pressure end (srynge full)
const int solPinVac   = 4;
const int pumpPin     = 6;
const int pumpPinVac  = 10;

void setup() {
  pinMode(endswitchPin, INPUT);
  pinMode(endswitch2Pin, INPUT);
  pinMode(pumpPin, OUTPUT);
  pinMode(pumpPinVac, OUTPUT);
  pinMode(solPinVac, OUTPUT);
  
  Serial.begin(9600);
}

void loop() {
  Serial.print(digitalRead(endswitchPin));
  Serial.print(" , ");
  Serial.println(digitalRead(endswitch2Pin));

  analogWrite(pumpPinVac, 100);
  delay(2000);   
  digitalWrite(pumpPinVac, LOW);
  analogWrite(pumpPin, 100);
  delay(2000);
  digitalWrite(pumpPin, LOW);

  digitalWrite(solPinVac, HIGH);
  delay(2000);
  digitalWrite(solPinVac, LOW);
  delay(1000);
}
