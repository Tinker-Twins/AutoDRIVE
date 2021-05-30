const byte speedPin = 6;
const byte dirxnPin = 7;
const byte brakePin = 8;

boolean dirxnValue = 0;
int speedValue = 0;
boolean brakeValue = 0;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  
  pinMode(speedPin, OUTPUT); // Declare speed pin
  pinMode(brakePin, OUTPUT); // Declare brake pin
  pinMode(dirxnPin, OUTPUT); // Declare direction pin
}

void loop()
{
  Serial.println("Drive Forward");
  dirxnValue = 0;
  speedValue = 200;
  brakeValue = 0;
  digitalWrite(dirxnPin, dirxnValue);
  digitalWrite(speedPin, 1);
  digitalWrite(brakePin, brakeValue);

  delay(5000);

  Serial.println("Brake");
  speedValue = 0;
  brakeValue = 1;
  digitalWrite(speedPin, 0);
  digitalWrite(brakePin, brakeValue);

  delay(1000);
  
  Serial.println("Drive Reverse");
  dirxnValue = 1;
  speedValue = 200;
  brakeValue = 0;
  digitalWrite(dirxnPin, dirxnValue);
  digitalWrite(speedPin, 1);
  digitalWrite(brakePin, brakeValue);

  delay(5000);

  Serial.println("Brake");
  speedValue = 0;
  brakeValue = 1;
  digitalWrite(speedPin, 0);
  digitalWrite(brakePin, brakeValue);
 
  delay(1000);
}
