int tailLights = 10;
int headLightsLB = 11;
int headLightsHB = 12;
int reverseIndicators = 13;
int leftIndicators = 14;
int rightIndicators = 15;

void setup()
{
  Serial.begin(115200); // Initialize the serial port
  lightsInit(); // Initialize the vehicle lights
}

void loop()
{
  Serial.println("Headlights (Low Beam)");
  digitalWrite(headLightsLB, 1);
  analogWrite(tailLights, 15);
  delay(5000);
  digitalWrite(headLightsLB, 0);
  analogWrite(tailLights, 0);

  Serial.println("Headlights (High Beam)");
  digitalWrite(headLightsLB, 1);
  digitalWrite(headLightsHB, 1);
  analogWrite(tailLights, 15);
  delay(5000);
  digitalWrite(headLightsLB, 0);
  digitalWrite(headLightsHB, 0);
  analogWrite(tailLights, 0);

  Serial.println("Brake Indicators");
  analogWrite(tailLights, 255);
  delay(5000);
  analogWrite(tailLights, 0);

  Serial.println("Reverse Indicators");
  digitalWrite(reverseIndicators, 1);
  delay(5000);
  digitalWrite(reverseIndicators, 0);

  // Left Indicators
  Serial.println("Left Turn Indicators");
  for(int i=4; i>=0; i--)
  {
    digitalWrite(leftIndicators, 1);
    delay(500);
    digitalWrite(leftIndicators, 0);
    delay(500);
  }

  Serial.println("Right Turn Indicators");
  for(int j=4; j>=0; j--)
  {
    digitalWrite(rightIndicators, 1);
    delay(500);
    digitalWrite(rightIndicators, 0);
    delay(500);
  }

  Serial.println("Hazard Indicators");
  for(int k=4; k>=0; k--)
  {
    digitalWrite(leftIndicators, 1);
    digitalWrite(rightIndicators, 1);
    delay(500);
    digitalWrite(leftIndicators, 0);
    digitalWrite(rightIndicators, 0);
    delay(500);
  }
}

void lightsInit()
{
  pinMode(tailLights, OUTPUT);
  pinMode(headLightsLB, OUTPUT);
  pinMode(headLightsHB, OUTPUT);
  pinMode(reverseIndicators, OUTPUT);
  pinMode(leftIndicators, OUTPUT);
  pinMode(rightIndicators, OUTPUT);
}
