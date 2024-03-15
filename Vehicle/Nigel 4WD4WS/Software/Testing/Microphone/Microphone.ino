const int buzzer = 46; // Buzzer pin
const int micDigitalPin = A0; // Microphone digital pin
const int micAnalogPin = A1; // Microphone analog pin

int micDigitalValue = 0; // Microphone digital reading
int micAnalogValue = 0; // Microphone analog reading

void setup(){
  pinMode(buzzer, OUTPUT); // Set buzzer pin as an output
  pinMode(micDigitalPin, INPUT); // Set microphone pin as input
  pinMode(micAnalogPin, INPUT); // Set microphone pin as input
  Serial.begin(9600); // Initialize the serial port
  delay(5000); // Initialization delay
}

void loop(){
  for(int i=0; i<=25; i++)
  {
    tone(buzzer, 1000); // Send 1 KHz sound signal
    micDigitalValue = digitalRead(micDigitalPin);
    micAnalogValue = analogRead(micAnalogPin);
    //Serial.print("Digital-Value:"); Serial.print(micDigitalValue); Serial.print(",");
    Serial.print("Analog-Value:"); Serial.println(micAnalogValue);
  }

  for(int i=0; i<=25; i++)
  {
    noTone(buzzer);     // Stop sound
    micDigitalValue = digitalRead(micDigitalPin);
    micAnalogValue = analogRead(micAnalogPin);
    //Serial.print("Digital-Value:"); Serial.print(micDigitalValue); Serial.print(",");
    Serial.print("Analog-Value:"); Serial.println(micAnalogValue);
  }
}
