const int buzzer = 46; // Buzzer pin

void setup(){
  pinMode(buzzer, OUTPUT); // Set buzzer pin as an output
}

void loop(){
  tone(buzzer, 1000); // Send 1 KHz sound signal
  delay(1000);        // Wait for 1 sec
  noTone(buzzer);     // Stop sound
  delay(1000);        // Wait for 1 sec
}