/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int potCount = 0;        // value read from the pot
float angleRead;

void setup() {
  // Set up serial port
  Serial.begin(115200);
}

void loop() {
  // read the analog in value:
  potCount = analogRead(analogInPin);

  
  // emperically, angleRead = 169.285780959057 + (-0.225789125999221) * potCount
  angleRead = 169.285780959057 + (-0.225789125999221) * potCount;
  
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);

  // print the results to the serial monitor:
  Serial.print("pot = ");
  Serial.println(potCount);
  Serial.print("angle = " );
  Serial.println(angleRead);
  Serial.println();
  delay(1000);  

}
