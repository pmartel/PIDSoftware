/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

int potValue = 0;        // value read from the pot

void setup() {
  // Set up serial port
  Serial.begin(115200);
}

void loop() {
  // read the analog in value:
  potValue = analogRead(analogInPin);
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);

  // print the results to the serial monitor:
  Serial.print("pot = ");
  Serial.println(potValue);
  delay(1);  

}
