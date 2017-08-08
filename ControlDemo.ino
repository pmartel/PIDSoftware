/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */
#include <Streaming.h>

const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const char ESC = 0x1B;

int potCount = 0;        // value read from the pot
float angleRead;

void setup() {
  // Set up serial port
  Serial.begin(115200);
  Serial << "Control Demo code\r\n";
}

void loop() {
  // read the analog in value:
  potCount = analogRead(analogInPin);

  
  // emperically, angleRead = 143.996650585439 + (-0.24148432002069) * potCount
  angleRead = 143.996650585439 + (-0.241484320020696) * potCount;
  
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);

  // print the results to the serial monitor:
//  Serial.print("pot = ");
//  Serial.println(potCount);
//  Serial.print("angle = " );
//  Serial.println(angleRead);
//  Serial.println();
Serial << "pot = " << potCount << endl;
Serial << "angle = " << angleRead << "\r\n\n";
  delay(1000);  

}
