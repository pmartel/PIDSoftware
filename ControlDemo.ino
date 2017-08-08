/*
 * This is code for a PID demonstration based on Tim Wescott's 
 * YouTube video  https://www.youtube.com/watch?v=2elEXcv0AV8&t=128s
 * 
 */
// Streaming supports Serial << "text"; (like C++)
#include <Streaming.h>
#include <Adafruit_MotorShield.h>

// #include "ControlDemo.h"
//Apparently Arduino doesn't like to let you write include files pr handle separate modules

// constants 
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const char ESC = 0x1B;

// states
const char stIdle = 'i';
const char stDisplay = 'a'; // angle display
const char stManual = 'm';
const char stBang = 'b';
const char stPID = 'p';

// globals
int     potCount = 0;        // value read from the pot
float   angleRead;
unsigned long   sTime, eTime;
char state, oldState;

// motor related
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M4
Adafruit_DCMotor *myMotor = AFMS.getMotor(4);

void setup() {
  // Set up serial port
  Serial.begin(115200);
  HelpIdle();
  state = oldState = stIdle;
  AFMS.begin();  // create with the default frequency 1.6KHz
  myMotor->run(RELEASE); // make sure motor is stopped

}

void loop() {
  if ( oldState != state ) {
    Serial << endl;
    Serial << "changed state from " << oldState << " to " << state << endl;
    oldState = state;
  }
  switch( state ) {
  case stIdle :
    IdleLoop();
    break;
  case stDisplay :
    DisplayAngle();
    break;
  case stManual :
    ManualMotor();
    break;
  case stBang :
    break;
  case stPID :
    break;
  default :
    break;
  }
}

// functions are alphabetical

void BangBang() {
  
}

void DisplayAngle() {
  int inByte;
  
  sTime = micros();
  angleRead = ReadAngle();  
  eTime = micros();
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);

  // print the results to the serial monitor:
  Serial << "pot = " << potCount << endl;
  Serial << "angle = " << angleRead << "\r\n";
  Serial << "angle calc took " << eTime - sTime << " usec\r\n\n";
  delay(1000); 
  inByte = Serial.read();
  if ( 'q' == inByte ) Quit();
  if ( '?' == inByte || 'h' == inByte ) HelpAll();
}

void IdleLoop() {
  ProcessInput();
  delay(500);
}

void HelpAll() {
  switch( state ) {
  case stIdle :
    HelpIdle();
    break;
  case stDisplay :
    Serial << "q to return to Idle\r\n";
    break;
  case stManual :
    Serial << "Enter a number -255 to 255 to set motor speed\r\n";
    Serial << "q to stop motor return to Idle state\r\n";
    break;
  case stBang :
    HelpBang();
    break;
  case stPID :
     Serial << "q to stop motor return to Idle state\r\n";
   break;
  default :
    Serial << "Unknown state 0x" << _HEX(state) << endl;
    Serial << "returning to Idle state\r\n";
    Quit();
    break;
  }
  
}

void HelpBang() {
  Serial << "q - stop motor return to Idle state\r\n";
  Serial << "a<number> - set target angle\r\n";
  Serial << "d<number> - set (1/2) deadband\r\n";
  Serial << "g - start\r\n";
  Serial << "s - stop but stay in bang-bang mode\r\n";
  Serial << "v<number> - set motor speed, 0-255 default 150\r\n";  
}

void HelpIdle() {
  Serial << "Control Demo code\r\n";
  Serial << "Commands are not case sensitive\r\n";
  Serial << "depending on your terminal you may need to hit enter\r\n";
  Serial << "q will stop the motor and return to the main loop\r\n";
  Serial << "h or ? - print this help\r\n";
  Serial << "a - display angle\r\n";
  Serial << "m - manual motor control\r\n";
  Serial << "b - bang-bang controller\r\n";
  Serial << "p - PID controller\r\n";

}

void ManualMotor() {
  int inByte, inNum;
  static int sp = 0, oldSp = 0;

  inByte = Serial.peek();
  switch( inByte ) {
  case 'q' :
    Quit();
    Serial.read(); // get rid of character
    return;
    break;
  case 'h' :
  case '?' :
    HelpAll();
    Serial.read(); // get rid of character
    return;
    break;
  case -1 : // no input
    Serial << "angle " << ReadAngle() << endl;
    delay(500);
    return;
    break;
  default: // assume input is a number
    inNum = Serial.parseInt();
    break;
  }
  Serial << "inByte = " << inByte << endl;
  Serial << "inNum = " << inNum << endl;
  oldSp = sp;
  //limit the speed
  sp = constrain( inNum, -255, 255 );
  Serial   << "Changing speed from " << oldSp << " to " << sp << endl;
  myMotor->setSpeed( abs(sp) );
  if ( sp >= 0 ) {
      myMotor->run(FORWARD);
  }
  else {
      myMotor->run(BACKWARD);    
  }
}

void ProcessInput() {
  int byteIn;
  
  if (Serial.available() > 0 ) {
    byteIn = Serial.read();
    switch( byteIn ) {
    case 'q' :
      Quit();
      break;
    case 'h' :
    case '?' :
      HelpAll();
      break;
    case 'a' :
      myMotor->run(RELEASE); // make sure motor is stopped (coasting) just in case
      oldState = state;
      state = stDisplay;
      break;
    case 'm' :
      oldState = state;
      state = stManual;
      break;
    case 'b' :
      break;
    case 'p' :
      break;
    default :  
      break;
    }
  }
}

void Quit() {
  myMotor->run(RELEASE); // make sure motor is stopped (coasting)
  oldState = state;
  state = stIdle;
}

// read the angle and return it.  
// potCount and angle read are globals,for convenience in DisplayAngle
// returns value for use in other pieces of code
float ReadAngle() {
    // read the analog in value:
  potCount = analogRead(analogInPin);
  // emperically, angleRead = 143.996650585439 + (-0.24148432002069) * potCount
  angleRead = 143.996650585439 + (-0.241484320020696) * potCount;

  return angleRead;

}

