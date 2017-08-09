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
unsigned long   sTime;
char    state, oldState;

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
    BangBang();
    break;
  case stPID :
    break;
  default :
    break;
  }
}

// functions are alphabetical

void BangBang() {
  int inByte, inNum;
  bool controllerOn = false;
  int sp = 150;
  float target;
  float deadBand = 10;
  
  while ( stBang == state ) {
    inByte = Serial.read();
    switch( inByte ) {
    case -1 : // no character
      break;
    case 'q' :
      controllerOn = false;
      Quit();
      return;
      break;
    case 'h' :
    case '?' :
      HelpAll();
      Serial << "\r\nBang Bang parameters\r\n";
      Serial << "Target angle = " << target << " deadband = " << deadBand << " motor speed = " << sp << endl;
      break;
    case 'a' : // set desired angle
      target = Serial.parseFloat();
      target = constrain( target, -20, 140);
      Serial << "new target = " << target << endl;
      break;
    case 'd' : // set (half) deadband default 10
      deadBand = Serial.parseFloat();
      deadBand = constrain( deadBand, 0, 45);
      Serial << "new deadband = " << deadBand << endl;
      break;
    case 'g' : // go - start controller
      controllerOn = true;
      break;
    case 's' : // stop controller
      controllerOn = false;
      break;
    case 'v' : // set motor speed (default = 150)
      sp = Serial.parseInt();
      sp = constrain( sp, 0, 255);
      Serial << "new speed = " << sp << endl;
      break;
      break;
    default: // bad input, just eat the character
      break;
    } 

    if ( controllerOn ) {
      float errorAng = ReadAngle() - target;

      if ( Timer( 1000 ) ) {
        Serial << "Target angle = " << target << " deadband = " << deadBand << " motor speed = " << sp << endl;
        Serial <<  "Error = " << errorAng << endl << endl;
      }
      myMotor->setSpeed( sp );
      if( abs( errorAng ) <= deadBand ) {
        myMotor->run(RELEASE); // make sure motor is stopped (coasting);      
      }
      else if ( errorAng < 0 ) {
        myMotor->run(FORWARD);
      }
      else {
        myMotor->run(BACKWARD);    
      }
   }
    else {
      myMotor->run(RELEASE); // make sure motor is stopped (coasting);      
    }
  } // while in stBang 
}

void DisplayAngle() {
  int inByte;
  
  angleRead = ReadAngle();  
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);

  // print the results to the serial monitor:
  Serial << "pot = " << potCount << endl;
  Serial << "angle = " << angleRead << "\r\n";
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
    Serial << "Angle display mode\r\n";
    Serial << "q to return to Idle\r\n";
    break;
  case stManual :
    Serial << "Manual motor control mode \r\n";
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
  Serial << "Bang bang controller mode\r\n";
  Serial << "q - stop motor return to Idle state\r\n";
  Serial << "a<number> - set target angle\r\n";
  Serial << "d<number> - set (1/2) deadband\r\n";
  Serial << "g - start\r\n";
  Serial << "s - stop but stay in bang-bang mode\r\n";
  Serial << "v<number> - set motor speed, 0-255 default 150\r\n";  
}

void HelpIdle() {
  Serial << "Control Demo code\r\n";
  Serial << "idle mode\r\n";
  Serial << "Use lower case commands\r\n";
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
    Serial.read(); // remove character
    return;
    break;
  case 'h' :
  case '?' :
    HelpAll();
    Serial.read(); // remove character
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
      oldState = state;
      state = stBang;
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

// if at least t milliseconds has passed since last call, reset timer and return true
// allows non-blocking timed events
boolean Timer( unsigned long t ) {
  static unsigned long  last = 0;
  unsigned long ms = millis();
  
  if ( ms >= t + last ) {
    last = ms;
    return true;
  }
  else {
    return false;
  }
}
// Timer utility functions.  Concept from Matlab
void tic() {
  sTime = micros();
}

unsigned long toc() {
  return micros() - sTime;
}


