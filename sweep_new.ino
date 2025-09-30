/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 https://www.arduino.cc/en/Tutorial/LibraryExamples/Sweep
*/

#include <Servo.h>

// Servo servoBR;  // create Servo object to control a servo
Servo servoTR;
// Servo servoBL;
// Servo servoTL;
// twelve Servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  servoTR.attach(9);  // attaches the servo on pin 9 to the Servo object
  delay(1000);
  servoTR.write(0);
  // servoBR.attach(10);
  // servoTL.attach(12);
  // servoBL.attach(13);
  // initAngle();
}

void initAngle() {
  servoTR.write(0);
  // servoTR.write(37);
  // servoTL.write(37);
  // servoBL.write(37);
}

void loop() {

  servoTR.write(45);
  delay(2000);
  servoTR.write(0);
  delay(2000);

  /*
  for (pos = 0; pos <= 35; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    servoTR.write(pos);              // tell servo to go to position in variable 'pos'
    servoTL.write(-pos);
    servoBR.write(-pos);
    servoBL.write(pos);
    delay(500);                       // waits 15 ms for the servo to reach the position
  }
  for (pos = 180; pos >= 35; pos -= 1) { // goes from 180 degrees to 0 degrees
    servoTR.write(pos);              // tell servo to go to position in variable 'pos'
    servoTL.write(-pos);
    servoBR.write(-pos);
    servoBL.write(pos);    
    delay(500);                       // waits 15 ms for the servo to reach the position
  }
  */

  // runLandingSequence();
// }

// void runLandingSequence() {
//   for (pos = 0; pos <= 35; pos += 1) { // goes from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     servoTR.write(37+pos);              // tell servo to go to position in variable 'pos'
//     servoTL.write(37-pos);
//     servoBR.write(37-pos);
//     servoBL.write(37+pos);
//     delay(500);                       // waits 15 ms for the servo to reach the position
//   }
//   for (pos = 35; pos >= 0; pos -= 1) { // goes from 0 degrees to 180 degrees
//     // in steps of 1 degree
//     servoTR.write(37+35-pos);              // tell servo to go to position in variable 'pos'
//     servoTL.write((37+35)-pos);
//     servoBR.write((37+35)-pos);
//     servoBL.write(37+35-pos);
//     delay(500);                       // waits 15 ms for the servo to reach the position
//   }
//   for (pos = 0; pos >= -35; pos -= 1) { // goes from 180 degrees to 0 degrees
//     servoTR.write(pos);              // tell servo to go to position in variable 'pos'
//     servoTL.write(-pos);
//     servoBR.write(-pos);
//     servoBL.write(pos);    
//     delay(500);                       // waits 15 ms for the servo to reach the position
//   }
//   for (pos = -35; pos <= 0; pos += 1) { // goes from 180 degrees to 0 degrees
//     servoTR.write(pos);              // tell servo to go to position in variable 'pos'
//     servoTL.write(-pos);
//     servoBR.write(-pos);
//     servoBL.write(pos);    
//     delay(500);                       // waits 15 ms for the servo to reach the position
//   }
}