/* 
 * inverse.ino
 * 
 * This example stop and run a DC motor conected in pins 6 and 7 
 * with inverse polarity.
 * 
 * Created by Jonathan Ruiz de Garibay
 */

#include <DcMotor.h>

DcMotor myMotor;

void setup() {

  // configute motor pins and inverse polarity
	myMotor.begin(6, 7, INVERSE);
}

void loop() {

  myMotor.goForward();    // move motor in forward direction
  delay(1000);
  myMotor.stop();         // stop motor
  delay(1000);
}
