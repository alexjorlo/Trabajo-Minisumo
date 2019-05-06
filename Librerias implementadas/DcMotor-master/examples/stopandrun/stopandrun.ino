/* 
 * stopandrun.ino
 * 
 * This example stop and run a DC motor connected in pins 6 and 7.
 * 
 * Created by Jonathan Ruiz de Garibay
 */

#include <DcMotor.h>

DcMotor myMotor;

void setup() {

  // configure motor pins
	myMotor.begin(6, 7);
}

void loop() {

  myMotor.goForward();    // move motor in forward direction
  delay(1000);
  myMotor.stop();         // stop motor
  delay(1000);
}
