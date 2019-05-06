/* 
 * variablespeed.ino
 * 
 * This example move a DC motor connected in pins 6 and 7
 * with a variable speed.
 * 
 * Created by Jonathan Ruiz de Garibay
 */

#include <DcMotor.h>

DcMotor myMotor;

void setup() {

  // configute motor pins
	myMotor.begin(6, 7);
}

void loop() {

  // increment speed from 0 to 255
  for (int i = 0; i <= 255; i++) {
    myMotor.move(FORWARD, i);
    delay(10);
  }
  delay(1000);
  // decrement speed from 255 to 0
  for (int i = 255; i >= 0; i--) {
    myMotor.move(FORWARD, i);
    delay(10);
  }
  delay(1000);
}
