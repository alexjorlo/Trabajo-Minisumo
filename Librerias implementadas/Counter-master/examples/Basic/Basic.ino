/* Counter Library - Basic Example
 * https://github.com/joruiz/Counter
 *
 * This example code is in the public domain.
 */
 
#include <Counter.h>

Counter myCounter(8);

void setup() {

  Serial.begin(9600);
  Serial.println("Basic Counter Test:");
}

long oldValue  = -999;

void loop() {

  long newValue = myCounter.read();
  if (newValue != oldValue) {
    oldValue = newValue;
    Serial.println(newValue);
  }
}
