#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "Arduino.h"

#define FORWARD		0
#define BACKWARD	1

#define DIRECT		0
#define INVERSE		1

class DcMotor
{
	public:
		void begin(uint8_t directionPin, uint8_t speedPin);
		void begin(uint8_t directionPin, uint8_t speedPin, uint8_t polarity);
		void stop();
		void goForward();
		void goBackward();
		void move(uint8_t direction, uint8_t speed);		
		void move(int16_t speed);
	
	private:
		uint8_t _directionPin;
		uint8_t _speedPin;
		uint8_t _polarity;
};

#endif
