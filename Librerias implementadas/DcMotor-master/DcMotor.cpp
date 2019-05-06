#include "DcMotor.h"

void DcMotor::begin(uint8_t directionPin, uint8_t speedPin){

	_directionPin = directionPin;
	_speedPin = speedPin;
	
	pinMode(_directionPin, OUTPUT);
	pinMode(_speedPin, OUTPUT);
	
	_polarity = DIRECT;
	stop();
}

void DcMotor::begin(uint8_t directionPin, uint8_t speedPin, uint8_t polarity){

	_directionPin = directionPin;
	_speedPin = speedPin;
	
	pinMode(_directionPin, OUTPUT);
	pinMode(_speedPin, OUTPUT);
	
	_polarity = polarity;
	stop();
}

void DcMotor::stop(){
	
	digitalWrite(_directionPin, LOW);
	digitalWrite(_speedPin, LOW);
}

void DcMotor::goForward(){
	
	if (_polarity == DIRECT){
		digitalWrite(_directionPin, LOW);
		digitalWrite(_speedPin, HIGH);
	}
	else{
		digitalWrite(_directionPin, HIGH);
		digitalWrite(_speedPin, LOW);
	}
}

void DcMotor::goBackward(){

	if (_polarity == DIRECT){
		digitalWrite(_directionPin, HIGH);
		digitalWrite(_speedPin, LOW);
	}
	else{
		digitalWrite(_directionPin, LOW);
		digitalWrite(_speedPin, HIGH);
	}
}

void DcMotor::move(uint8_t direction, uint8_t speed){
	
	if (((direction == FORWARD) && (_polarity == DIRECT)) || ((direction == BACKWARD) && (_polarity == INVERSE))){
		digitalWrite(_directionPin, LOW);
		analogWrite(_speedPin, speed);
	}
	else{
		digitalWrite(_directionPin, HIGH);
		analogWrite(_speedPin, 255 - speed);
	}		
}

void DcMotor::move(int16_t speed){
	
	if (speed == 0){
		stop();
	} else if (((speed > 0) && (_polarity == DIRECT)) || ((speed < 0) && (_polarity == INVERSE))){
		digitalWrite(_directionPin, LOW);
		analogWrite(_speedPin, speed);
	}
	else{
		digitalWrite(_directionPin, HIGH);
		analogWrite(_speedPin, 255 - speed);
	}		
}
