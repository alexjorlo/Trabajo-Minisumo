#include "Counter.h"

Counter_internal_state_t * Counter::interruptArgs[];

Counter::Counter(uint8_t pin) {
	
	#warning Counter need that pin having interruption

	pinMode(pin, INPUT_PULLUP);

	state.pin = pin;
	state.value = 0;

	interrupt_in_use = attach_interrupt(pin, &state);
	
}

uint32_t Counter::read() {
	
	noInterrupts();
	uint32_t value = state.value;
	interrupts();
	return value;
}
	
void Counter::write(uint32_t newValue) {

	noInterrupts();
	state.value = newValue;
	interrupts();
}
