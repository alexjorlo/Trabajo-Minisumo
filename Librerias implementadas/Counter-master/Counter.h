/* Counter Library, for measuring pulses
 * https://github.com/joruiz/Counter
 * Copyright (c) 2016 Jonathan Ruiz de Garibay
 *
 * Version 0.9 - initial release (no checked)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef COUNTER_H
#define COUNTER_H

#include "Arduino.h"
#include "./interrupt_pins.h"

typedef struct {
	uint8_t pin;
	uint32_t value;
} Counter_internal_state_t;

class Counter
{
	public:
		Counter(uint8_t pin);
		uint32_t read();
		void write(uint32_t newValue);
	
		static Counter_internal_state_t * interruptArgs[CORE_NUM_INTERRUPT];	
	private:
		Counter_internal_state_t state;
		uint8_t interrupt_in_use;
		
		static void update(Counter_internal_state_t *arg){
			
			arg->value++;
		}		

		static uint8_t attach_interrupt(uint8_t pin, Counter_internal_state_t *state) {

			switch (pin) {
		#ifdef CORE_INT0_PIN
				case CORE_INT0_PIN:
					interruptArgs[0] = state;
					attachInterrupt(0, isr0, RISING);
					break;
		#endif
		#ifdef CORE_INT1_PIN
				case CORE_INT1_PIN:
					interruptArgs[1] = state;
					attachInterrupt(1, isr1, RISING);
					break;
		#endif
		#ifdef CORE_INT2_PIN
				case CORE_INT2_PIN:
					interruptArgs[2] = state;
					attachInterrupt(2, isr2, RISING);
					break;
		#endif
		#ifdef CORE_INT3_PIN
				case CORE_INT3_PIN:
					interruptArgs[3] = state;
					attachInterrupt(3, isr3, RISING);
					break;
		#endif
		#ifdef CORE_INT4_PIN
				case CORE_INT4_PIN:
					interruptArgs[4] = state;
					attachInterrupt(4, isr4, RISING);
					break;
		#endif
		#ifdef CORE_INT5_PIN
				case CORE_INT5_PIN:
					interruptArgs[5] = state;
					attachInterrupt(5, isr5, RISING);
					break;
		#endif
		#ifdef CORE_INT6_PIN
				case CORE_INT6_PIN:
					interruptArgs[6] = state;
					attachInterrupt(6, isr6, RISING);
					break;
		#endif
				default:
					return 0;
			}
			return 1;
		}
		
		#ifdef CORE_INT0_PIN
		static void isr0() { update(interruptArgs[0]); }
		#endif

		#ifdef CORE_INT1_PIN
		static void isr1() { update(interruptArgs[1]); }
		#endif

		#ifdef CORE_INT2_PIN
		static void isr2() { update(interruptArgs[2]); }
		#endif

		#ifdef CORE_INT3_PIN
		static void isr3() { update(interruptArgs[3]); }
		#endif

		#ifdef CORE_INT4_PIN
		static void isr4() { update(interruptArgs[4]); }
		#endif

		#ifdef CORE_INT5_PIN
		static void isr5() { update(interruptArgs[5]); }
		#endif
};

#endif