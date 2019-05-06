// interrupt pins for known boards

#if !defined(CORE_NUM_INTERRUPT)

	// Arduino Uno, Duemilanove, Diecimila, LilyPad, Mini, Fio, etc...
	#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__)
	  #define CORE_NUM_INTERRUPT	2
	  #define CORE_INT0_PIN		2
	  #define CORE_INT1_PIN		3

	// Arduino Mega
	#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
	  #define CORE_NUM_INTERRUPT	6
	  #define CORE_INT0_PIN		2
	  #define CORE_INT1_PIN		3
	  #define CORE_INT2_PIN		21
	  #define CORE_INT3_PIN		20
	  #define CORE_INT4_PIN		19
	  #define CORE_INT5_PIN		18

	// Arduino Leonardo, Micro
	#elif defined(__AVR_ATmega32U4__) && !defined(CORE_TEENSY)
	  #define CORE_NUM_INTERRUPT	5
	  #define CORE_INT0_PIN		3
	  #define CORE_INT1_PIN		2
	  #define CORE_INT2_PIN		0
	  #define CORE_INT3_PIN		1
	  #define CORE_INT4_PIN		7

	#endif
#endif

#if !defined(CORE_NUM_INTERRUPT)
#error "Interrupts are unknown for this board, please add to this code"
#endif
#if CORE_NUM_INTERRUPT <= 0
#error "Counter requires interrupt pins, but this board does not have any :("
#endif

