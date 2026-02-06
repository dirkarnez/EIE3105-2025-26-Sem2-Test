#ifdef __AVR_ATmega328P__
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NORMAL_MODE_VALUE(timer_bit, n_seconds, prescaler) ((int)(((1UL) << (timer_bit)) - ((n_seconds) * ((F_CPU) / (prescaler)))))
#define CTC_MODE_VALUE(n_seconds, prescaler) ((int)(((n_seconds) * ((F_CPU) / (prescaler))) - (1UL)))

/* 
https://github.com/arduino/ArduinoCore-avr/blob/87faf934a742fd6aa9fc269c99de5d529363f204/bootloaders/atmega/ATmegaBOOT_168.c#L375C1-L384C7

#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__)

#ifdef DOUBLE_SPEED
	UCSR0A = (1<<U2X0); //!!!!!!! Double speed mode USART0
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;
#else
	UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*16L)-1);
	UBRR0H = (F_CPU/(BAUD_RATE*16L)-1) >> 8;
#endif
*/
#define NORMAL_MODE_VALUE(timer_bit, n_seconds, prescaler) ((int)(((1UL) << (timer_bit)) - ((n_seconds) * ((F_CPU) / (prescaler)))))
#define CTC_MODE_VALUE(n_seconds, prescaler) ((int)(((n_seconds) * ((F_CPU) / (prescaler))) - (1UL)))

#define TICKS_NEEDED_FOR_CTC_1_SEC CTC_MODE_VALUE(1.0, 1024)
#define TICKS_NEEDED_FOR_CTC_HALF_SEC CTC_MODE_VALUE(0.5, 1024)

#define UBRR_VALUE_LOW_SPEED(UART_BAUDRATE) ((unsigned char)(((F_CPU)/((UART_BAUDRATE) * (16UL)))-((double)(1UL))))
#define UBRR_VALUE_DOUBLE_SPEED(UART_BAUDRATE) ((unsigned char)(((F_CPU)/((UART_BAUDRATE) * (8L)))-((double)(1UL))))

#include <avr/io.h>

void Timer_1_Delay();		// Prototype for Delay Function


void usart_init_interupt_mode()
{
	UCSR0B = (1<<TXEN0) /*enable TX*/ | (1<<RXEN0) /* enable RX */| (1<<UDRIE0) /* Register Empty Interrupt */| (1<<RXCIE0) /* Complete Interrupt Enable */;
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);  // no parity, 1 stop bit, 8-bit data
	// UBRR0 = UBRR_VALUE_LOW_SPEED(9600);

	UCSR0A = (1<<U2X0); //Double speed mode USART0
	UBRR0 = UBRR_VALUE_DOUBLE_SPEED(115200);

	// UBRR0L = (uint8_t)(F_CPU/(115200*16L)-1);
	// UBRR0H = (F_CPU/(115200*16L)-1) >> 8;
} 
 
#define OCRA0_VALUE(TARGET_FREQ, PRESCALER) (((unsigned char)((unsigned int)((unsigned int)((F_CPU) / (TARGET_FREQ)) / (PRESCALER)))) - 1UL)

// wave frequency to 500 Hz. The duty cycle should be 50%.
void Timer_0() {
	//TCCR0A =
	//(1 << COM0A1) | // 7
	//(1 << COM0A0) | // 6
	//(1 << COM0B1) | // 5
	//(1 << COM0B0) | // 4, nothing in 3 and 2
	//
	//(1 << WGM01) |  // 1
	//(1 << WGM00);   // 0
//
	//TCCR0B =
	//(1 << FOC0A) |  // 7
	//(1 << FOC0B) |  // 6
	//(1 << WGM02) |  // 3
	//(1 << CS02) |   // 2
	//(1 << CS01) |   // 1
	//(1 << CS00);    // 0
	
	// timer mode 7: 
	TCCR0A = (1 << COM0A1) | (0 << COM0A0) | //00
		(1 << COM0B1) | (0 << COM0B0) |
		(1 << WGM01) | (1 << WGM00);
		
	TCCR0B = (1 << WGM02) |
		(1 << CS02) | (0 << CS01) | (0 << CS00); // prescaler 256
		
	unsigned char ocr0a_value = OCRA0_VALUE(500, 256);
	OCR0A = ocr0a_value; //64kHz,  ((F_CPU) / (64000)) - 1
	// 	OCR0B = (unsigned char) ocr0a_value * 0.5; //20% duty cycle, 249 * 0.2
	DDRD = 0b00100000; // PD5 (OC0B), have to set as output
}

int state = 0;

ISR(TIMER1_COMPA_vect)
{
	if (state == 0) {
		OCR0B = (unsigned char) ( OCRA0_VALUE(500, 256) * 0.8); //20% duty cycle, 249 * 0.2
		state = 1;
	} else if (state == 1) {
		OCR0B = (unsigned char) (OCRA0_VALUE(500, 256) * 0.2); //20% duty cycle, 249 * 0.2
		state = 0;
	}
}

// WGM1x 0 1 0 0 CTC
// CS1x 1 0 1 /1024
void timer_1_ctc_interrupt_init() {
	// _Static_assert((1UL << 16) > TICKS_NEEDED_FOR_CTC && TICKS_NEEDED_FOR_CTC >= 0, "Overflow");

	OCR1A = TICKS_NEEDED_FOR_CTC_1_SEC;

	TCCR1A = (0 << COM1A1)| // 7
	(0 << COM1A0)| // 6
	(0 << COM1B1)| // 5
	(0 << COM1B0)| // 4, nothing in 3 and 2
	(0 << WGM11)| // 1
	(0 << WGM10); //0

	TCCR1B = (0 << ICNC1)| // 7
	(0 << ICES1)| //  6
	(0 << WGM13)| //  4, nothing in 5
	(1 << WGM12)| //  3
	(1 << CS12)| //  2
	(0 << CS11)| //  1
	(1 << CS10); // 0

	TIMSK1 = (1<<OCIE1A);	// Enable Interrupt (Output Compare Interupt Enable Timer 1 Channel A)
}


int main(void)
{
	timer_1_ctc_interrupt_init();
	sei();
    while (1) {
		Timer_0();
	}
}

#else
#include "gtest/gtest.h"
using ::testing::InitGoogleTest;

// Demonstrate some basic assertions.
TEST(MyTest, BasicAssertions) {
	EXPECT_EQ(UBRR_VALUE_LOW_SPEED(9600), 103);
	EXPECT_EQ(UBRR_VALUE_LOW_SPEED(4800), 207);
	EXPECT_EQ(UBRR_VALUE_DOUBLE_SPEED(115200), 16);
}


int main(int argc, char** argv) {
  	InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif

