/*
 * MotorControllerFeedforward.c
 *
 * Created: 3/28/2023 3:20:49 PM
 * Author : KASO
 */ 

/*
	This code is used to count pulses
	from motor's Hall encoder so that
	PPR (Pulses per rotation) can be
	experimentally measured. The number
	or pulses is continually sent over UART.
*/

#ifndef F_CPU
	#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


/*
 *	Begin Macros
 */

#define SET_BIT(REG, BIT) REG |= _BV(BIT)
#define CLR_BIT(REG, BIT) REG &= ~_BV(BIT)
#define TGL_BIT(REG, BIT) REG ^= _BV(BIT)

#define IS_BIT_SET(REG, BIT) (REG & _BV(BIT))

#define GET_BYTE(REG, OFFSET) (uint8_t)((REG >> OFFSET) & 0xFF)

// Flag that is used to declare that a function or a block
// of code uses some resource named X. If a resource is reused
// i.e. multiple occurrences of USES_RESOURCE(X) then the compiler
// throws error (multiple variable definitions);
// HAS TO BE USED IN GLOBAL SCOPE
//#define USES_RESOURCE(X) uint8_t uses_resource_##X##_flag = 1;

#define UNUSED(X) (void)X;

/*
 *	End Macros
 */

/*
 *	Begin Pin Definitions
 */


/*
 *	End Pin Definitions
 */

/*
 *	Begin Constants
 */ 

// Baud rate = 115.2kbps; U2X=0
#if (F_CPU == 16000000)
	#define BAUD_RATE_UBBR_115_2_KBPS 8
#elif (F_CPU == 8000000)
	#define BAUD_RATE_UBBR_115_2_KBPS 3
#else
	#error "Baud rate not supported for frequency"
#endif

/*
 *	End Constants
 */


/*
 *	Start Global Variables
 */

volatile uint8_t pulse_counter = 0;

/*
 *	End Global Variables
 */

/*
 *	Start User Code Declaration
 */


/*
 *	End User Code Declaration
 */


/*
 *	Start User Code Implementation
 */

void enable_encoder_interrupt(void)
{
	// Configure PD2 as input
	CLR_BIT(DDRD, DDD2);
	
	// Enable pullup (IG32E Hall encoder docs require 1k external pullup)
	SET_BIT(PORTD, PORTD2);
	CLR_BIT(MCUCR, PUD);
	
	// Set rising edge mode for INT0 (PD2)
	SET_BIT(EICRA, ISC00);
	SET_BIT(EICRA, ISC01);
	
	// Enable external interrupt
	SET_BIT(EIMSK, INT0);
}


void setup_usart(void)
{
	// USART0
	
	// Set baud rate to 115.2 kbps
	UBRR0 = (uint16_t)BAUD_RATE_UBBR_115_2_KBPS;

	// Enable transmitter
	SET_BIT(UCSR0B, TXEN0);
	
	// Set frame format: 8data, 1 stop bit
	SET_BIT(UCSR0C, UCSZ00);
	SET_BIT(UCSR0C, UCSZ01);
	CLR_BIT(UCSR0B, UCSZ02);

}

int main(void)
{
	// Setup
	setup_usart();
	enable_encoder_interrupt();
	//_delay_ms(STEP_INPUT_DELAY_MS);
	
	
	sei();
    while (1) 
    {
		if(IS_BIT_SET(UCSR0A, UDRE0))
		{
			UDR0 = pulse_counter;
		}
    }
}

/*
 *	End User Code Implementation
 */

/*
 *	Start Signal Handlers
 */

ISR(INT0_vect)
{
	++pulse_counter;
}

/*
 *	End Signal Handlers
 */

