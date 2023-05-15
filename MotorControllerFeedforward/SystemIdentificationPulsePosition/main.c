/*
 * MotorControllerFeedforward.c
 *
 * Created: 3/28/2023 3:20:49 PM
 * Author : KASO
 */ 

/*
	This code is used for system identification
	of vnh2sp30 board driven DC motor.
	On positive pulse edge of encoder, timer is 
	sampled and value is sent over UART
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

// VNH2SP30 Speed Control Pin = PD5
#define SPEED_CTL_PORT	PORTD
#define SPEED_CTL_BIT	PORTD5

// VNH2SP30 IN A Pin = PD7
#define IN_A_PORT	PORTD
#define IN_A_BIT	PORTD7

// VNH2SP30 IN B Pin = PB0
#define IN_B_PORT	PORTB
#define IN_B_BIT	PORTB0

// VNH2SP30 EN Pin = PC0
#define EN_PORT	PORTC
#define EN_BIT	PORTC0

/*
 *	End Pin Definitions
 */

/*
 *	Begin Constants
 */ 

// System Input

// Time delay of step function input
#define STEP_INPUT_DELAY_MS 3000

// Motor PWM duty cycle step function
// steady state value (percentage 0-100)
#define DUTY_CYCLE 30

// Motor PWM input duty cycle percentage denominator
// PWM_DUTY_CYCLE = 100 / DUTY_CYCLE_DENOMINATOR
#define DUTY_CYCLE_DENOMINATOR (uint8_t)(100.0 / DUTY_CYCLE)

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

// Used by TIMER 1 overflow ISR to increment
volatile uint16_t pulse_tick_counter_high_nibble = 0;

// Used by INT0 ISR to store (buffer) data for sending (UART)
volatile uint16_t buffered_pulse_tick_counter_high_nibble = 0;
volatile uint16_t buffered_pulse_tick_counter_low_nibble = 0;

volatile uint16_t pulse_number = 0;
volatile uint8_t should_sample_flag = 0;

#define SAMPLE_BUFFER_SIZE 7
volatile uint8_t sample_buffer[SAMPLE_BUFFER_SIZE] = {0};

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

void setup_gpio_pins(void)
{
	// == LED pin = PB5
	SET_BIT(DDRB, DDB5);
	
	// == IN A Pin = PD7
	SET_BIT(DDRD, DDD7);
	
	// == IN B Pin = PB0
	SET_BIT(DDRB, DDB0);
	
	// == PWM Pin = PD5
	SET_BIT(DDRD, DDD5);
	
	// == EN Pin = PC0
	SET_BIT(DDRC, DDC0);
}

void set_motor_direction(void)
{
	// Clockwise direction = INA & ~INB
	SET_BIT(IN_A_PORT, IN_A_BIT);
	CLR_BIT(IN_B_PORT, IN_B_BIT);
}

void set_motor_enable(void)
{
	SET_BIT(EN_PORT, EN_BIT);
}

void set_motor_speed_pwm(void)
{
	// In phase corrected PWM mode:
	// Set on upcount, clear on downcount
	SET_BIT(TCCR0A, COM0B0);
	SET_BIT(TCCR0A, COM0B1);
	
	// Set phase corrected PWM mode of operation
	SET_BIT(TCCR0A, WGM00);
	CLR_BIT(TCCR0A, WGM01);
	CLR_BIT(TCCR0B, WGM02);
	
	// Set clock prescaler to 1/8
	// Which gives 16MHz/(8 * 510) = 3.9 kHz PWM frequency
	CLR_BIT(TCCR0B, CS00);
	SET_BIT(TCCR0B, CS01);
	CLR_BIT(TCCR0B, CS02);
	
	// Disable interrupts
	// Overflow
	CLR_BIT(TIMSK0, TOV0);
	// Output Compare B Match
	CLR_BIT(TIMSK0, OCIE0B);
	
	// Set phase corrected PWM value
	// from defined duty cycle
	// NOTE: Formula used is modified for integer division
	
	OCR0B = (uint8_t)((255 * ((uint32_t)100 - DUTY_CYCLE))/100);
}


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

void configure_pulse_tick_timer(void) 
{
	// Set normal mode of operation
	CLR_BIT(TCCR1A, COM1A0);
	CLR_BIT(TCCR1A, COM1A1);
	
	CLR_BIT(TCCR1A, WGM10);
	CLR_BIT(TCCR1A, WGM11);
	CLR_BIT(TCCR1B, WGM12);
	CLR_BIT(TCCR1B, WGM13);
	
	// Initialize timer value
	TCNT1 = 0;
}

void enable_pulse_tick_timer(void)
{
	// Enable overflow interrupt	
	SET_BIT(TIMSK1, TOIE1);
	
	// Enable clock (no prescaler)
	SET_BIT(TCCR1B, CS10);
	CLR_BIT(TCCR1B, CS11);
	CLR_BIT(TCCR1B, CS12);
}

void do_sample_data(void)
{	
	sample_buffer[0] = GET_BYTE(pulse_number, 0);
	sample_buffer[1] = GET_BYTE(pulse_number, 8);
	sample_buffer[2] = GET_BYTE(buffered_pulse_tick_counter_low_nibble, 0);
	sample_buffer[3] = GET_BYTE(buffered_pulse_tick_counter_low_nibble, 8);
	sample_buffer[4] = GET_BYTE(buffered_pulse_tick_counter_high_nibble, 0);
	sample_buffer[5] = GET_BYTE(buffered_pulse_tick_counter_high_nibble, 8);
	sample_buffer[6] = '\n';
	
	pulse_number++;
}

void send_sample_data(void)
{	
	for (uint8_t i = 0; i < SAMPLE_BUFFER_SIZE; i++)
	{
		while (!IS_BIT_SET(UCSR0A, UDRE0))
		{
			//_delay_ms(10);
			;
		}
		
		// Put data into buffer
		UDR0 = sample_buffer[i];
	}
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
	setup_gpio_pins();
	set_motor_direction();
	
	configure_pulse_tick_timer();
	setup_usart();
	
	_delay_ms(STEP_INPUT_DELAY_MS);
	
	enable_encoder_interrupt();
	enable_pulse_tick_timer();
	set_motor_enable();
	set_motor_speed_pwm();
	
	
	sei();
	
	// Ignore first measurement
	while(!should_sample_flag)
	{
		;
	}
	should_sample_flag = 0;
	
	
    while (1) 
    {
		while(!should_sample_flag)
		{
			;
		}
		
		do_sample_data();
		send_sample_data();
		should_sample_flag = 0;
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
	// Save old values for UART
	buffered_pulse_tick_counter_low_nibble = TCNT1;
	buffered_pulse_tick_counter_high_nibble = pulse_tick_counter_high_nibble;
	
	// Signal data to UART
	should_sample_flag = 1;
	
	// Clear timer/counter
	TCNT1 = 0;
	pulse_tick_counter_high_nibble = 0;
	
	
}

ISR(TIMER1_OVF_vect)
{
	++pulse_tick_counter_high_nibble;
	
	// For debugging and calculating
	// timer period using oscilloscope
	TGL_BIT(PORTB, PORTB5); 
}
/*
 *	End Signal Handlers
 */

