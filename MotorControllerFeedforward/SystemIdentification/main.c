/*
 * MotorControllerFeedforward.c
 *
 * Created: 3/28/2023 3:20:49 PM
 * Author : KASO
 */ 

/*
	This code is used for system identification
	of vnh2sp30 board driven DC motor.
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

// Sampling frequency and interval for sampling
// motor response
#define SAMPLING_FREQUENCY 1000
#define SAMPLING_INTERVAL_US (uint32_t)(1000000.0/SAMPLING_FREQUENCY)
//#define SAMPLING_INTERVAL_MS (uint32_t)(1000.0/SAMPLING_FREQUENCY)

// System Input

// Time delay of step function input
#define STEP_INPUT_DELAY_MS 3000

// Motor PWM duty cycle step function
// steady state value (percentage 0-100)
#define DUTY_CYCLE 50

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

volatile uint32_t encoder_ticks = 0;
volatile uint8_t should_sample_flag = 0;
volatile uint32_t sample_number = 0;

const uint8_t sample_buffer_size = 12;
//const uint8_t sample_buffer_size = 1;
volatile uint8_t sample_buffer[] = {
	0,			/* SAMPLE NUMBER */
	0,
	0,
	0,
	',',
	(uint8_t)DUTY_CYCLE,			/* INPUT VALUE */
	',',
	0,			/* RESPONSE VALUE */
	0,
	0,
	0,
	'\n'
};

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

// Uses TIMER0
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

void configure_response_sampling_timer(void) 
{
	// Time for timer to tick once T1 = 256/F_CPU (prescaler = 256).
	// If we want timer to interrupt every T seconds, we must set compare register
	// to N ticks where N = T/T1 = F_CPU/F_S where F_S is the sampling (interrupt) frequency.
	// For explanation of (- 1) at the end, see documentation
	const uint16_t timer_compare_value = (uint16_t)(F_CPU/(256 * (uint32_t)SAMPLING_FREQUENCY));
	//const uint16_t timer_compare_value = (uint16_t)(0xFFFF);
	// Set output compare register A value
	OCR1A = timer_compare_value;
	
	// Initialize timer value
	TCNT1 = 0;
	
	// Set OC CTC mode of operation
	CLR_BIT(TCCR1A, WGM10);
	CLR_BIT(TCCR1A, WGM11);
	SET_BIT(TCCR1B, WGM12);
	CLR_BIT(TCCR1B, WGM13);
}

void enable_response_sampling_timer(void)
{
	// Enable interrupt	
	SET_BIT(TIMSK1, OCIE1A);
	
	// Enable clock (prescaler = 256)
	CLR_BIT(TCCR1B, CS10);
	CLR_BIT(TCCR1B, CS11);
	SET_BIT(TCCR1B, CS12);
}

void do_sample_data(void)
{
	sample_buffer[0] = *((uint8_t*)&sample_number + 0);
	sample_buffer[1] = *((uint8_t*)&sample_number + 1);
	sample_buffer[2] = *((uint8_t*)&sample_number + 2);
	sample_buffer[3] = *((uint8_t*)&sample_number + 3);
	//sample_buffer[2] = input_value; // HARDCODED
	sample_buffer[7]  = *((uint8_t*)&encoder_ticks + 0);
	sample_buffer[8]  = *((uint8_t*)&encoder_ticks + 1);
	sample_buffer[9]  = *((uint8_t*)&encoder_ticks + 2);
	sample_buffer[10] = *((uint8_t*)&encoder_ticks + 3);
	
	sample_number++;
}

void send_sample_data(void)
{	
	for (uint8_t i = 0; i < sample_buffer_size; i++)
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
	
	configure_response_sampling_timer();
	setup_usart();
	
	_delay_ms(STEP_INPUT_DELAY_MS);
	
	enable_response_sampling_timer();
	enable_encoder_interrupt();
	set_motor_enable();
	set_motor_speed_pwm();
	
	
	sei();
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
	++encoder_ticks;
}

ISR(TIMER1_COMPA_vect)
{
	should_sample_flag = 1;
	TGL_BIT(PORTB, PORTB5);
}
/*
 *	End Signal Handlers
 */

