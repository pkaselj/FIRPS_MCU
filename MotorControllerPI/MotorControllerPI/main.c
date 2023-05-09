/*
 * MotorControllerPI.c
 *
 * Created: 4/20/2023 10:56:31 PM
 * Author : KASO
 */ 

/*
	This code implements PID
	control of motor driven by
	vnh2sp30 motor driver.
*/


#ifndef F_CPU
	#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "pid.h"


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
// #define USES_RESOURCE(X) uint8_t uses_resource_##X##_flag = 1;

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
const float setpoint_rpm = 0.6f;

#define PID_KP	(float)3.3176
#define PID_TI	(float)16.5668

//#define PID_KP	(float)1.0f
//#define PID_TI	(float)1.0f

#define SAMPLING_FREQUENCY (float)62.5
#define SAMPLE_TIME_S 1/(float)SAMPLING_FREQUENCY

#define PULSES_PER_ROTATION 245

// Baud rate = 115.2kbps; U2X=0
#if (F_CPU == 16000000)
	#define BAUD_RATE_UBBR_115_2_KBPS 8
#elif (F_CPU == 8000000)
	#define BAUD_RATE_UBBR_115_2_KBPS 3
#else
	#error "Baud rate not supported for frequency"
#endif

// Defines maximum "believable" RPM value
// All RPM values above RPM_UPPER_DISCARD_LIMIT
// will not be saved as RPM values, instead old
// value will be held. (Crude low pass filter)
// (Disturbance rejection)
#define RPM_UPPER_DISCARD_LIMIT 100

/*
 *	End Constants
 */


/*
 *	Start Global Variables
 */

// Used by TIMER 1 overflow ISR to increment
// Used to keep track of RPM pulse counters (volatile, do not use)
volatile uint16_t pulse_tick_counter_high_nibble = 0;

// Used by INT0 ISR to store (buffer) rpm pulse counter
volatile uint32_t buffered_pulse_tick_counter = 0;

volatile uint16_t pulse_number = 0;
volatile uint8_t is_rpm_measurement_ready = 0;

volatile uint8_t flag_pid = 0;

#define SAMPLE_BUFFER_SIZE 16
volatile uint8_t sample_buffer[SAMPLE_BUFFER_SIZE] = {0};

float g_current_rpm = 0;
float g_current_duty_cycle_input = 0;
float g_current_error = 0;
	
pid_t g_pid;

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

void configure_motor_pwm_timer(void)
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
}

// duty_cycle = [0..100]
void set_motor_speed_pwm(uint32_t duty_cycle)
{
	// Set phase corrected PWM value
	// from defined duty cycle
	// NOTE: Formula used is modified for integer division
	
	OCR0B = (uint8_t)((255 * ((uint32_t)100 - duty_cycle))/100);
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

void do_update_rpm(void)
{	
	float new_rpm = F_CPU/((float)buffered_pulse_tick_counter * PULSES_PER_ROTATION);
	
	// Crude low-pass (disturbance rejection) filter
	if(new_rpm < 10)
	{
		g_current_rpm = new_rpm;
	}
}

void setup_pid_timer(void)
{
	// Time for timer to tick once T1 = 256/F_CPU (prescaler = 256).
	// If we want timer to interrupt every T seconds, we must set compare register
	// to N ticks where N = T/T1 = F_CPU/F_S where F_S is the sampling (interrupt) frequency.
	// For explanation of (- 1) at the end, see documentation
	//const uint8_t timer_compare_value = (uint8_t)(F_CPU/(1024 * SAMPLING_FREQUENCY));
	const uint8_t timer_compare_value = 250;
	// Set output compare register A value
	OCR2A = timer_compare_value;
	
	// Initialize timer value
	TCNT2 = 0;
	
	// Set OC CTC mode of operation
	CLR_BIT(TCCR2A, WGM20);
	CLR_BIT(TCCR2A, WGM21);
	SET_BIT(TCCR2B, WGM22);
}

void enable_pid_timer(void)
{
	// Enable interrupt
	SET_BIT(TIMSK2, OCIE2A);
	
	// Enable clock (prescaler = 1024)
	CLR_BIT(TCCR2B, CS20);
	CLR_BIT(TCCR2B, CS21);
	SET_BIT(TCCR2B, CS22);
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

void setup_PID(void)
{
	PID_Init(&g_pid, PID_KP, 0, PID_TI, 0, 95);
}

int main(void)
{
	// Setup
	setup_gpio_pins();
	set_motor_direction();
	
	configure_pulse_tick_timer();
	setup_usart();
	
	setup_PID();
	
	//_delay_ms(STEP_INPUT_DELAY_MS);
	
	enable_encoder_interrupt();
	enable_pulse_tick_timer();
	configure_motor_pwm_timer();
	set_motor_speed_pwm(30);

	
	sei();
	
	 //Ignore first measurement
	 
	while(!is_rpm_measurement_ready)
	{
		;
	}
	is_rpm_measurement_ready = 0;
	
	setup_pid_timer();
	enable_pid_timer();
	
	
    while (1) 
    {
		if(is_rpm_measurement_ready)
		{
			do_update_rpm();
			is_rpm_measurement_ready = 0;
		}
		
		if(flag_pid)
		{
			g_current_error = setpoint_rpm - g_current_rpm;
			g_current_duty_cycle_input = PID_Advance(&g_pid, SAMPLE_TIME_S, g_current_error);
			set_motor_speed_pwm((uint32_t)g_current_duty_cycle_input);
			flag_pid = 0;
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
	// Save old values for UART
	buffered_pulse_tick_counter = TCNT1;
	buffered_pulse_tick_counter |= ((uint32_t)(pulse_tick_counter_high_nibble)) << 16;
	
	// Signal data to UART
	is_rpm_measurement_ready = 1;
	
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

ISR(TIMER2_COMPA_vect)
{
	flag_pid = 1;
}
/*
 *	End Signal Handlers
 */


