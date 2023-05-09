/*
 * DuoMotorControllerPI.c
 *
 * Created: 5/2/2023 8:13:33 PM
 * Author : KASO
 */ 

/*
	This code implements PI
	control for TWO 
	vnh2sp30 motor drivers.
*/

#ifndef F_CPU
	#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pid.h"


/*
 *	Begin Macros
 */

#ifndef NULL
#define NULL (void*)0
#endif

//////////////////////////////////////////////////////////////////////////
// -- Bitwise Operators
#define SET_BIT(REG, BIT) REG |= _BV(BIT)
#define CLR_BIT(REG, BIT) REG &= ~_BV(BIT)
#define TGL_BIT(REG, BIT) REG ^= _BV(BIT)

#define IS_BIT_SET(REG, BIT) (REG & _BV(BIT))

#define GET_BYTE(REG, OFFSET) (uint8_t)((REG >> OFFSET) & 0xFF)
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// ------ Pin Modes
#define PIN_MODE_OUTPUT(REG, BIT) SET_BIT(REG, BIT)
#define PIN_MODE_INPUT(REG, BIT)  CLR_BIT(REG, BIT)
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Flag that is used to declare that a function or a block
// of code uses some resource named X. If a resource is reused
// i.e. multiple occurrences of USES_RESOURCE(X) then the compiler
// throws error (multiple variable definitions);
// HAS TO BE USED IN GLOBAL SCOPE
// #define USES_RESOURCE(X) uint8_t uses_resource_##X##_flag = 1;

#define UNUSED(X) (void)X;
//////////////////////////////////////////////////////////////////////////

/*
 *	End Macros
 */

/*
 *	Begin Type Definitions
 */

typedef struct {
	uint32_t timer_value;
	uint32_t buffered_timer_value;
	float current_rps;
	uint8_t  is_measurement_ready;
} hall_encoder_t;

typedef struct {
	hall_encoder_t hall_encoder;
	pid_t pid;
	float setpoint;
} motor_t;

/*
 *	End Type Definitions
 */


/*
 *	Begin Pin Definitions
 */

// Connection table for two VNH2SP30 motor controllers
// * Pin format: ATMEGA328P (ARDUINO UNO)

// +=========+============+===========+===========+=============+==============+=======+
// |         |    IN A    |    IN B   |    PWM    |  HALL CH A  |  OC REGISTER |  EXTI |
// +=========+============+===========+===========+=============+==============+=======+
// | MOTOR1  |   PD7(7)   |  PB1(9)   |  PD5(5)   |   PD2(2)    |   OC0B       |  INT0 |
// +---------+------------+-----------+-----------+-------------+--------------+-------+
// | MOTOR2  |   PB0(8)   |  PB2(10)  |  PD6(6)   |   PD3(3)    |   OC0A       |  INT1 |
// +---------+------------+-----------+-----------+-------------+--------------+-------+



/*
 *	End Pin Definitions
 */

/*
 *	Begin Constants
 */ 

//////////////////////////////////////////////////////////////////////////
// ------- Rover UART Commands

typedef enum {
	COMMAND_FORWARD,
	COMMAND_BACKWARD,
	COMMAND_LEFT,
	COMMAND_RIGHT,
	COMMAND_STOP,
	COMMAND_UNKNOWN
} command_e;

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// ------- PID

//#define PID_KP	(float)1.4049
//#define PID_TI	(float)2.1316

//#define PID_KP	(float)3.3176
//#define PID_TI	(float)16.5668

#define PID_KP	(float)2.2049
#define PID_TI	(float)128.8773

#define SAMPLING_FREQUENCY 62.5f
#define SAMPLE_TIME_S (1.0f/SAMPLING_FREQUENCY)
//////////////////////////////////////////////////////////////////////////

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
#define RPM_UPPER_DISCARD_LIMIT 10

// Commands last for 3.008 seconds (T_PID = 1/62.5 s)
const uint32_t command_duration_pids = 188;

const float motor_on_rps = 0.5;

/*
 *	End Constants
 */


/*
 *	Start Global Variables
 */

// Used by TIMER 1 overflow ISR to increment
// Used to keep track of RPM pulse counters (volatile, do not use)
uint16_t pulse_tick_counter_high_nibble = 0;

// Motors
motor_t g_motor_1;
motor_t g_motor_2;

uint8_t g_flag_pid = 0;

uint32_t g_command_timer_pids;
uint8_t g_flag_command_received = 0;
volatile int8_t g_command_buffer = 0;
uint8_t g_flag_command_running = 0;

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

void debug_led_on(void)
{
	SET_BIT(PORTB, PORTB5);
}

void debug_led_off(void)
{
	CLR_BIT(PORTB, PORTB5);
}

void debug_led_toggle(void)
{
	TGL_BIT(PORTB, PORTB5);
}

void do_blink_debug_led(void)
{
	debug_led_on();
	_delay_ms(500);
	debug_led_off();
	_delay_ms(500);
}

void do_blink_debug_led_times(int times)
{
	int i;
	for (i = 0; i < times; ++i)
	{
		do_blink_debug_led();
	}
}

void do_handle_fatal_error(void)
{
	// Signal fatal error with debug LED blinking
	while(1)
	{
		debug_led_toggle();
		_delay_ms(50);
	}
}

void usart_send(unsigned char* pData, int length)
{
	// Send STX (Start Byte)
	while (!IS_BIT_SET(UCSR0A, UDRE0))
	{
		;
	}
	UDR0 = 0x02;
	
	// Send Data
	int i;
	for (i = 0; i < length; ++i)
	{
		while (!IS_BIT_SET(UCSR0A, UDRE0))
		{
			;
		}
		UDR0 = pData[i];
	}
}

void usart_send_reorder(unsigned char* pData, int length)
{
	// Send STX (Start Byte)
	while (!IS_BIT_SET(UCSR0A, UDRE0))
	{
		;
	}
	UDR0 = 0x02;
	
	// Send Data
	int i;
	for (i = length-1; i >= 0; --i)
	{
		while (!IS_BIT_SET(UCSR0A, UDRE0))
		{
			;
		}
		UDR0 = pData[i];
	}
}

inline void hall_encoder_do_save_timer_value(hall_encoder_t* hEncoder)
{
	if (NULL == hEncoder)
	{
		do_handle_fatal_error();
		return;
	}
	
	// Save timer value
	hEncoder->buffered_timer_value = hEncoder->timer_value;
	
	// Save timestamp
	hEncoder->timer_value = TCNT1;
	hEncoder->timer_value |= ((uint32_t)(pulse_tick_counter_high_nibble)) << 16;
	
	// Signal scheduler to calculate RPS
	hEncoder->is_measurement_ready = 1;
}


void setup_gpio_pins(void)
{
	// DEBUG INBUILD-LED
	PIN_MODE_OUTPUT(DDRB, DDB5);
	
	// --- MOTOR 1
		// IN A - PD7
		PIN_MODE_OUTPUT(DDRD, DDD7);
		// IN B - PB1
		PIN_MODE_OUTPUT(DDRB, DDB1);
		// PWM - PD5
		PIN_MODE_OUTPUT(DDRD, DDD5);
		// HALL CH A - PD2
		PIN_MODE_INPUT(DDRD, DDD2);
		
	// --- MOTOR 2
		// IN A - PB0
		PIN_MODE_OUTPUT(DDRB, DDB0);
		// IN B - PB2
		PIN_MODE_OUTPUT(DDRB, DDB2);
		// PWM - PD6
		PIN_MODE_OUTPUT(DDRD, DDD6);
		// HALL CH A - PD3
		PIN_MODE_INPUT(DDRD, DDD3);
		
	// UART
		PIN_MODE_INPUT(DDRD, DDD0);
		PIN_MODE_OUTPUT(DDRD, DDD1);
}

void set_motor_direction(command_e command)
{
	// Clockwise direction = INA & ~INB
	
	// --- MOTOR 1 : IN A (PD7), IN B (PB1)
	// --- MOTOR 2 : IN A (PB0), IN B (PB2)

	switch(command)
	{
		case COMMAND_FORWARD:
		//do_blink_debug_led_times(1);
		
		// Motor 1
		SET_BIT(PORTD, PORTD7);
		CLR_BIT(PORTB, PORTB1);
		
		// Motor 2
		SET_BIT(PORTB, PORTB0);
		CLR_BIT(PORTB, PORTB2);
		
		break;
		
		case COMMAND_BACKWARD:
		//do_blink_debug_led_times(2);
		// Motor 1
		CLR_BIT(PORTD, PORTD7);
		SET_BIT(PORTB, PORTB1);
		
		// Motor 2
		CLR_BIT(PORTB, PORTB0);
		SET_BIT(PORTB, PORTB2);
		break;
		
		case COMMAND_LEFT:
		//do_blink_debug_led_times(3);
		// Motor 1
		SET_BIT(PORTD, PORTD7);
		CLR_BIT(PORTB, PORTB1);
		
		// Motor 2
		CLR_BIT(PORTB, PORTB0);
		SET_BIT(PORTB, PORTB2);
		break;
		
		case COMMAND_RIGHT:
		//do_blink_debug_led_times(4);
		// Motor 1
		CLR_BIT(PORTD, PORTD7);
		SET_BIT(PORTB, PORTB1);
		
		// Motor 2
		SET_BIT(PORTB, PORTB0);
		CLR_BIT(PORTB, PORTB2);
		break;
		
		case COMMAND_STOP:
		// Motor 1
		CLR_BIT(PORTD, PORTD7);
		CLR_BIT(PORTB, PORTB1);
		
		// Motor 2
		CLR_BIT(PORTB, PORTB0);
		CLR_BIT(PORTB, PORTB2);
		break;
		
		default:
		//do_blink_debug_led_times(5);
		// Do nothing
		break;
	}

}

void configure_motor_pwm_timer(void)
{
	//////////////////////////////////////////////////////////////////////////
	// Two motors need two PWM channels (Output Compare Registers):
	// - 8 bit TIMER0 Channels A & B
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// -- Timer 0
	// Phase corrected PWM mode
	
	// Set phase corrected PWM mode of operation
	SET_BIT(TCCR0A, WGM00);
	CLR_BIT(TCCR0A, WGM01);
	CLR_BIT(TCCR0B, WGM02);
	
	// Set on upcount, clear on downcount
	// Output Compare - A
	SET_BIT(TCCR0A, COM0A0);
	SET_BIT(TCCR0A, COM0A1);
	
	// Output Compare - B
	SET_BIT(TCCR0A, COM0B0);
	SET_BIT(TCCR0A, COM0B1);

	// Set clock prescaler to 1/8
	// Which gives 16MHz/(8 * 510) = 3.9 kHz PWM frequency
	CLR_BIT(TCCR0B, CS00);
	SET_BIT(TCCR0B, CS01);
	CLR_BIT(TCCR0B, CS02);
	
	// Disable interrupts
	// Overflow
	CLR_BIT(TIMSK0, TOV0);
	// Output Compare B Match
	CLR_BIT(TIMSK0, OCIE0A);
	// Output Compare B Match
	CLR_BIT(TIMSK0, OCIE0B);
	//////////////////////////////////////////////////////////////////////////
}

// input:
// - duty_cycle = [0..100] = Duty Cycle Percentage
// output:
// - Output compare value for:
//		* 8 bit timer
//		* Phase corrected PWM mode
//		* Set on upcount, clear on downcount
uint8_t calculate_oc_value_from_dc(uint32_t duty_cycle)
{
	// Set phase corrected PWM value
	// from defined duty cycle
	// NOTE: Formula used is modified for integer division
	
	return (uint8_t)((255 * ((uint32_t)100 - duty_cycle))/100);
}


void enable_encoder_interrupt(void)
{
	// Enable Pullups (Disable pullup blockade)
	CLR_BIT(MCUCR, PUD);
	
	//////////////////////////////////////////////////////////////////////////
	// -- MOTOR 1
	// Configure PD2 (INT0) as input
	CLR_BIT(DDRD, DDD2);
	
	// Enable pullup (IG32E Hall encoder docs require 1k external pullup)
	SET_BIT(PORTD, PORTD2);
	
	// Set rising edge mode for INT0 (PD2)
	SET_BIT(EICRA, ISC00);
	SET_BIT(EICRA, ISC01);
	
	// Enable external interrupt
	SET_BIT(EIMSK, INT0);
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// -- MOTOR 2
	// Configure PD3 (INT1) as input
	CLR_BIT(DDRD, DDD3);
	
	// Enable pullup (IG32E Hall encoder docs require 1k external pullup)
	SET_BIT(PORTD, PORTD3);
	
	// Set rising edge mode for INT1 (PD3)
	SET_BIT(EICRA, ISC10);
	SET_BIT(EICRA, ISC11);
	
	// Enable external interrupt
	SET_BIT(EIMSK, INT1);
	//////////////////////////////////////////////////////////////////////////

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

void do_update_rpm(hall_encoder_t* hEncoder)
{	
	if (NULL == hEncoder)
	{
		do_handle_fatal_error();
		return;
	}
	
	float ticks = 0;
	
	if (hEncoder->buffered_timer_value > hEncoder->timer_value)
	{
		ticks = hEncoder->timer_value + (UINT32_MAX - hEncoder->buffered_timer_value);
	} 
	else
	{
		ticks = hEncoder->timer_value - hEncoder->buffered_timer_value;
	}
	
	// Ticks equals approx. 0
	if(ticks < 0.9)
	{
		return;
	}
	
	float new_rpm = F_CPU/(ticks * PULSES_PER_ROTATION);
	
	// Crude low-pass (disturbance rejection) filter
	if(new_rpm < RPM_UPPER_DISCARD_LIMIT)
	{
		hEncoder->current_rps = new_rpm;
	}
	
	//usart_send((unsigned char*)&new_rpm, sizeof(float));
	
}

void setup_pid_timer(void)
{
	// Time for timer to tick once T1 = 1024/F_CPU (prescaler = 1024).
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
	SET_BIT(TCCR2B, CS20);
	SET_BIT(TCCR2B, CS21);
	SET_BIT(TCCR2B, CS22);
}

void setup_usart_receive(void)
{
	// USART0
	
	// Set baud rate to 115.2 kbps
	UBRR0 = (uint16_t)BAUD_RATE_UBBR_115_2_KBPS;
	
	// Enable receiver
	SET_BIT(UCSR0B, RXEN0);
	
	// DEBUG
	// Enable transmitter
	SET_BIT(UCSR0B, TXEN0);

	// Set frame format: 8data, 1 stop bit
	SET_BIT(UCSR0C, UCSZ00);
	SET_BIT(UCSR0C, UCSZ01);
	CLR_BIT(UCSR0B, UCSZ02);

	// Enable RX interrupt
	SET_BIT(UCSR0B, RXCIE0);


}

void setup_PID(void)
{
	PID_Init(&g_motor_1.pid, PID_KP, 0, PID_TI, 0, 95);
	PID_Init(&g_motor_2.pid, PID_KP, 0, PID_TI, 0, 95);
}

void do_advance_pids(void)
{
	float error = 0;
	float input = 0;
	
	// TODO: Robust sample time calculation from 1.0f/g_motor_1.hall_encoder.current_rps
	
	error = g_motor_1.setpoint - g_motor_1.hall_encoder.current_rps;
	input = PID_Advance(&g_motor_1.pid, SAMPLE_TIME_S, error);
	OCR0A = calculate_oc_value_from_dc((uint32_t)input);
	//OCR0A = 0xAA;
	
	error = g_motor_2.setpoint - g_motor_2.hall_encoder.current_rps;
	input = PID_Advance(&g_motor_2.pid, SAMPLE_TIME_S, error);
	OCR0B = calculate_oc_value_from_dc((uint32_t)input);
	
	//uint8_t a = calculate_oc_value_from_dc((uint32_t)input);
	//usart_send(&a, 1);
	
	//uint16_t input_int = (uint16_t)input;
	//usart_send((char*)&input_int, sizeof(uint16_t));
	
	usart_send((unsigned char*)&input, sizeof(float));
	//const float a = PID_KP;
	//usart_send((unsigned char*)&a, sizeof(float));
	
	//usart_send((unsigned char*)&g_motor_2.hall_encoder.current_rps, sizeof(float));
	//usart_send((unsigned char*)&error, sizeof(float));
	//usart_send((unsigned char*)&g_motor_1.setpoint, sizeof(float));
	
}

void do_parse_command(void)
{
	if(g_flag_command_running)
	{
		return;
	}	
	g_motor_1.setpoint = motor_on_rps;
	g_motor_2.setpoint = motor_on_rps;
	
	command_e command = COMMAND_UNKNOWN;
	
	// Weird USART bug that sets MSB bit to one
	g_command_buffer = g_command_buffer & 0x7F;
	
	switch(g_command_buffer)
	{
		case 'w':
		command = COMMAND_FORWARD;
		//do_blink_debug_led_times(1);
		break;
		case 's':
		command = COMMAND_BACKWARD;
		//do_blink_debug_led_times(2);
		break;
		case 'a':
		command = COMMAND_LEFT;
		//do_blink_debug_led_times(3);
		break;
		case 'd':
		command = COMMAND_RIGHT;
		//do_blink_debug_led_times(4);
		break;
		case 'x':
		command = COMMAND_STOP;
		break;
		default:
		command = COMMAND_UNKNOWN;
		//do_blink_debug_led_times(5);
		break;
	}

	set_motor_direction(command);
	
	g_flag_command_running = 1;
}

void do_on_command_complete(void)
{
	//set_motor_direction(COMMAND_STOP);
	
	g_motor_1.setpoint = 0;
	g_motor_2.setpoint = 0;
	
	OCR0A = 0xFF;
	OCR0B = 0xFF;
	
	PID_ClearAccumulatedValues(&g_motor_1.pid);
	PID_ClearAccumulatedValues(&g_motor_2.pid);
	//debug_led_off();
}


int main(void)
{
	
	// Setup
	setup_gpio_pins();
	
	configure_motor_pwm_timer();
	setup_pid_timer();
	configure_pulse_tick_timer();
	
	setup_usart_receive();
	
	setup_PID();
	
	enable_encoder_interrupt();
	enable_pulse_tick_timer();

	
	sei();

	enable_pid_timer();
	
	g_motor_1.hall_encoder.current_rps = 0;
	g_motor_2.hall_encoder.current_rps = 0;
	
    while (1) 
    {
		if(g_motor_1.hall_encoder.is_measurement_ready)
		{
			do_update_rpm(&g_motor_1.hall_encoder);
			g_motor_1.hall_encoder.is_measurement_ready = 0;
		}
		
		if(g_motor_2.hall_encoder.is_measurement_ready)
		{
			do_update_rpm(&g_motor_2.hall_encoder);
			g_motor_2.hall_encoder.is_measurement_ready = 0;
		}
		
		if(g_flag_pid && g_flag_command_running)
		{
			do_advance_pids();
			g_flag_pid = 0;
		}
		
		if(g_flag_command_received)
		{
			// TODO: do not set command running flag on unknown command
			do_parse_command();
			g_flag_command_received = 0;
		}
		
		if(g_flag_command_running && g_command_timer_pids >= command_duration_pids)
		{
			do_on_command_complete();
			g_flag_command_running = 0;
			g_command_timer_pids = 0;
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
	hall_encoder_do_save_timer_value(&g_motor_1.hall_encoder);
}

ISR(INT1_vect)
{
	hall_encoder_do_save_timer_value(&g_motor_2.hall_encoder);
}

ISR(TIMER1_OVF_vect)
{
	++pulse_tick_counter_high_nibble;
}

ISR(TIMER2_COMPA_vect)
{	
	// Increment command duration timer (which is measured in multiples of PID period)
	if (g_flag_command_running)
	{	
		++g_command_timer_pids;
	}
	
	// Signal the loop that PID is waiting for next calculation
	g_flag_pid = 1;
}

ISR(USART_RX_vect)
{
	if (IS_BIT_SET(UCSR0A, RXC0) && !g_flag_command_running)
	{
		g_command_buffer = UDR0;
		g_flag_command_received = 1;
	}
	else
	{
		// Read to clear RX flag
		volatile uint8_t discard = UDR0;
		UNUSED(discard);
	}
}

/*
 *	End Signal Handlers
 */


