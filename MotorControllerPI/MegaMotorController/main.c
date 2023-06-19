/*
 * MegaMotorControllerPI.c
 *
 * Created: 5/27/2023 7:17:33 PM
 * Author : KASO
 */ 

/*
	This code implements PI
	control for THREE 
	vnh2sp30 motor drivers
	on Arduino Mega board.
*/

#ifndef F_CPU
	#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h> // memset
#include <limits.h> // UINT32_MAX
#include "pid.h"
#include "utils_bitops.h"
#include "util_pindefs.h"
#include "stxetx_protocol.h"
#include "circular_buffer.h"
#include "cma.h"				


/*
 *	Begin Macros
 */

#ifndef NULL
#define NULL (void*)0
#endif

#define PRIVATE static


//////////////////////////////////////////////////////////////////////////
// ------ Pin Modes
//#define PIN_MODE_OUTPUT(REG, BIT) SET_BIT(REG, BIT)
//#define PIN_MODE_INPUT(REG, BIT)  CLR_BIT(REG, BIT)
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
// Flag that is used to declare that a function or a block
// of code uses some resource named X. If a resource is reused
// i.e. multiple occurrences of USES_RESOURCE(X) then the compiler
// throws error (multiple variable definitions);
// HAS TO BE USED IN GLOBAL SCOPE
// #define USES_RESOURCE(X) uint8_t uses_resource_##X##_flag = 1;

// To avoid "unused variable" warnings
#define UNUSED(X) (void)X;
//////////////////////////////////////////////////////////////////////////

// usart_send(...) sends data in little_endian byte order
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
	#define usart_send _usart_send_little_endian
	// Prevent compiler from warning us of unused function
	#define USE_IF_LITTLE_ENDIAN
	#define USE_IF_BIG_ENDIAN		__attribute__((unused))
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	#define usart_send _usart_send_big_endian
	// Prevent compiler from warning us of unused function
	#define USE_IF_LITTLE_ENDIAN	__attribute__((unused))
	#define USE_IF_BIG_ENDIAN		
#else
	#error "Unknown byte order: " __BYTE_ORDER__
#endif
	
	
#define SET_MOTOR_DIRECTION_FORWARD(M_IN_A, M_IN_B)		\
		do {											\
			WRITE_PIN(M_IN_A, 1);						\
			WRITE_PIN(M_IN_B, 0);						\
		 } while(0);
	 
#define SET_MOTOR_DIRECTION_BACKWARD(M_IN_A, M_IN_B)	\
		do {											\
			WRITE_PIN(M_IN_A, 0);						\
			WRITE_PIN(M_IN_B, 1);						\
		} while(0);

#define SET_MOTOR_DIRECTION_STOP(M_IN_A, M_IN_B)		\
		do {											\
			WRITE_PIN(M_IN_A, 0);						\
			WRITE_PIN(M_IN_B, 0);						\
		} while(0);

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
	cumulative_moving_average_t average_rps;
	uint8_t  is_measurement_ready;
} hall_encoder_t;

typedef struct {
	hall_encoder_t hall_encoder;
	pid_t pid;
	float setpoint;
} motor_t;

typedef enum {
	CMD_RCV_IDLE = 0,
	CMD_RCV_RECEIVING_IN_PROCESS = 1,
	CMD_RCV_ESCAPE_ACTIVE = 2,
	CMD_RCV_FULL_FRAME_READ = 3
} command_receiver_state_e;

/*
 *	End Type Definitions
 */


/*
 *	Begin Pin Definitions
 */

// Connection table for two VNH2SP30 motor controllers
// * Pin format: ATMEGA2560 (ARDUINO MEGA)


//	+===========+============+============+============+
//	|     /     |  MOTOR 1   |   MOTOR 2  |   MOTOR 3  |
//	+===========+============+============+============+
//	| IN A      | PA1(DIO23) | PA2(DIO24) | PA3(DIO25) |
//	+-----------+------------+------------+------------+
//	| IN B      | PC1(DIO36) | PC2(DIO35) | PC3(DIO34) |
//	+-----------+------------+------------+------------+
//	| PWM       | PH6(PWM9)  | PG5(PWM4)  | PB4(PWM10) |
//	+-----------+------------+------------+------------+
//	| HALL CH A | PE4(PWM2)  | PE5(PWM3)  | PD2(COM19) |
//	+-----------+------------+------------+------------+
//	| EXT INT   | INT4       | INT5       | INT2       |
//	+-----------+------------+------------+------------+
//	| OC REG    | OC2B       | OC0B       | OC2A       |
//	+-----------+------------+------------+------------+

#define MOTOR_1_IN_A	PIN_A1
#define MOTOR_1_IN_B	PIN_C1
#define MOTOR_1_PWM		PIN_H6
#define MOTOR_1_HCHA	PIN_E4
#define MOTOR_1_ENCODER_ISR	INT4_vect

#define MOTOR_2_IN_A	PIN_A2
#define MOTOR_2_IN_B	PIN_C2
#define MOTOR_2_PWM		PIN_G5
#define MOTOR_2_HCHA	PIN_E5
#define MOTOR_2_ENCODER_ISR	INT5_vect

#define MOTOR_3_IN_A	PIN_A3
#define MOTOR_3_IN_B	PIN_C3
#define MOTOR_3_PWM		PIN_B4
#define MOTOR_3_HCHA	PIN_D2
#define MOTOR_3_ENCODER_ISR	INT2_vect

#define ONBOARD_LED		PIN_B7

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

//#define PID_KP	(float)8.0f
//#define PID_TI	(float)100.0f

//#define PID_KP	(float)2.2049
//#define PID_TI	(float)128.8773

#define PID_KP	(float)4.0f
#define PID_TI	(float)128.8773f

//#define PID_KP	(float)50
//#define PID_TI	(float)10

//#define PID_KP	(float)30.0f
//#define PID_TI	(float)100000.0f


// PID Sampling frequency and period
#define SAMPLING_FREQUENCY 62.5f
#define SAMPLE_TIME_S (1.0f/SAMPLING_FREQUENCY)
//////////////////////////////////////////////////////////////////////////

// RPS (Revolutions Per Second) measurement filter
// Implemented as Exponential Moving Average filter
// with following difference equation
// x[n] = RPS_ALPHA * x[n-1] + (1 - RPS_ALPHA) * u[n]
// where x is the filtered value and u is measurement
#define RPS_ALPHA 0.5f

// IG32E-35K motor Hall encoder number of pulses per rotation
#define PULSES_PER_ROTATION 245

// Baud rate = 115.2kbps; U2X=0
//#define BAUD_RATE_UBBR_115_2_KBPS 8
#define BAUD_RATE_UBBR_9_6_KBPS 103
#define BAUD_RATE_UBBR_115_2_KBPS BAUD_RATE_UBBR_9_6_KBPS

// Defines maximum "believable" RPS value
// All RPS values above RPS_UPPER_DISCARD_LIMIT
// will not be saved as RPS values, instead old
// value will be held. (Crude low pass filter)
// (Disturbance rejection)
#define RPS_UPPER_DISCARD_LIMIT 10.0f

// Commands last for 3.008 seconds (T_PID = 1/62.5 s)
//const uint32_t command_duration_pids = 188;
PRIVATE volatile uint32_t g_target_command_duration__50ms_ticks = 3000/50;

// Incremented in task timer ISR. Represents duration of command
// in units of 50ms ticks.
PRIVATE volatile uint32_t g_command_duration_counter__50ms_ticks;

// Incremented in task timer ISR. Represents time since last
// odometry broadcast in units of 50ms ticks.
PRIVATE volatile uint32_t g_odometry_time_since_last_broadcast__50ms_ticks;

// Time between odometry broadcasts
PRIVATE const uint32_t odometry_broadcast_period__50ms_ticks = 500/50;

// Setpoint RPS (Revolutions Per Second) when motor is on
// PRIVATE const float motor_on_rps = 0.5f;

/*
 *	End Constants
 */


/*
 *	Start Global Variables
 */

// Updated by TIMER 1 overflow ISR to increment.
// Used to extend 16-bit TIMER 1 to 32 bits.
// Acts as higher nibble for 16-bit TIMER 1.
PRIVATE volatile uint16_t pulse_tick_counter_high_nibble = 0;

// Motors
PRIVATE motor_t g_motor_1;
PRIVATE motor_t g_motor_2;
PRIVATE motor_t g_motor_3;

// Signal to main loop that PID algorithm is
// ready to be executed.
PRIVATE volatile uint8_t g_flag_pid = 0;

// Flag that indicates that there is a command in command buffer,
// and that main loop should parse it.
PRIVATE volatile uint8_t g_flag_frame_awaits_decoding = 0;

// Flag that indicates command (in form of a stxetx_frame_t g_received_frame)
// is ready to be processed
PRIVATE volatile uint8_t g_flag_command_in_queue = 0;

// Size of receiving frame buffer used to serialize and send frame, in bytes
#define FRAME_ENCODE_BUFFER_SIZE 32
// Size of receiving frame buffer used to deserialize frame from receive buffer, in bytes
#define FRAME_DECODE_BUFFER_SIZE 32
// See STXETX stxetx_decode_n function description
#define PAYLOAD_BUFFER_SIZE 64

// Received and decoded STXETX frame.
PRIVATE volatile stxetx_frame_t g_received_frame;

// Frame buffer that holds *one full* received raw frame from UART.
PRIVATE volatile uint8_t g_frame_decode_buffer[FRAME_DECODE_BUFFER_SIZE] = {0};

// Number of bytes written to g_frame_decode_buffer
PRIVATE volatile uint8_t g_frame_decode_buffer_length = 0;

// Frame buffer that holds encoded frame to be sent over UART.
										  
PRIVATE volatile uint8_t g_frame_encode_buffer[FRAME_DECODE_BUFFER_SIZE] = {0};

// Number of bytes written in g_frame_encode_buffer
PRIVATE volatile uint8_t g_frame_encode_buffer_length = 0;

// Flag that indicated that current command is being executed.
PRIVATE volatile uint8_t g_flag_command_running = 0;

PRIVATE volatile command_receiver_state_e g_frame_receiver_state = CMD_RCV_IDLE;

#define RECEIVE_BUFFER_SIZE 64
																	 
												  
													   

PRIVATE volatile uint8_t g_receive_buffer_internal_[RECEIVE_BUFFER_SIZE] = {0};
PRIVATE volatile circular_buffer_t g_receive_buffer;


/*
 *	End Global Variables
 */

/*
 *	Start User Code Declaration
 */


/*
 *	End User Code Declaration
 */

PRIVATE void debug_led_on(void);
PRIVATE void debug_led_off(void);
PRIVATE void debug_led_toggle(void);
PRIVATE void do_blink_debug_led(void);
PRIVATE void do_blink_debug_led_times(int times);
PRIVATE void do_handle_fatal_error(void);
PRIVATE void do_handle_fatal_error_with_error_code(uint8_t error_code);
PRIVATE USE_IF_LITTLE_ENDIAN void _usart_send_little_endian(unsigned char* pData, int length);
PRIVATE USE_IF_BIG_ENDIAN	 void _usart_send_big_endian(unsigned char* pData, int length);
PRIVATE void usart_send_frame(stxetx_frame_t frame);
PRIVATE inline void hall_encoder_do_save_timer_value(hall_encoder_t* hEncoder);
PRIVATE void setup_gpio_pins(void);
PRIVATE void configure_motors_for_action(float motor_1_rps, float motor_2_rps, float motor_3_rps);
PRIVATE void configure_motor_pwm_timer(void);
PRIVATE inline uint8_t calculate_oc_value_from_dc(uint32_t duty_cycle);
PRIVATE void enable_encoder_interrupt(void);
PRIVATE void configure_pulse_tick_timer(void) ;
PRIVATE void enable_pulse_tick_timer(void);
PRIVATE void do_update_rps(hall_encoder_t* hEncoder);
PRIVATE void setup_task_timer(void);
PRIVATE void enable_task_timer(void);
PRIVATE void setup_pid_timer(void);
PRIVATE void enable_pid_timer(void);
PRIVATE void pause_pid_timer(void);
PRIVATE void resume_pid_timer(void);
PRIVATE void setup_usart_receive(void);
PRIVATE void setup_PID(void);
PRIVATE void do_advance_pids(void);
PRIVATE void on_received_msg_command(void);
PRIVATE void on_received_msg_stop(void);
PRIVATE void on_received_msg_unknown(void);
PRIVATE void do_execute_command(void);
PRIVATE void do_broadcast_average_rps(void);
PRIVATE void do_on_command_complete(void);
PRIVATE inline void command_buffer_write_byte(uint8_t data_byte);
PRIVATE inline void clear_command_buffer();
PRIVATE void do_on_command_byte_received(uint8_t byte_received);
PRIVATE void do_parse_received_frame(void);
PRIVATE void setup_motors(void);


/*
 *	Start User Code Implementation
 */

void debug_led_on(void)
{
	WRITE_PIN(ONBOARD_LED, 1);
}

void debug_led_off(void)
{
	WRITE_PIN(ONBOARD_LED, 0);
}

void debug_led_toggle(void)
{
	TOGGLE_PIN(ONBOARD_LED);
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

void do_handle_fatal_error_with_error_code(uint8_t error_code)
{
	const int flutter_half_period_ms = 50;
	const int flutter_duration_ms = 2000;
	const int error_code_flutter_delay_ms = 1000;
	// Signal fatal error with debug LED blinking
	while(1)
	{
		for (int i = 0; i < flutter_duration_ms/flutter_half_period_ms; i++)
		{
			debug_led_toggle();
			_delay_ms(50);
		}
		
		debug_led_off();
		
		_delay_ms(error_code_flutter_delay_ms);
		do_blink_debug_led_times(error_code);
		_delay_ms(error_code_flutter_delay_ms);
	}
}
										  
// Assumes little-endianness
void _usart_send_little_endian(unsigned char* pData, int length)
{
	
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

// Assumes big-endianness
void _usart_send_big_endian(unsigned char* pData, int length)
{
	
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


void usart_send_frame(stxetx_frame_t frame)
{
	memset(g_frame_encode_buffer, 0, FRAME_ENCODE_BUFFER_SIZE);

	uint8_t frame_size = 0;

	uint8_t ec = stxetx_encode_n(
		g_frame_encode_buffer,
		frame,
		frame_size,
		&frame_size
	);
	
	if (ec != STXETX_ERROR_NO_ERROR)				
	{
		do_handle_fatal_error_with_error_code(ec);
	}
	
	g_frame_encode_buffer_length = frame_size;
	usart_send(g_frame_encode_buffer, g_frame_encode_buffer_length);
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
	// DEBUG INBUILT-LED
	PIN_MODE_OUTPUT(ONBOARD_LED);
	
	// MOTOR 1
	PIN_MODE_OUTPUT(MOTOR_1_IN_A);
	PIN_MODE_OUTPUT(MOTOR_1_IN_B);
	PIN_MODE_OUTPUT(MOTOR_1_PWM);
	PIN_MODE_INPUT(MOTOR_1_HCHA);
		
	// MOTOR 2
	PIN_MODE_OUTPUT(MOTOR_2_IN_A);
	PIN_MODE_OUTPUT(MOTOR_2_IN_B);
	PIN_MODE_OUTPUT(MOTOR_2_PWM);
	PIN_MODE_INPUT(MOTOR_2_HCHA);
		
	// MOTOR 3
	PIN_MODE_OUTPUT(MOTOR_3_IN_A);
	PIN_MODE_OUTPUT(MOTOR_3_IN_B);
	PIN_MODE_OUTPUT(MOTOR_3_PWM);
	PIN_MODE_INPUT(MOTOR_3_HCHA);
		
	//// --- UART 0
		//// RX - PE0
		//PIN_MODE_INPUT(DDRE, DDE0);
		//// TX - PE1
		//PIN_MODE_OUTPUT(DDRE, DDE1);
}

// `motor_X_rps` is positive for positive rotations
// and negative for negative rotations. Zero stops motors.
void configure_motors_for_action(float motor_1_rps, float motor_2_rps, float motor_3_rps)
{
	// Clockwise direction = INA & ~INB
	
	// --- MOTOR 1 : IN A (PA1), IN B (PC1)
	// --- MOTOR 2 : IN A (PA2), IN B (PC2)
	// --- MOTOR 3 : IN A (PA3), IN B (PC3)
	
	
	/*============= MOTOR 1 =============*/
	if (motor_1_rps < 0)
	{
		g_motor_1.setpoint = fabs(motor_1_rps);
		SET_MOTOR_DIRECTION_BACKWARD(MOTOR_1_IN_A, MOTOR_1_IN_B);
	}
	else if(g_motor_1.setpoint > 0)
	{
		g_motor_1.setpoint = motor_1_rps;
		SET_MOTOR_DIRECTION_FORWARD(MOTOR_1_IN_A, MOTOR_1_IN_B);
	}
	else
	{
		g_motor_1.setpoint = 0;
		SET_MOTOR_DIRECTION_STOP(MOTOR_1_IN_A, MOTOR_1_IN_B);
	}
	
	/*============= MOTOR 2 =============*/
	if (g_motor_2.setpoint < 0)
	{
		g_motor_2.setpoint = fabs(motor_2_rps);
		SET_MOTOR_DIRECTION_BACKWARD(MOTOR_2_IN_A, MOTOR_2_IN_B);
	}
	else if(g_motor_2.setpoint > 0)
	{
		g_motor_2.setpoint = motor_2_rps;
		SET_MOTOR_DIRECTION_FORWARD(MOTOR_2_IN_A, MOTOR_2_IN_B);
	}
	else
	{
		g_motor_2.setpoint = 0;
		SET_MOTOR_DIRECTION_STOP(MOTOR_2_IN_A, MOTOR_2_IN_B);
	}
		
	/*============= MOTOR 3 =============*/
	if (g_motor_3.setpoint < 0)
	{
		g_motor_3.setpoint = fabs(motor_3_rps);
		SET_MOTOR_DIRECTION_BACKWARD(MOTOR_3_IN_A, MOTOR_3_IN_B);
	}
	else if(g_motor_3.setpoint > 0)
	{
		g_motor_3.setpoint = motor_3_rps;
		SET_MOTOR_DIRECTION_FORWARD(MOTOR_3_IN_A, MOTOR_3_IN_B);
	}
	else
	{
		g_motor_3.setpoint = 0;
		SET_MOTOR_DIRECTION_STOP(MOTOR_3_IN_A, MOTOR_3_IN_B);
	}

}

void configure_motor_pwm_timer(void)
{
	//////////////////////////////////////////////////////////////////////////
	// Three motors need three PWM channels (Output Compare Registers):
	// - 8 bit TIMER2 Channels A & B
	// - 8 bit TIMER0 Channel  B
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// -- Timer 0
	// Phase corrected PWM mode
	
	// Set phase corrected PWM mode of operation
	SET_BIT(TCCR0A, WGM00);
	CLR_BIT(TCCR0A, WGM01);
	CLR_BIT(TCCR0B, WGM02);
	
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
	CLR_BIT(TIMSK0, OCIE0B);
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// -- Timer 2
	// Phase corrected PWM mode
	
	// Set phase corrected PWM mode of operation
	SET_BIT(TCCR2A, WGM20);
	CLR_BIT(TCCR2A, WGM21);
	CLR_BIT(TCCR2B, WGM22);
	
	// Set on upcount, clear on downcount
	// Output Compare - A
	SET_BIT(TCCR2A, COM2A0);
	SET_BIT(TCCR2A, COM2A1);
	
	// Output Compare - B
	SET_BIT(TCCR2A, COM2B0);
	SET_BIT(TCCR2A, COM2B1);

	// Set clock prescaler to 1/8
	// Which gives 16MHz/(8 * 510) = 3.9 kHz PWM frequency
	CLR_BIT(TCCR2B, CS20);
	SET_BIT(TCCR2B, CS21);
	CLR_BIT(TCCR2B, CS22);
	
	// Disable interrupts
	// Overflow
	CLR_BIT(TIMSK2, TOV2);
	// Output Compare B Match
	CLR_BIT(TIMSK2, OCIE2A);
	// Output Compare B Match
	CLR_BIT(TIMSK2, OCIE2B);
	//////////////////////////////////////////////////////////////////////////
}

// input:
// - duty_cycle = [0..100] = Duty Cycle Percentage
// output:
// - Output compare value for:
//		* 8 bit timer
//		* Phase corrected PWM mode
//		* Set on upcount, clear on downcount
inline uint8_t calculate_oc_value_from_dc(uint32_t duty_cycle)
{
	// Set phase corrected PWM value from defined duty cycle
	// NOTE: Formula used is modified for integer division
	
	// Phase Corrected PWM (TOP = 0xFF)
	// with 'Set on upcount, reset on downcount'
	return (uint8_t)((255 * ((uint32_t)100 - duty_cycle))/100);
}


void enable_encoder_interrupt(void)
{
	// Enable Pullups (Disable pullup blockade)
	CLR_BIT(MCUCR, PUD);
	
	//////////////////////////////////////////////////////////////////////////
	// -- MOTOR 1
	
	// Enable pullup (IG32E Hall encoder docs require 1k external pullup)
	ENABLE_PULLUP(MOTOR_1_HCHA);
	
	// TODO: MACRO FOR INT
	// Set rising edge mode for INT4
	SET_BIT(EICRB, ISC40);
	SET_BIT(EICRB, ISC41);
	
	// Enable external interrupt
	 SET_BIT(EIMSK, INT4);
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// -- MOTOR 2
	
	// Enable pullup (IG32E Hall encoder docs require 1k external pullup)
	ENABLE_PULLUP(MOTOR_2_HCHA);
	
	// TODO: MACRO FOR INT
	// Set rising edge mode for INT5
	SET_BIT(EICRB, ISC50);
	SET_BIT(EICRB, ISC51);
	
	// Enable external interrupt
	SET_BIT(EIMSK, INT5);
	//////////////////////////////////////////////////////////////////////////
	
	//////////////////////////////////////////////////////////////////////////
	// -- MOTOR 3
	
	// Enable pullup (IG32E Hall encoder docs require 1k external pullup)
	ENABLE_PULLUP(MOTOR_3_HCHA);
	
	// TODO: MACRO FOR INT
	// Set rising edge mode for INT4
	SET_BIT(EICRA, ISC20);
	SET_BIT(EICRA, ISC21);
	
	// Enable external interrupt
	SET_BIT(EIMSK, INT2);
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

// Calculated new RPS value for 'hEncoder' Hall Encoder
// From saved timer values (which are saved in INT0/1 ISRs)
void do_update_rps(hall_encoder_t* hEncoder)
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
	
	float new_rps = F_CPU/(ticks * PULSES_PER_ROTATION);
	
	// Crude low-pass (disturbance rejection) filter
	if(new_rps < RPS_UPPER_DISCARD_LIMIT)
	{
		hEncoder->current_rps = RPS_ALPHA * hEncoder->current_rps + (1-RPS_ALPHA) * new_rps;
		//hEncoder->current_rps = new_rps;
		
		cma_feed_sample(&hEncoder->average_rps, new_rps);
	}
	
	//usart_send((unsigned char*)&new_rps, sizeof(float));
	
}

void setup_task_timer(void)
{
	// 16-bit TIMER3 is used to time tasks like:
	// Execution of commands, periodic sending of data, etc.
	
	// For this task, one tick of timer is set to 50ms and
	// each task should sample timer at the beginning and
	// measure duration in main loop.
	
	// Prescaler = 256, Compare Value = 3125 gives one tick period of 50ms
	const uint16_t timer_compare_value = 3125;
	
	// Set output compare register A value
	OCR3A = timer_compare_value;
	
	// Initialize timer value
	TCNT3 = 0;
	
	// Set OC CTC (TOP = OCR3A) mode of operation
	WRITE_BIT(TCCR3A, WGM30, 0);
	WRITE_BIT(TCCR3A, WGM31, 0);
	WRITE_BIT(TCCR3B, WGM32, 1);
	WRITE_BIT(TCCR3B, WGM33, 0);
}

void enable_task_timer(void)
{
	// Enable interrupt
	SET_BIT(TIMSK3, OCIE3A);
	
	// Enable clock (prescaler = 256)
	WRITE_BIT(TCCR3B, CS30, 0);
	WRITE_BIT(TCCR3B, CS31, 0);
	WRITE_BIT(TCCR3B, CS32, 1);
}

void setup_pid_timer(void)
{
	// Time for timer to tick once T1 = 1024/F_CPU (prescaler = 1024).
	// If we want timer to interrupt every T seconds, we must set compare register
	// to N ticks where N = T/T1 = F_CPU/F_S where F_S is the sampling (interrupt) frequency.

	//const uint8_t timer_compare_value = (uint8_t)(F_CPU/(1024 * SAMPLING_FREQUENCY));
	
	// 62.5 Hz
	const uint16_t timer_compare_value = 250;
	
	// Set output compare register A value
	OCR4A = timer_compare_value;
	
	// Initialize timer value
	TCNT4 = 0;
	
	// Set OC CTC (TOP = OCR4A) mode of operation
	WRITE_BIT(TCCR4A, WGM40, 0);
	WRITE_BIT(TCCR4A, WGM41, 0);
	WRITE_BIT(TCCR4B, WGM42, 1);
	WRITE_BIT(TCCR4B, WGM43, 0);
}

void enable_pid_timer(void)
{
	// Enable interrupt
	SET_BIT(TIMSK4, OCIE4A);
	
	// Enable clock (prescaler = 1024)
	WRITE_BIT(TCCR4B, CS40, 1);
	WRITE_BIT(TCCR4B, CS41, 0);
	WRITE_BIT(TCCR4B, CS42, 1);
}

void pause_pid_timer(void)
{
	// Disable interrupt
	CLR_BIT(TIMSK4, OCIE4A);
}

void resume_pid_timer(void)
{
	// Reset timer value
	TCNT4 = 0;
	
	// Enable interrupt
	SET_BIT(TIMSK4, OCIE4A);
}

void setup_usart_receive(void)
{
	// --- USART0
	
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

	// Setup receive buffer
	uint8_t error = CBuf_Init(&g_receive_buffer, g_receive_buffer_internal_, RECEIVE_BUFFER_SIZE);
	if(error != CBUF_ERROR_NO_ERROR)
	{
		do_handle_fatal_error_with_error_code(error);
	}
}

void setup_PID(void)
{
	// PID Controller for Motor 1
	PID_Init(
		&g_motor_1.pid,		/* pid_t Handle				*/
		PID_KP,				/* Kp - Proportional Term	*/
		0,					/* Td - Derivative Term		*/
		PID_TI,				/* Ti - Integral Term		*/
		0,					/* Minimum PID Output Value */
		95					/* Maximum PID Output Value */
	);
	
	// PID Controller for Motor 2
	PID_Init(
		&g_motor_2.pid,		/* pid_t Handle				*/
		PID_KP,				/* Kp - Proportional Term	*/
		0,					/* Td - Derivative Term		*/
		PID_TI,				/* Ti - Integral Term		*/
		0,					/* Minimum PID Output Value */
		95					/* Maximum PID Output Value */
	);
	
	// PID Controller for Motor 3
	PID_Init(
		&g_motor_3.pid,		/* pid_t Handle				*/
		PID_KP,				/* Kp - Proportional Term	*/
		0,					/* Td - Derivative Term		*/
		PID_TI,				/* Ti - Integral Term		*/
		0,					/* Minimum PID Output Value */
		95					/* Maximum PID Output Value */
	);
}

void do_advance_pids(void)
{
	float error_1 = 0;
	float input_1 = 0;
	
	float error_2 = 0;
	float input_2 = 0;
	
	float error_3 = 0;
	float input_3 = 0;
	
	// TODO: Robust sample time calculation
	
	error_1 = g_motor_1.setpoint - g_motor_1.hall_encoder.current_rps;
	input_1 = PID_Advance(&g_motor_1.pid, SAMPLE_TIME_S, error_1);
	OCR2B = calculate_oc_value_from_dc((uint32_t)input_1);
	
	error_2 = g_motor_2.setpoint - g_motor_2.hall_encoder.current_rps;
	input_2 = PID_Advance(&g_motor_2.pid, SAMPLE_TIME_S, error_2);
	OCR0B = calculate_oc_value_from_dc((uint32_t)input_2);
	
	error_3 = g_motor_3.setpoint - g_motor_3.hall_encoder.current_rps;
	input_3 = PID_Advance(&g_motor_3.pid, SAMPLE_TIME_S, error_3);
	OCR2A = calculate_oc_value_from_dc((uint32_t)input_3);
	
	// Reset PWM timer for correct transition between duty cycles
	TCNT0 = 0;
	TCNT2 = 0;
	
	//////////////////////////////////////////////////////////////////////////
	// -------------- DEBUG
	
	//usart_send(&g_motor_2.pid.current_error, sizeof(float));
	
	
	//if (PID_CheckError(&g_motor_1.pid, NULL) || PID_CheckError(&g_motor_2.pid, NULL))
	//{
		//do_handle_fatal_error();
	//}
	
	//uint8_t a = calculate_oc_value_from_dc((uint32_t)input);
	//usart_send(&a, 1);
	
	
	//usart_send((unsigned char*)&input_1, sizeof(float));
	//const float a = SAMPLE_TIME_S;
	//usart_send((unsigned char*)&a, sizeof(float));
	
	//usart_send((unsigned char*)&g_motor_3.hall_encoder.current_rps, sizeof(float));
	//usart_send((unsigned char*)&error_2, sizeof(float));
	//usart_send((unsigned char*)&g_motor_2.setpoint, sizeof(float));
	//////////////////////////////////////////////////////////////////////////
	
}

void on_received_msg_command(void)
{		
	uint32_t command_duration_ms = 0;
	
	if (g_received_frame.len_bytes < 3 * sizeof(float) + sizeof(uint32_t))
	{
		// TODO: Add general errors
		// TODO: Add info messages
		do_handle_fatal_error();
	}
	
	float motor_1_rps_setpoint = 0;
	float motor_2_rps_setpoint = 0;
	float motor_3_rps_setpoint = 0;
	
	memcpy((void*)&motor_1_rps_setpoint,  (const void*)(g_received_frame.p_payload +  0), sizeof(float));
	memcpy((void*)&motor_2_rps_setpoint,  (const void*)(g_received_frame.p_payload +  4), sizeof(float));
	memcpy((void*)&motor_3_rps_setpoint,  (const void*)(g_received_frame.p_payload +  8), sizeof(float));
	memcpy((void*)&command_duration_ms,   (const void*)(g_received_frame.p_payload + 12), sizeof(uint32_t));
	
	// UINT32_MAX will be used in place of `float`'s INFINITY
	// i.e. it will denote command that never stops, in this case
	// a command that runs for UINT32_MAX * 50ms = 214748364750 ms (approx. 2485 days)
	if (command_duration_ms == UINT32_MAX)
	{
		g_target_command_duration__50ms_ticks = UINT32_MAX;
	}
	else
	{
		// Round command duration to nearest multiple of 50ms
		g_target_command_duration__50ms_ticks = command_duration_ms / 50;
		if((command_duration_ms % 50) > (50/2))
		{
			g_target_command_duration__50ms_ticks += 1;
		}	
	}

	cma_reset(&g_motor_1.hall_encoder.average_rps);
	cma_reset(&g_motor_2.hall_encoder.average_rps);
	cma_reset(&g_motor_3.hall_encoder.average_rps);
	
	configure_motors_for_action(motor_1_rps_setpoint, motor_2_rps_setpoint, motor_3_rps_setpoint);
	
	g_command_duration_counter__50ms_ticks = 0;
	g_odometry_time_since_last_broadcast__50ms_ticks = 0;
	
	g_flag_command_running = 1;
	resume_pid_timer();
}

void on_received_msg_stop(void)
{
	g_flag_command_running = 0;
	do_on_command_complete();
}

void on_received_msg_unknown(void)
{
	do_handle_fatal_error();
}


void do_execute_command(void)
{		
	//// Ignore new command while current is being executed
	//if(g_flag_command_running)
	//{
		//return;
	//}	
	
	switch(g_received_frame.msg_type)
	{
		case MSG_TYPE_COMMAND:
			on_received_msg_command();
		break;
		
		case MSG_TYPE_STOP:
			on_received_msg_stop();
		break;
		
		default:
			on_received_msg_unknown();
		break;
	}
}

void do_broadcast_average_rps(void)
{
	// TODO: timestamp?
	
	const float motor_1_rps = cma_get_value(&g_motor_1.hall_encoder.average_rps);
	const float motor_2_rps = cma_get_value(&g_motor_2.hall_encoder.average_rps);
	const float motor_3_rps = cma_get_value(&g_motor_3.hall_encoder.average_rps);
	
	stxetx_frame_t frame;
	
	stxetx_init_empty_frame(&frame);
	
	frame.msg_type = MSG_TYPE_ODOMETRY;
	
	// const uint8_t payload_size = 3 * sizeof(float);
	
	//uint8_t p_payload[payload_size] = {
	uint8_t p_payload[12] = {0};
		
	memcpy(p_payload + 0, (const void*)&motor_1_rps, sizeof(float));
	memcpy(p_payload + 4, (const void*)&motor_2_rps, sizeof(float));
	memcpy(p_payload + 8, (const void*)&motor_3_rps, sizeof(float));
	
	//uint8_t ec = stxetx_add_payload(&frame, p_payload, payload_size);
	uint8_t ec = stxetx_add_payload(&frame, p_payload, 12);
	
	if (ec != STXETX_ERROR_NO_ERROR)
	{
		do_handle_fatal_error_with_error_code(ec);
	}
	
	usart_send_frame(frame);
}


void do_on_command_complete(void)
{
	pause_pid_timer();
	
	// Stop motors by setting their speed to 0.
	configure_motors_for_action(0, 0, 0);
	
	cma_reset(&g_motor_1.hall_encoder.average_rps);
	cma_reset(&g_motor_2.hall_encoder.average_rps);
	cma_reset(&g_motor_3.hall_encoder.average_rps);
	
	// Clamp PWM to 0% duty cycle
	OCR0B = 0xFF;
	OCR2A = 0xFF;
	OCR2B = 0xFF;
	
	PID_ClearAccumulatedValues(&g_motor_1.pid);
	PID_ClearAccumulatedValues(&g_motor_2.pid);
	PID_ClearAccumulatedValues(&g_motor_3.pid);
	//debug_led_off();
	
	// Send `FINISHED` message
	stxetx_frame_t frame;
	stxetx_init_empty_frame(&frame);
	frame.msg_type = MSG_TYPE_FINISHED;
	usart_send_frame(frame);
}


inline void command_buffer_write_byte(uint8_t data_byte)
{
	g_frame_decode_buffer[g_frame_decode_buffer_length++] = data_byte;
}

inline void clear_command_buffer()
{
	g_frame_decode_buffer_length = 0;
	memset(g_frame_decode_buffer, 0, FRAME_DECODE_BUFFER_SIZE);
}


void do_on_command_byte_received(uint8_t byte_received)
{
	switch (g_frame_receiver_state)
	{
		case CMD_RCV_IDLE:
			if(stxetx_is_character_frame_start_delimiter(byte_received))
			{
				clear_command_buffer();
				command_buffer_write_byte(byte_received);
				g_frame_receiver_state = CMD_RCV_RECEIVING_IN_PROCESS;
			}
			else
			{
				// Error state:
				// Invalid character received, expected STX character
				// TODO Write error data to EEPROM
				goto error;
			}
		break;
		case CMD_RCV_RECEIVING_IN_PROCESS:
			if (stxetx_is_character_frame_end_delimiter(byte_received))
			{
				command_buffer_write_byte(byte_received);
				g_frame_receiver_state = CMD_RCV_FULL_FRAME_READ;
				g_flag_frame_awaits_decoding = 1;
			}
			else if(stxetx_is_character_escape(byte_received))
			{
				command_buffer_write_byte(byte_received);
				g_frame_receiver_state = CMD_RCV_ESCAPE_ACTIVE;
			}
			else if(stxetx_is_character_frame_start_delimiter(byte_received))
			{
				// Error state:
				// Detected unescaped STX character in middle of a frame
				goto error;
			}
			else if(g_frame_decode_buffer_length < FRAME_DECODE_BUFFER_SIZE)
			{
				command_buffer_write_byte(byte_received);
			}
			else
			{
				// Error state:
				// Buffer Overflowed
				goto error;
			}
		break;
		case CMD_RCV_ESCAPE_ACTIVE:
			if (g_frame_decode_buffer_length < FRAME_DECODE_BUFFER_SIZE)
			{
				command_buffer_write_byte(byte_received);
				g_frame_receiver_state = CMD_RCV_RECEIVING_IN_PROCESS;
			}
			else
			{
				// Error state:
				// Buffer Overflowed
				goto error;
			}
		break;
		case CMD_RCV_FULL_FRAME_READ:
			// Always an error case
			// Should be reset externally after reading command
			goto error;
		default:
		// Error state:
		// Unknown State
		goto error;
	}
	
	return;
	
error:
	g_frame_receiver_state = CMD_RCV_IDLE;
	clear_command_buffer();
	return;
}

void do_parse_received_frame(void)
{
	uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE] = {0};
	stxetx_init_empty_frame(&g_received_frame);
	
	uint8_t status = stxetx_decode_n(
		g_frame_decode_buffer,
		&g_received_frame,
		g_frame_decode_buffer_length,
		payload_buffer,
		PAYLOAD_BUFFER_SIZE
	);
	
	if (status != STXETX_ERROR_NO_ERROR)
	{
		// Error State:
		// TODO: Write to EEPROM
		do_handle_fatal_error_with_error_code(status);
		return;
	}
}								  

void setup_motors(void)
{
	g_motor_1.hall_encoder.current_rps = 0;
	g_motor_2.hall_encoder.current_rps = 0;
	g_motor_3.hall_encoder.current_rps = 0;
	
	cma_init(&g_motor_1.hall_encoder.average_rps);
	cma_init(&g_motor_2.hall_encoder.average_rps);
	cma_init(&g_motor_3.hall_encoder.average_rps);
}

int main(void)
{
	
	// Setup
	setup_gpio_pins();
	
	configure_motor_pwm_timer();
	setup_pid_timer();
	setup_task_timer();
	configure_pulse_tick_timer();
	
	setup_usart_receive();
	
	setup_PID();
	
	enable_encoder_interrupt();
	enable_pulse_tick_timer();
	
	setup_motors();
	
	sei();

	enable_pid_timer();
	enable_task_timer();
	
	g_frame_receiver_state = CMD_RCV_IDLE;
	
    while (1) 
    {
		if(g_motor_1.hall_encoder.is_measurement_ready)
		{
			do_update_rps(&g_motor_1.hall_encoder);
			g_motor_1.hall_encoder.is_measurement_ready = 0;
		}
		
		if(g_motor_2.hall_encoder.is_measurement_ready)
		{
			do_update_rps(&g_motor_2.hall_encoder);
			g_motor_2.hall_encoder.is_measurement_ready = 0;
		}
		
		if(g_motor_3.hall_encoder.is_measurement_ready)
		{
			do_update_rps(&g_motor_3.hall_encoder);
			g_motor_3.hall_encoder.is_measurement_ready = 0;
		}
		
		if(g_flag_pid && g_flag_command_running)
		{
			do_advance_pids();
			g_flag_pid = 0;
		}
		
		if(CBuf_AvailableForRead(g_receive_buffer) && g_frame_receiver_state != CMD_RCV_FULL_FRAME_READ)
		{
			uint8_t value;
			uint8_t error = CBuf_Read(&g_receive_buffer, &value);
			if (error == STXETX_ERROR_NO_ERROR)
			{
				do_on_command_byte_received(value);
			}
			else
			{
				do_handle_fatal_error_with_error_code(error);
			}
		}

		if(g_flag_frame_awaits_decoding && !g_flag_command_in_queue)
		{	
			do_parse_received_frame();
			g_flag_frame_awaits_decoding = 0;
			g_flag_command_in_queue = 1;
			g_frame_receiver_state = CMD_RCV_IDLE;
		}
		
		if (g_flag_command_in_queue)
		{
			do_execute_command();
			g_flag_command_in_queue = 0;
		}
		
		if(g_flag_command_running
			&& g_odometry_time_since_last_broadcast__50ms_ticks >= odometry_broadcast_period__50ms_ticks)
		{
			do_broadcast_average_rps();
			g_odometry_time_since_last_broadcast__50ms_ticks = 0;
		}
		
		if(g_flag_command_running
			&& g_command_duration_counter__50ms_ticks >= g_target_command_duration__50ms_ticks)
		{
			do_on_command_complete();
			g_flag_command_running = 0;
			g_command_duration_counter__50ms_ticks = 0;
		}
    }
}

/*
 *	End User Code Implementation
 */

/*
 *	Start Signal Handlers
 */

ISR(INT4_vect)
{
	hall_encoder_do_save_timer_value(&g_motor_1.hall_encoder);
}

ISR(INT5_vect)
{
	hall_encoder_do_save_timer_value(&g_motor_2.hall_encoder);
}

ISR(INT2_vect)
{
	hall_encoder_do_save_timer_value(&g_motor_3.hall_encoder);
}

ISR(TIMER1_OVF_vect)
{
	++pulse_tick_counter_high_nibble;
}

ISR(TIMER4_COMPA_vect)
{	
	// Signal the loop that PID is waiting for next calculation
	if (g_flag_command_running)
	{
		g_flag_pid = 1;
	}
}

ISR(TIMER3_COMPA_vect)
{
	// Increment task timers
	
	// ONLY If command is being executed:
	if (g_flag_command_running)
	{
		// Increment command duration counter
		++g_command_duration_counter__50ms_ticks;
		
		// Increment odometry counter
		++g_odometry_time_since_last_broadcast__50ms_ticks;
	}
	

}

ISR(USART0_RX_vect)
{
	if (
		IS_BIT_SET(UCSR0A, RXC0)
		&& CBuf_AvailableForWrite(g_receive_buffer)
		/* && !g_flag_command_running */
		)
	{
		uint8_t byte_received = UDR0;
		
		// For some reason, MSB is always 1
		//byte_received &= 0x7F;
		
		uint8_t error = CBuf_Write(&g_receive_buffer, byte_received);
		if (error != CBUF_ERROR_NO_ERROR)
		{
			do_handle_fatal_error_with_error_code(error);
		}
		
		// // Loopback
		//while (!IS_BIT_SET(UCSR0A, UDRE0))
		//{
			//;
		//}
		//UDR0 = byte_received;
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


