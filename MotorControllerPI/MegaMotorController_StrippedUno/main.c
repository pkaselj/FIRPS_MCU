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
	PRIVATE void _usart_send_little_endian(unsigned char* pData, int length);
	__attribute__((unused)) void _usart_send_big_endian(unsigned char* pData, int length);
	
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__

	#define usart_send _usart_send_big_endian
	
	// Prevent compiler from warning us of unused function
	__attribute__((unused)) void _usart_send_little_endian(unsigned char* pData, int length);
	PRIVATE void _usart_send_big_endian(unsigned char* pData, int length);
	
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

#define ONBOARD_LED		PIN_B5

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


/*
 *	Start User Code Implementation
 */

PRIVATE void debug_led_on(void)
{
	WRITE_PIN(ONBOARD_LED, 1);
}

PRIVATE void debug_led_off(void)
{
	WRITE_PIN(ONBOARD_LED, 0);
}

PRIVATE void debug_led_toggle(void)
{
	TOGGLE_PIN(ONBOARD_LED);
}

PRIVATE void do_blink_debug_led(void)
{
	debug_led_on();
	_delay_ms(500);
	debug_led_off();
	_delay_ms(500);
}

PRIVATE void do_blink_debug_led_times(int times)
{
	int i;
	for (i = 0; i < times; ++i)
	{
		do_blink_debug_led();
	}
}

PRIVATE void do_handle_fatal_error(void)
{
	// Signal fatal error with debug LED blinking
	while(1)
	{
		debug_led_toggle();
		_delay_ms(50);
	}
}

PRIVATE void do_handle_fatal_error_with_error_code(uint8_t error_code)
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


PRIVATE void usart_send_frame(stxetx_frame_t frame)
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


PRIVATE void setup_task_timer(void)
{
	// 16-bit TIMER3 is used to time tasks like:
	// Execution of commands, periodic sending of data, etc.
	
	// For this task, one tick of timer is set to 50ms and
	// each task should sample timer at the beginning and
	// measure duration in main loop.
	
	// Prescaler = 256, Compare Value = 3125 gives one tick period of 50ms
	const uint16_t timer_compare_value = 3125;
	
	// Set output compare register A value
	OCR1A = timer_compare_value;
	
	// Initialize timer value
	TCNT1 = 0;
	
	// Set OC CTC (TOP = OCR3A) mode of operation
	WRITE_BIT(TCCR1A, WGM10, 0);
	WRITE_BIT(TCCR1A, WGM11, 0);
	WRITE_BIT(TCCR1B, WGM12, 1);
	WRITE_BIT(TCCR1B, WGM13, 0);
}

PRIVATE void enable_task_timer(void)
{
	// Enable interrupt
	SET_BIT(TIMSK1, OCIE1A);
	
	// Enable clock (prescaler = 256)
	WRITE_BIT(TCCR1B, CS10, 0);
	WRITE_BIT(TCCR1B, CS11, 0);
	WRITE_BIT(TCCR1B, CS12, 1);
}


PRIVATE void setup_usart_receive(void)
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

PRIVATE void on_received_msg_command(void)
{		
	uint32_t command_duration_ms = 0;
	
	if (g_received_frame.len_bytes < 3 * sizeof(float) + sizeof(uint32_t))
	{
		// TODO: Add general errors
		// TODO: Add info messages
		do_handle_fatal_error();
	}
	
	memcpy((void*)&g_motor_1.setpoint,  (const void*)(g_received_frame.p_payload +  0), sizeof(float));
	memcpy((void*)&g_motor_2.setpoint,  (const void*)(g_received_frame.p_payload +  4), sizeof(float));
	memcpy((void*)&g_motor_3.setpoint,  (const void*)(g_received_frame.p_payload +  8), sizeof(float));
	memcpy((void*)&command_duration_ms, (const void*)(g_received_frame.p_payload + 12), sizeof(uint32_t));
	
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

	// TODO:
	// Set motor directions, fabs(setpoint)
	// Resume PID timer
	// Reset CMA
	
	// DEBUG
	cma_reset(&g_motor_1.hall_encoder.average_rps);
	cma_reset(&g_motor_2.hall_encoder.average_rps);
	cma_reset(&g_motor_3.hall_encoder.average_rps);
	
	g_motor_1.hall_encoder.current_rps = g_motor_1.setpoint;
	g_motor_2.hall_encoder.current_rps = g_motor_2.setpoint;
	g_motor_3.hall_encoder.current_rps = g_motor_3.setpoint;

	
	g_command_duration_counter__50ms_ticks = 0;
	g_odometry_time_since_last_broadcast__50ms_ticks = 0;
	
	g_flag_command_running = 1;
}

PRIVATE void on_received_msg_stop(void)
{
	
	g_flag_command_running = 0;
	
	usart_send_frame(g_received_frame);
	
	// TODO: Stop motors
}

PRIVATE void on_received_msg_unknown(void)
{
	do_handle_fatal_error();
}


PRIVATE void do_execute_command(void)
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

PRIVATE void do_broadcast_average_rps(void)
{
	// DEBUG
	
	g_motor_1.hall_encoder.current_rps += 1;
	g_motor_2.hall_encoder.current_rps += 1;
	g_motor_3.hall_encoder.current_rps += 1;
	
	cma_feed_sample(&g_motor_1.hall_encoder.average_rps, g_motor_1.hall_encoder.current_rps);
	cma_feed_sample(&g_motor_2.hall_encoder.average_rps, g_motor_2.hall_encoder.current_rps);
	cma_feed_sample(&g_motor_3.hall_encoder.average_rps, g_motor_3.hall_encoder.current_rps);
	
	// TODO: timestamp?
	
	stxetx_frame_t frame;
	
	stxetx_init_empty_frame(&frame);
	
	frame.msg_type = MSG_TYPE_ODOMETRY;
	
	// const uint8_t payload_size = 3 * sizeof(float);
	
	//uint8_t p_payload[payload_size] = {
	uint8_t p_payload[12] = {0};
		
	memcpy(p_payload + 0, (const void*)&g_motor_1.hall_encoder.average_rps.cma_value , sizeof(float));
	memcpy(p_payload + 4, (const void*)&g_motor_2.hall_encoder.average_rps.cma_value , sizeof(float));
	memcpy(p_payload + 8, (const void*)&g_motor_3.hall_encoder.average_rps.cma_value , sizeof(float));
	
	////uint8_t ec = stxetx_add_payload(&frame, p_payload, payload_size);
	uint8_t ec = stxetx_add_payload(&frame, p_payload, 12);
	
	if (ec != STXETX_ERROR_NO_ERROR)
	{
		do_handle_fatal_error_with_error_code(ec);
	}
	
	usart_send_frame(frame);
}

PRIVATE void do_on_command_complete(void)
{
	//set_motor_direction(COMMAND_STOP);

	stxetx_frame_t frame;
	stxetx_init_empty_frame(&frame);
	frame.msg_type = MSG_TYPE_FINISHED;
	usart_send_frame(frame);

	
	//debug_led_off();
}

PRIVATE inline void command_buffer_write_byte(uint8_t data_byte)
{
	g_frame_decode_buffer[g_frame_decode_buffer_length++] = data_byte;
}

PRIVATE inline void clear_command_buffer()
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

PRIVATE void do_parse_received_frame(void)
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

int main(void)
{
	
	// Setup
	
	PIN_MODE_OUTPUT(PIN_B5);
	
	setup_task_timer();
	
	setup_usart_receive();
	
	sei();
	
	enable_task_timer();
	
	g_frame_receiver_state = CMD_RCV_IDLE;
	
    while (1) 
    {
	
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


ISR(TIMER1_COMPA_vect)
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

ISR(USART_RX_vect)
{
	if (
		IS_BIT_SET(UCSR0A, RXC0)
		&& CBuf_AvailableForWrite(g_receive_buffer)
		/* && !g_flag_command_running */
		)
	{
		//g_frame_decode_buffer = UDR0;
		//g_flag_frame_awaits_decoding = 1;
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




