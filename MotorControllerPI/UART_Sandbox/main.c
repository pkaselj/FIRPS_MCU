/*
 * DuoMotorControllerPI.c
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
#include "utils_bitops.h"
#include "util_pindefs.h"
#include "stxetx_protocol.h"
#include "circular_buffer.h"


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
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
	#define usart_send _usart_send_big_endian
#else
	#error "Unknown byte order: " __BYTE_ORDER__
#endif
	

/*
 *	End Macros
 */

/*
 *	Begin Type Definitions
 */

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

#define BAUD_RATE_UBBR_9_6_KBPS 103
// Baud rate = 115.2kbps; U2X=0
//#define BAUD_RATE_UBBR_115_2_KBPS 8
#define BAUD_RATE_UBBR_115_2_KBPS BAUD_RATE_UBBR_9_6_KBPS

// Commands last for 3.008 seconds (T_PID = 1/62.5 s)
//const uint32_t command_duration_pids = 188;
PRIVATE const uint32_t command_duration_pids = 2*188;


/*
 *	End Constants
 */


/*
 *	Start Global Variables
 */


// Incremented on each PID execution (when PID timer triggers)
// Used to keep track of duration of current command (measured in
// multiples of PID intervals, see 'command_duration_pids')
PRIVATE volatile uint32_t g_command_timer_pids;

// Flag that indicates that there is a command in command buffer,
// and that main loop should parse it.
PRIVATE volatile uint8_t g_flag_frame_awaits_decoding = 0;

// Size of receiving command buffer in bytes
#define COMMAND_BUFFER_SIZE 32
// See STXETX stxetx_decode_n function description
#define PAYLOAD_BUFFER_SIZE 64

// Command buffer that holds received raw command from UART.
// Will be extended to an array in future.
PRIVATE volatile uint8_t g_frame_buffer[COMMAND_BUFFER_SIZE] = {0};

// Number of bytes written to g_frame_buffer
PRIVATE volatile uint8_t g_frame_buffer_length = 0;

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
PRIVATE void _usart_send_little_endian(unsigned char* pData, int length)
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
PRIVATE void _usart_send_big_endian(unsigned char* pData, int length)
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

PRIVATE void setup_pid_timer(void)
{
	// If we want timer to interrupt every T seconds, we must set compare register
	// to N ticks where N = T/T1 = F_CPU/F_S where F_S is the sampling (interrupt) frequency.

	//const uint8_t timer_compare_value = (uint8_t)(F_CPU/(1024 * SAMPLING_FREQUENCY));
	
	// 62.5 Hz
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

PRIVATE void enable_pid_timer(void)
{
	// Enable interrupt
	SET_BIT(TIMSK2, OCIE2A);
	
	// Enable clock (prescaler = 1024)
	SET_BIT(TCCR2B, CS20);
	SET_BIT(TCCR2B, CS21);
	SET_BIT(TCCR2B, CS22);
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

PRIVATE void do_parse_command(void)
{		
	// Ignore new command while current is being executed
	if(g_flag_command_running)
	{
		return;
	}	
	
	command_e command = COMMAND_UNKNOWN;
	
	uint8_t payload_buffer[PAYLOAD_BUFFER_SIZE] = {0};
	stxetx_frame_t command_frame;
	stxetx_init_empty_frame(&command_frame);
	uint8_t status = stxetx_decode_n(
		g_frame_buffer,
		&command_frame,
		g_frame_buffer_length,
		payload_buffer,
		PAYLOAD_BUFFER_SIZE
	);
	
	if (status != STXETX_ERROR_NO_ERROR)
	{
		stxetx_frame_t frame;
		stxetx_init_empty_frame(&frame);
		frame.msg_type = 1;
		frame.flags = 0;
		frame.checksum = 0;
		stxetx_add_payload(&frame, g_frame_buffer, g_frame_buffer_length);
		
		uint8_t p_data[64] = {0};
		uint32_t bytes_written = 0;
		stxetx_encode_n(p_data, frame, 64, &bytes_written);
		usart_send(p_data, bytes_written);
		// Error State:
		// TODO: Write to EEPROM
		do_handle_fatal_error_with_error_code(status);
		return;
	}
	
	do_blink_debug_led_times(command_frame.msg_type);
	
	//switch(command_frame.msg_type)
	//{
		//case MSG_TYPE_GO_FORWARD:
		//command = COMMAND_FORWARD;
		//do_blink_debug_led_times(1);
		//break;
		//case MSG_TYPE_GO_BACKWARD:
		//command = COMMAND_BACKWARD;
		//do_blink_debug_led_times(2);
		//break;
		//case MSG_TYPE_ROTATE_LEFT:
		//command = COMMAND_LEFT;
		//do_blink_debug_led_times(3);
		//break;
		//case MSG_TYPE_ROTATE_RIGHT:
		//command = COMMAND_RIGHT;
		//do_blink_debug_led_times(4);
		//break;
		//case MSG_TYPE_STOP:
		//command = COMMAND_STOP;
		//do_blink_debug_led_times(5);
		//break;
		//default:
		//command = COMMAND_UNKNOWN;
		//do_blink_debug_led_times(6);
		//break;
	//}
	//
	//g_flag_command_running = 1;
}

PRIVATE void do_on_command_complete(void)
{
	//g_flag_command_running = 0;
}

PRIVATE inline void command_buffer_write_byte(uint8_t data_byte)
{
	g_frame_buffer[g_frame_buffer_length++] = data_byte;
}

PRIVATE void clear_command_buffer()
{
	g_frame_buffer_length = 0;
	memset(g_frame_buffer, 0, COMMAND_BUFFER_SIZE);
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
			else if(g_frame_buffer_length < COMMAND_BUFFER_SIZE)
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
			if (g_frame_buffer_length < COMMAND_BUFFER_SIZE)
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


int main(void)
{
	
	// Setup
	setup_pid_timer();
	setup_usart_receive();
	
	// RX
	PIN_MODE_INPUT(PIN_D0);
	// TX
	PIN_MODE_OUTPUT(PIN_D1);
	// DEBUG LED
	PIN_MODE_OUTPUT(ONBOARD_LED);

	sei();

	enable_pid_timer();
	
	debug_led_off();
	
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

		if(g_flag_frame_awaits_decoding)
		{
			// TODO: do not set command running flag on unknown command
			do_parse_command();
			g_flag_frame_awaits_decoding = 0;
			g_frame_receiver_state = CMD_RCV_IDLE;
		}
		
		if(g_flag_command_running /* && g_command_timer_pids >= command_duration_pids */)
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

ISR(TIMER2_COMPA_vect)
{	
	// Increment command duration timer (which is measured in multiples of PID period)
	if (g_flag_command_running)
	{	
		++g_command_timer_pids;
	}
}

ISR(USART_RX_vect)
{
	if (IS_BIT_SET(UCSR0A, RXC0) && !g_flag_command_running)
	{
		//g_frame_buffer = UDR0;
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


