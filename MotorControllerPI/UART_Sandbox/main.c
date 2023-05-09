/*
 * UART_Sandbox.c
 *
 * Created: 5/9/2023 5:21:59 PM
 * Author : KASO
 */ 

#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//////////////////////////////////////////////////////////////////////////
// -- Bitwise Operators
#define SET_BIT(REG, BIT) REG |= _BV(BIT)
#define CLR_BIT(REG, BIT) REG &= ~_BV(BIT)
#define TGL_BIT(REG, BIT) REG ^= _BV(BIT)

#define WRITE_BIT(REG, BIT, VAL) WRITE_BIT_##VAL(REG, BIT)
#define WRITE_BIT_1(REG, BIT) SET_BIT(REG, BIT)
#define WRITE_BIT_0(REG, BIT) CLR_BIT(REG, BIT)
		

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


// Baud rate = 115.2kbps; U2X=0
#if (F_CPU == 16000000)
#define BAUD_RATE_UBBR_115_2_KBPS 8
#define BAUD_RATE_UBBR_9_6_KBPS 103
#else
#error "Baud rate not supported for frequency"
#endif

#define BAUD_RATE_UBBR BAUD_RATE_UBBR_9_6_KBPS

#define DEBUG_LED_HALF_PERIOD_FLICKER_MS 50
#define DEBUG_LED_HALF_PERIOD_TOGGLE_MS  250

#define UART_SEND_INTERBYTE_PAUSE_MS 10



volatile uint8_t g_command_buffer;
uint8_t g_flag_command_received;

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

void do_flicker_debug_led_indefinitely()
{
	while(1)
	{
		debug_led_toggle();
		_delay_ms(DEBUG_LED_HALF_PERIOD_FLICKER_MS);
	}
}

void do_flicker_debug_led(int duration_ms)
{
	int N = duration_ms / (2 * DEBUG_LED_HALF_PERIOD_FLICKER_MS);
	
	int i;
	for (i = 0; i < N; ++i)
	{
		debug_led_toggle();
		_delay_ms(DEBUG_LED_HALF_PERIOD_FLICKER_MS);
	}
}

void do_blink_debug_led()
{
	debug_led_toggle();
	_delay_ms(DEBUG_LED_HALF_PERIOD_TOGGLE_MS);
	debug_led_toggle();
	_delay_ms(DEBUG_LED_HALF_PERIOD_TOGGLE_MS);
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
	do_flicker_debug_led_indefinitely();
}

void usart_send(char* pData, int length)
{
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

void usart_send_reorder(char* pData, int length)
{
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

void setup_gpio_pins(void)
{
	// UART
	PIN_MODE_INPUT(DDRD, DDD0);
	PIN_MODE_OUTPUT(DDRD, DDD1);
	
	// DEBUG LED
	PIN_MODE_OUTPUT(DDRB, DDB5);
}

void setup_usart(void)
{
	// USART0
	
	// Set baud rate
	UBRR0 = (uint16_t)BAUD_RATE_UBBR;
	
	// Enable receiver
	SET_BIT(UCSR0B, RXEN0);
	
	// DEBUG
	// Enable transmitter
	SET_BIT(UCSR0B, TXEN0);

	// Set frame format
	// 8 data bits
	WRITE_BIT(UCSR0C, UCSZ00, 1);
	WRITE_BIT(UCSR0C, UCSZ01, 1);
	WRITE_BIT(UCSR0B, UCSZ02, 0);
	
	// 1 stop bit
	WRITE_BIT(UCSR0C, USBS0, 0);
	
	// No parity bits
	WRITE_BIT(UCSR0C, UPM00, 0);
	WRITE_BIT(UCSR0C, UPM01, 0);

	// Enable RX interrupt
	SET_BIT(UCSR0B, RXCIE0);
}

int main(void)
{
    setup_gpio_pins();
	setup_usart();
	
	g_flag_command_received = 0;
	
	sei();
	
    while (1) 
    {
		if (g_flag_command_received)
		{
			do_flicker_debug_led(1000);
			float data = 4e5;
			usart_send((char*)&data, sizeof(float));
			g_flag_command_received = 0;
		}
    }
}

ISR(USART_RX_vect)
{
	if (IS_BIT_SET(UCSR0A, RXC0))
	{
		g_command_buffer = UDR0;
		g_flag_command_received = 1;
	}
}

