/*
 * MotorControllerFeedforward.c
 *
 * Created: 3/28/2023 3:20:49 PM
 * Author : KASO
 */ 

/*
	This code is used for feedforward control
	of vnh2sp30 motor driver board.
*/

#include <avr/io.h>


/*
 *	Begin Macros
 */

#define SET_BIT(REG, BIT) REG |= _BV(BIT)
#define CLR_BIT(REG, BIT) REG &= ~_BV(BIT)

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
//#define SAMPLING_FREQUENCY 1000
//#define SAMPLING_INTERVAL 1.0/SAMPLING_FREQUENCY

// Motor PWM duty cycle (percentage 0-100)
#define DUTY_CYCLE 33

// Motor PWM input duty cycle percentage denominator
// PWM_DUTY_CYCLE = 100 / DUTY_CYCLE_DENOMINATOR
#define DUTY_CYCLE_DENOMINATOR (uint8_t)(100.0 / DUTY_CYCLE)

/*
 *	End Constants
 */


/*
 *	Start Global Variables
 */


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
	
	//uint32_t inverted_pwm_value = 100 * (DUTY_CYCLE_DENOMINATOR - 1)/(DUTY_CYCLE_DENOMINATOR);
	OCR0B = (uint8_t)((255 * ((uint32_t)100 - DUTY_CYCLE))/100);
}

int main(void)
{
	// Setup
	setup_gpio_pins();
	set_motor_direction();
	set_motor_speed_pwm();
	set_motor_enable();
	
    while (1) 
    {
    }
}

/*
 *	End User Code Implementation
 */

