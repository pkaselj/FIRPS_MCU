/*
 * util_pindefs.h
 *
 * Created: 5/27/2023 9:34:02 PM
 *  Author: KASO
 */ 

/*
 * Utility pin definitions so that pins
 * can be changed and reassigned easily.
 * Macros and defines are used to save
 * memory during runtime and to mitigate
 * pin decoding from runtime to compile-time.
*/

#ifndef UTIL_PINDEFS_H_
#define UTIL_PINDEFS_H_

//////////////////////////////////////////////////////////////////////////
//							PUBLIC INTERFACE
//////////////////////////////////////////////////////////////////////////

/*
	Begin Public Interface
*/

// Configure PIN_T as output. E.g. PIN_MODE_OUTPUT(PIN_A2)
#define PIN_MODE_OUTPUT(PIN_T)  (DDR_REG_(PIN_T) |= (1 << DDR_BIT_(PIN_T)))
// Configure PIN_T as input. E.g. PIN_MODE_INOPUT(PIN_A2)
#define PIN_MODE_INPUT(PIN_T)	(DDR_REG_(PIN_T) &= ~(1 << DDR_BIT_(PIN_T)))

// Write 1 or 0 to pin PIN_T. NOTE: Pin must be configured as output.
// E.g. WRITE_PIN(PIN_A2, 1)
#define WRITE_PIN(PIN_T, VALUE) CONCAT(WRITE_PIN_VALUE_,VALUE)(PIN_T)

#define ENABLE_PULLUP(PIN_T)	WRITE_PIN(PIN_T, 1)
#define DIABLE_PULLUP(PIN_T)	WRITE_PIN(PIN_T, 0)

#define TOGGLE_PIN(PIN_T)	(PORT_REG_(PIN_T) ^= (1 << PORT_BIT_(PIN_T)))

/*
	End Public Interface
*/

/*
	Begin Pin Definitions
*/

// By convention, only these defines should be used
// as arguments to macros that take PIN_T

#define PIN_A0 A0
#define PIN_A1 A1
#define PIN_A2 A2
#define PIN_A3 A3
#define PIN_A4 A4
#define PIN_A5 A5
#define PIN_A6 A6
#define PIN_A7 A7

#define PIN_B0 B0
#define PIN_B1 B1
#define PIN_B2 B2
#define PIN_B3 B3
#define PIN_B4 B4
#define PIN_B5 B5
#define PIN_B6 B6
#define PIN_B7 B7

#define PIN_C0 C0
#define PIN_C1 C1
#define PIN_C2 C2
#define PIN_C3 C3
#define PIN_C4 C4
#define PIN_C5 C5
#define PIN_C6 C6
#define PIN_C7 C7

#define PIN_D0 D0
#define PIN_D1 D1
#define PIN_D2 D2
#define PIN_D3 D3
#define PIN_D4 D4
#define PIN_D5 D5
#define PIN_D6 D6
#define PIN_D7 D7

#define PIN_E0 E0
#define PIN_E1 E1
#define PIN_E2 E2
#define PIN_E3 E3
#define PIN_E4 E4
#define PIN_E5 E5
#define PIN_E6 E6
#define PIN_E7 E7

#define PIN_F0 F0
#define PIN_F1 F1
#define PIN_F2 F2
#define PIN_F3 F3
#define PIN_F4 F4
#define PIN_F5 F5
#define PIN_F6 F6
#define PIN_F7 F7

#define PIN_G0 G0
#define PIN_G1 G1
#define PIN_G2 G2
#define PIN_G3 G3
#define PIN_G4 G4
#define PIN_G5 G5

#define PIN_H0 H0
#define PIN_H1 H1
#define PIN_H2 H2
#define PIN_H3 H3
#define PIN_H4 H4
#define PIN_H5 H5
#define PIN_H6 H6
#define PIN_H7 H7

#define PIN_J0 J0
#define PIN_J1 J1
#define PIN_J2 J2
#define PIN_J3 J3
#define PIN_J4 J4
#define PIN_J5 J5
#define PIN_J6 J6
#define PIN_J7 J7

#define PIN_K0 K0
#define PIN_K1 K1
#define PIN_K2 K2
#define PIN_K3 K3
#define PIN_K4 K4
#define PIN_K5 K5
#define PIN_K6 K6
#define PIN_K7 K7

#define PIN_L0 L0
#define PIN_L1 L1
#define PIN_L2 L2
#define PIN_L3 L3
#define PIN_L4 L4
#define PIN_L5 L5
#define PIN_L6 L6
#define PIN_L7 L7

/*
	End Pin Definitions
*/

//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//							PRIVATE IMPLEMENTATIONS
//////////////////////////////////////////////////////////////////////////

// Concatenation utilites, do not use outside of
// this file. Concatenation uses second level
// expansion to expand macros before concatenating
#define CONCAT_(X, Y) X##Y
#define CONCAT(X, Y) CONCAT_(X, Y)

#define CONCAT_3_(X, Y, Z) X##Y##Z
#define CONCAT_3(X, Y, Z) CONCAT_3_(X, Y, Z)

// Macro to get letter of a pin
// e.g. LETTER_(PIN_A2) evaluates to A
#define LETTER_(PIN_T) CONCAT(LETTER_, PIN_T)
// Macro to get number of a pin
// e.g. NUMBER_(PIN_A2) evaluates to 2
#define NUMBER_(PIN_T) CONCAT(NUMBER_, PIN_T)

// Macro to get PORT register of a PIN_T
// e.g. PORT_REG_(PIN_A2) evaluates to PORTA
#define PORT_REG_(PIN_T) CONCAT(PORT, LETTER_(PIN_T))
// Macro to get DDR register of a PIN_T
// e.g. DDR_REG_(PIN_A2) evaluates to DDRA
#define DDR_REG_(PIN_T) CONCAT(DDR, LETTER_(PIN_T))
// Macro to get PIN register of a PIN_T
// e.g. PIN_REG_(PIN_A2) evaluates to PINA
#define PIN_REG_(PIN_T) CONCAT(PIN, LETTER_(PIN_T))

// Macro to get PORT register's bit position of a PIN_T
// e.g. PORT_BIT_(PIN_A2) evaluates to PORTA2
#define PORT_BIT_(PIN_T) CONCAT(PORT, PIN_T)
// Macro to get DDR register's bit position of a PIN_T
// e.g. DDR_BIT_(PIN_A2) evaluates to DDA2
#define DDR_BIT_(PIN_T) CONCAT(DD, PIN_T)
// Macro to get PIN register's bit position of a PIN_T
// e.g. PIN_BIT_(PIN_A2) evaluates to PINA2
#define PIN_BIT_(PIN_T) CONCAT(PIN, PIN_T)

// Stringification macros
//#define STR_(X) #X
//#define STR(X) STR_(X)

#define WRITE_PIN_VALUE_1(PIN_T) (PORT_REG_(PIN_T) |= (1 << PORT_BIT_(PIN_T)))
#define WRITE_PIN_VALUE_0(PIN_T) (PORT_REG_(PIN_T) &= ~(1 << PORT_BIT_(PIN_T)))

//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
//			Dirty Trick to Make LETTER_ and NUMBER_ Work
//////////////////////////////////////////////////////////////////////////

#define LETTER_A0 A
#define LETTER_A1 A
#define LETTER_A2 A
#define LETTER_A3 A
#define LETTER_A4 A
#define LETTER_A5 A
#define LETTER_A6 A
#define LETTER_A7 A

#define LETTER_B0 B
#define LETTER_B1 B
#define LETTER_B2 B
#define LETTER_B3 B
#define LETTER_B4 B
#define LETTER_B5 B
#define LETTER_B6 B
#define LETTER_B7 B

#define LETTER_C0 C
#define LETTER_C1 C
#define LETTER_C2 C
#define LETTER_C3 C
#define LETTER_C4 C
#define LETTER_C5 C
#define LETTER_C6 C
#define LETTER_C7 C

#define LETTER_D0 D
#define LETTER_D1 D
#define LETTER_D2 D
#define LETTER_D3 D
#define LETTER_D4 D
#define LETTER_D5 D
#define LETTER_D6 D
#define LETTER_D7 D

#define LETTER_E0 E
#define LETTER_E1 E
#define LETTER_E2 E
#define LETTER_E3 E
#define LETTER_E4 E
#define LETTER_E5 E
#define LETTER_E6 E
#define LETTER_E7 E

#define LETTER_F0 F
#define LETTER_F1 F
#define LETTER_F2 F
#define LETTER_F3 F
#define LETTER_F4 F
#define LETTER_F5 F
#define LETTER_F6 F
#define LETTER_F7 F

#define LETTER_G0 G
#define LETTER_G1 G
#define LETTER_G2 G
#define LETTER_G3 G
#define LETTER_G4 G
#define LETTER_G5 G

#define LETTER_H0 H
#define LETTER_H1 H
#define LETTER_H2 H
#define LETTER_H3 H
#define LETTER_H4 H
#define LETTER_H5 H
#define LETTER_H6 H
#define LETTER_H7 H

#define LETTER_J0 J
#define LETTER_J1 J
#define LETTER_J2 J
#define LETTER_J3 J
#define LETTER_J4 J
#define LETTER_J5 J
#define LETTER_J6 J
#define LETTER_J7 J

#define LETTER_K0 K
#define LETTER_K1 K
#define LETTER_K2 K
#define LETTER_K3 K
#define LETTER_K4 K
#define LETTER_K5 K
#define LETTER_K6 K
#define LETTER_K7 K

#define LETTER_L0 L
#define LETTER_L1 L
#define LETTER_L2 L
#define LETTER_L3 L
#define LETTER_L4 L
#define LETTER_L5 L
#define LETTER_L6 L
#define LETTER_L7 L

//////////////////////////////////////////////////////////////////////////

#define NUMBER_A0 0
#define NUMBER_A1 1
#define NUMBER_A2 2
#define NUMBER_A3 3
#define NUMBER_A4 4
#define NUMBER_A5 5
#define NUMBER_A6 6
#define NUMBER_A7 7

#define NUMBER_B0 0
#define NUMBER_B1 1
#define NUMBER_B2 2
#define NUMBER_B3 3
#define NUMBER_B4 4
#define NUMBER_B5 5
#define NUMBER_B6 6
#define NUMBER_B7 7

#define NUMBER_C0 0
#define NUMBER_C1 1
#define NUMBER_C2 2
#define NUMBER_C3 3
#define NUMBER_C4 4
#define NUMBER_C5 5
#define NUMBER_C6 6
#define NUMBER_C7 7

#define NUMBER_D0 0
#define NUMBER_D1 1
#define NUMBER_D2 2
#define NUMBER_D3 3
#define NUMBER_D4 4
#define NUMBER_D5 5
#define NUMBER_D6 6
#define NUMBER_D7 7

#define NUMBER_E0 0
#define NUMBER_E1 1
#define NUMBER_E2 2
#define NUMBER_E3 3
#define NUMBER_E4 4
#define NUMBER_E5 5
#define NUMBER_E6 6
#define NUMBER_E7 7

#define NUMBER_F0 0
#define NUMBER_F1 1
#define NUMBER_F2 2
#define NUMBER_F3 3
#define NUMBER_F4 4
#define NUMBER_F5 5
#define NUMBER_F6 6
#define NUMBER_F7 7

#define NUMBER_G0 0
#define NUMBER_G1 1
#define NUMBER_G2 2
#define NUMBER_G3 3
#define NUMBER_G4 4
#define NUMBER_G5 5

#define NUMBER_H0 0
#define NUMBER_H1 1
#define NUMBER_H2 2
#define NUMBER_H3 3
#define NUMBER_H4 4
#define NUMBER_H5 5
#define NUMBER_H6 6
#define NUMBER_H7 7

#define NUMBER_J0 0
#define NUMBER_J1 1
#define NUMBER_J2 2
#define NUMBER_J3 3
#define NUMBER_J4 4
#define NUMBER_J5 5
#define NUMBER_J6 6
#define NUMBER_J7 7

#define NUMBER_K0 0
#define NUMBER_K1 1
#define NUMBER_K2 2
#define NUMBER_K3 3
#define NUMBER_K4 4
#define NUMBER_K5 5
#define NUMBER_K6 6
#define NUMBER_K7 7

#define NUMBER_L0 0
#define NUMBER_L1 1
#define NUMBER_L2 2
#define NUMBER_L3 3
#define NUMBER_L4 4
#define NUMBER_L5 5
#define NUMBER_L6 6
#define NUMBER_L7 7

//////////////////////////////////////////////////////////////////////////

#endif /* UTIL_PINDEFS_H_ */