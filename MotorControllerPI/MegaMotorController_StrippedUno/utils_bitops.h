/*
 * utils_bitops.h
 *
 * Created: 5/27/2023 9:51:22 PM
 *  Author: KASO
 */ 

/*
 * Utility library for bit manipulation
 */

#ifndef UTILS_BITOPS_H_
#define UTILS_BITOPS_H_


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


#endif /* UTILS_BITOPS_H_ */