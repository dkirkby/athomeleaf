#ifndef FLOAT16_H
#define FLOAT16_H

//==============================================================================
// float16 is a 16-bit floating point format optimized for values in the range
// [1,32768), where they have their full 12-bit precision, i.e.
//
//   |to_float16(value) - value| <= value/4096
//
// Values between 0-1 can also be represented but with a logarithmically
// decreasing precision as values approach zero:
//
//   value > 0.243896   ensures frac. error < 0.1%
//   value > 0.0241699  ensures frac. error < 1%
//   value > 0.00219727 ensures frac. error < 10%
//
// Conversions are round-trip invariant, i.e., the following holds for all
// possible float16 values:
//
//   value == to_float16(from_float16(value))
//
// All values <= 0.000244141 are converted as the special value FLOAT16_ZERO.
// All values >= 32768 are converted to the special value FLOAT_INF.
//
// The float16 representation has no sign bit (assumed to be positive), uses
// 4 bits for an exponent with bias 1 and the remaining 12 bits for the
// significand. There are only two special values FLOAT_ZERO and FLOAT_INF.
// Unlike the standard IEEE floating point formats, values with the maximum
// exponent (15) are used to represent values when the significand is zero
// (IEEE reserves these for additional special values).
//==============================================================================

#include <stdint.h>

#define FLOAT16_EXP_SIZE 4
#define FLOAT16_SIG_SIZE 12

// ZERO has all bits set to zero and represents any float <= ?
#define FLOAT16_ZERO (uint16_t)0x0000

// INF has all exponent bits set to one and all significant bits set to zero,
// and represents any float >= ? 
#define FLOAT16_INF  (uint16_t)(((1<<FLOAT16_EXP_SIZE)-1)<<FLOAT16_SIG_SIZE)

typedef uint16_t float16;

extern float16 to_float16(float value);
extern float from_float16(float16 value);

// Macros to deconstruct an IEEE single-precision float
#define BITS(VALUE) (*(uint32_t*)(&(VALUE)))
#define SIGN(VALUE) (uint8_t)(BITS(VALUE)>>31)
#define EXPONENT(VALUE) (uint8_t)(BITS(VALUE)>>23)
#define SIGNIFICAND(VALUE) (uint32_t)(BITS(VALUE)&0x7FFFFF)
#define IMPLICIT_MSBIT (uint32_t)(1<<23)

// Macros to reconstruct a positive IEEE single-precision float
#define FLOAT_BITS(EXP,SIG) ((EXP<<23)|(SIG))
#define FROM_BITS(VALUE) (*(float*)(&(VALUE)))

#endif
