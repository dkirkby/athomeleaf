#include "float16.h"

// Macros to deconstruct an IEEE single-precision float
#define BITS(VALUE) (*(uint32_t*)(&(VALUE)))
#define SIGN(VALUE) (uint8_t)(BITS(VALUE)>>31)
#define EXPONENT(VALUE) (uint8_t)(BITS(VALUE)>>23)
#define SIGNIFICAND(VALUE) (uint32_t)(BITS(VALUE)&0x7FFFFF)
#define IMPLICIT_MSBIT (uint32_t)(1<<23)

// Macros to reconstruct a positive IEEE single-precision float
#define FLOAT_BITS(EXP,SIG) ((EXP<<23)|(SIG))
#define FROM_BITS(VALUE) (*(float*)(&(VALUE)))

static uint8_t _u8val;
static uint32_t _u32val;

// Converts a single-precision (32-bit) floating point value
// to its closest float16 value.
uint16_t to_float16(float value) {
    // check for underflow
    if(value <= 0.0) return FLOAT16_ZERO;
    // extract the input value's exponent
    _u8val = EXPONENT(value);
    // check for overflow
    if(_u8val >= 126 + (1<<FLOAT16_EXP_SIZE)) return FLOAT16_INF;
    // is the exponent within our normalized range?
    if(_u8val > 126) {
        // change exponent bias from 127 to 1: new biased exp is 1 to (1<<EXP_SIZE)-1
        _u8val -= 126;
        // build a normalized value with truncated significand
        _u32val = (_u8val << FLOAT16_SIG_SIZE) |
            (SIGNIFICAND(value) >> (7+FLOAT16_EXP_SIZE));
        // check for the special case of max exponent and zero significand
        // which would be confused with our INF special value: add an LS bit
        // to distinguish it.
        return _u32val == FLOAT16_INF ? (FLOAT16_INF|1) : _u32val;
    }
    // is the exponent within our sub-normalized range?
    if(_u8val > 126-FLOAT16_SIG_SIZE) {
        // how much do we need to shift the signficand by? (1 to SIG_SIZE)
        _u8val = 127 - _u8val;
        // return a sub-normalized value with zero exponent
        return (IMPLICIT_MSBIT|SIGNIFICAND(value)) >> (7+FLOAT16_EXP_SIZE+_u8val);
    }
    // if we get here, the exponent is too small to represent
    return FLOAT16_ZERO;
}

// Converts a float16 value to its closest single-precision (32-bit)
// floating point value.
float from_float16(uint16_t value) {
    // check for special values
    if(value == FLOAT16_ZERO) return 0.;
    if(value == FLOAT16_INF) return +1./0.;
    // extract the input value's exponent and significand
    _u8val = value >> FLOAT16_SIG_SIZE;
    value &= ((1<<FLOAT16_SIG_SIZE)-1);
    // is this a normalized value?
    if(_u8val > 0 && _u8val <= (1<<FLOAT16_EXP_SIZE)-1) {
        // change exponent bias from 1 to 127
        _u8val += 126;
    }
    else if(_u8val == 0) {
        // change exponent bias from 1 to 127
        _u8val = 126;
        // decrease exponent and shift significand to compensate for sub-normalization
        while(0 == (value & (1<<(FLOAT16_SIG_SIZE-1)))) {
            _u8val--;
            value <<= 1;
        }
        // one more shift for the implicit leading one of the normalized float
        value <<= 1;
        // remove the implicit leading one
        value &= ~(1<<FLOAT16_SIG_SIZE);
    }
    // combine the exponent and zero-padded significand
    _u32val = FLOAT_BITS(_u8val, value << (7+FLOAT16_EXP_SIZE));
    // return the result as a float
    return FROM_BITS(_u32val);
}
