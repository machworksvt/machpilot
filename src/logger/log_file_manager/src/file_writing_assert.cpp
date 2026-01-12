// Source - https://codereview.stackexchange.com/q
// Posted by icdae, modified by community. See post 'Timeline' for change
// history Retrieved 2025-12-16, License - CC BY-SA 3.0

/*
 * A simple compile-time endian test
 * g++ -std=c++11 -Wall -Werror -Wextra -pedantic -pedantic-errors endian.cpp -o
 * endian
 *
 * This can be used with specialized template functions, classes, and class
 * methods in order better tailor code and reduce reliance on runtime
 * checking systems.
 */

#include <cstdint>
#include <iostream>

/**
 * hl_endianness
 *
 * This enumeration can be placed into templated objects in order to generate
 * compile-time code based on a program's target endianness.
 *
 * The values placed in this enum are used just in case the need arises in
 * order to manually compare them against the number order in the
 * endianValues[] array.
 */
enum hl_endianness : uint32_t {
  HL_LITTLE_ENDIAN = 0x03020100,
  HL_BIG_ENDIAN = 0x00010203,
  HL_PDP_ENDIAN = 0x01000302,
  HL_UNKNOWN_ENDIAN = 0xFFFFFFFF
};

/**
 * A constant array used to determine a program's target endianness. The
 * values
 *  in this array can be compared against the values placed in the
 * hl_endianness enumeration.
 */
static constexpr uint8_t endianValues[4] = {0, 1, 2, 3};

/**
 * A simple function that can be used to help determine a program's endianness
 * at compile-time.
 */
constexpr hl_endianness getEndianOrder() {
  return (0x00 == endianValues[0])        // If Little Endian Byte Order,
             ? HL_LITTLE_ENDIAN           // return 0 for little endian.
             : (0x03 == endianValues[0])  // Else if Big Endian Byte Order,
                   ? HL_BIG_ENDIAN        // return 1 for big endian.
                   : (0x02 ==
                      endianValues[0])         // Else if PDP Endian Byte Order,
                         ? HL_PDP_ENDIAN       // return 2 for pdp endian.
                         : HL_UNKNOWN_ENDIAN;  // Else return -1 for wtf endian.
}

static_assert(getEndianOrder() == HL_LITTLE_ENDIAN);

static_assert(sizeof(short) == 2, "short must be 2 bytes");
static_assert(sizeof(int) == 4, "int must be 4 bytes");
static_assert(sizeof(long) == 8, "long must be 8 bytes");
static_assert(sizeof(long long) == 8, "long long must be 8 bytes");
static_assert(sizeof(size_t) == 8, "size_t must be 8 bytes");