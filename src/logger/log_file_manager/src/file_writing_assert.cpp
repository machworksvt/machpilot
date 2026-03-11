#include <cstddef>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  
#elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    #error "Big Endian is not supported"
#else
    #error "Unknown endianness"
#endif



static_assert(sizeof(short) == 2, "short must be 2 bytes");
static_assert(sizeof(int) == 4, "int must be 4 bytes");
static_assert(sizeof(long) == 8, "long must be 8 bytes");
static_assert(sizeof(long long) == 8, "long long must be 8 bytes");
static_assert(sizeof(std::size_t) == 8, "size_t must be 8 bytes");