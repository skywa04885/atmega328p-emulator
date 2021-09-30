#pragma once

#include <stdint.h>

#define __BV(N, T) \
    ((T) 0x1 << (T) N)

#define _BV8(N) __BV(N, uint8_t)
#define _BV16(N) __BV(N, uint16_t)
#define _BV32(N) __BV(N, uint32_t)
#define _KB(N) ((uint32_t) (N) * (uint32_t) 1024)
#define _MB(N) (_KB (n) * (uint32_t) 1024)

#ifdef __M328P_DEBUG
#define __M328P_VERBOSE(A) A
#else
#define __M328P_VERBOSE(A)
#endif
