#pragma once

#include <stdint.h>

#undef NULL
#define NULL				(void*)0

#define INV32				(0xFFFFFFFF)
#define INV16				(0xFFFF)
#define INV8				(0xFF)
#define NOT_NUMBER			(0x7FFFFFFF)

typedef uint32_t			uint32;
typedef int32_t				int32;
typedef uint16_t			uint16;
typedef int16_t				int16;
typedef uint8_t				uint8;
typedef int8_t				int8;
typedef uint64_t			uint64;
typedef int64_t				int64;

typedef void(*Cbf)(uint8 nTag, uint8 nValue);

#if !defined(cplusplus)
typedef uint8				bool;
#define true				(!0)
#define false				(!!0)
#endif

