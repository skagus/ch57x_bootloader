#pragma once

#include "sched.h"

#define TIME_FOR_LONG			MSEC_TO_TICK(1000)
#define TIME_DEBOUNCE			MSEC_TO_TICK(30)
#define TOTAL_PIN_IN			(1)	///
#define NUM_KEY_IN				(1)	///
#define NUM_ENC_IN				(2)

typedef enum _KeyOp
{
	KOP_SHORT,
	KOP_LONG,
	KOP_CW,	// for rotary.
	KOP_CCW, // for rotary.
	NUM_KOP,
}  KeyOp;

void KEY_Init(void);

/**
 * Callback will called with tag and KeyOp.
*/
void KEY_AddFunc(uint32_t nKeyId,Cbf pfCb,uint8_t nTag);

