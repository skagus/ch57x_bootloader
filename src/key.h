#pragma once

#include "sched.h"

#define TIME_FOR_LONG			MSEC_TO_TICK(1000)
#define TIME_DEBOUNCE			MSEC_TO_TICK(30)
#define TOTAL_PIN_IN			(1)	///
#define NUM_KEY_IN				(1)	///
#define NUM_ENC_IN				(2)

void KEY_ISR();
void KEY_Init();
void KEY_Run();

