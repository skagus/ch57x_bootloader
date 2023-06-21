#pragma once

#define OS_MSEC(x)		(x)
#define MS_PER_TICK     (10)

typedef enum _TaskId
{
	TID_CONSOLE,
	TID_BUT,
	NUM_TASK,
} TaskId;

typedef enum _Evt
{
	EVT_TICK,
	EVT_UART,
	EVT_ECALL_DONE,
	NUM_EVT,
} EvtId;
 