#pragma once

#define OS_MSEC(x)		(x)
#define MS_PER_TICK     (10)

typedef enum _TaskId
{
	TID_CONSOLE,
	TID_BUT,
	TID_YMODEM,
	NUM_TASK,
} TaskId;

typedef enum _Evt
{
	EVT_TICK,
	EVT_UART,
	EVT_UART_YM,	///< UART Event for Y modem.
	EVT_ECALL_DONE,
	EVT_YMODEM,			///< Ymodem request or done.
	NUM_EVT,
} EvtId;
 