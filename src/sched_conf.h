#pragma once

#define MS_PER_TICK				(10)
#define MSEC_TO_TICK(x)			((x) / MS_PER_TICK)

typedef enum _TaskId
{
	TID_LED,
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
	EVT_LED_CMD,
	NUM_EVT,
} EvtId;
