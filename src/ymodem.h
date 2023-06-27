#pragma once
#include "types.h"


typedef enum
{
	YS_INIT,
	YS_HEADER,
	YS_DATA,	///< Some data received.
	YS_ERROR,		///< Some error occured.
	YS_CANCEL,		///< Canceled by Host.
	YS_END,
} YMState;

typedef enum _YRet
{
	YR_DONE,
	YR_CANCEL,
	YR_ERROR,
} YRet;

#define YM_TIMEOUT			(300) // 3sec.
/**
YModem에 뭔가를 요청할 때에는 Handler를 넘겨줘야 한다. 
RX일때 handler는. 
	YS_HEADER --> pBuf: file name, pnBytes: file size.
	YS_DATA --> pBuf: data, pnBytes: 수신 data packet size.
	YS_FAIL, YS_DONE --> parameter 의미없음
*
*/

typedef bool (*YmHandle)(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam);

YRet YM_DoRx(YmHandle pfRxHandle, void* pParam);
YRet YM_DoTx(YmHandle pfTxHandle, void* pParam);
void YM_Init(void);
