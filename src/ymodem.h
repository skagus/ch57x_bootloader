#pragma once
#include "types.h"


typedef enum
{
	YS_META,		///< 1st packet. (File name and size, or EOT or end pkt)
	YS_DATA,		///< Some data received.
	YS_END
} YMState;


#define YM_TIMEOUT			(1000)	// 10 secs.
#define YM_RETRY_PERIOD		(100)	// 1 sec.
/**
YModem에 뭔가를 요청할 때에는 Handler를 넘겨줘야 한다. 
RX일때 handler는. 
	YS_HEADER --> pBuf: file name, pnBytes: file size.
	YS_DATA --> pBuf: data, pnBytes: 수신 data packet size.
	YS_FAIL, YS_DONE --> parameter 의미없음
*/

typedef bool (*YmHandle)(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam);

typedef struct _YReq
{
	bool bRx; // TX or RX.
	YmHandle pfHandle;
	void* pParam;
} YReq;


//void YM_DoRx(YmHandle pfRxHandle, void* pParam);
//void YM_DoTx(YmHandle pfTxHandle, void* pParam);

void YM_Init(void);
bool YM_Request(YReq* pstReq);
