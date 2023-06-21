#pragma once
#include "types.h"

typedef enum _YmStep
{
	YS_HEADER,	// File 이름과 size를 넘겨준다.
	YS_DATA,	// Data를 넘겨준다.
	YS_DONE,	// 완료됨.
	YS_FAIL,	// Error 발생함.
	NUM_YS,
} YmStep;

/**
YModem에 뭔가를 요청할 때에는 Handler를 넘겨줘야 한다. 
Handler는 Tx와 Rx일때 조금 다르게 동작한다. 
RX일때 handler는. 
	YS_HEADER --> pBuf: file name, pnBytes: file size.
	YS_DATA --> pBuf: data, pnBytes: 수신 data packet size.
	YS_FAIL, YS_DONE --> parameter 의미없음
TX일때 handler는 	 
	YS_HEADER --> pBuf: file name, pnBytes: file size.
	YS_DATA --> pBuf: data, pnBytes: 수신 data packet size,
		이때 pnBytes는 in/out으로 사용되는데 추가 데이터가 없지않은 이상 ymodem이 원하는 크기를 채워줘야 함.
	YS_FAIL, YS_DONE --> parameter 의미없음
 * 
*/

typedef bool (*YmHandle)(uint8* pBuf, uint32* pnBytes, YmStep eStep);
bool YM_DoRx(YmHandle pfRxHandle);
bool YM_DoTx(YmHandle pfTxHandle);
void YM_Init(void);
