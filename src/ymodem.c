#include <string.h>
#include "types.h"
#include "macro.h"
#include "util.h"
#include "cli.h"
#include "ymodem.h"
#include "hal.h"
#include <stdarg.h>

#define YMODEM_SOH 0x01
#define YMODEM_STX 0x02
#define YMODEM_ACK 0x06
#define YMODEM_NACK 0x15
#define YMODEM_EOT 0x04
#define YMODEM_C 0x43
#define YMODEM_CAN 0x18
//#define YMODEM_BS 0x08

#define MAX_FILE_NAME_LEN	(128)
#define DATA_BYTE_SMALL		(128)
#define DATA_BYTE_BIG		(1024)

#define YMODEM_BUF_LENGTH	(DATA_BYTE_BIG + 5)

typedef enum
{
	YMODEM_STATE_WAIT_HEAD,
	YMODEM_STATE_WAIT_FIRST,
	YMODEM_STATE_WAIT_DATA,
	YMODEM_STATE_WAIT_LAST,
	YMODEM_STATE_WAIT_END,
	YMODEM_STATE_WAIT_CANCEL,
} YMState;

typedef enum
{
	YMODEM_PACKET_WAIT_FIRST,
	YMODEM_PACKET_WAIT_SEQ1,
	YMODEM_PACKET_WAIT_SEQ2,
	YMODEM_PACKET_WAIT_DATA,
	YMODEM_PACKET_WAIT_CRCH,
	YMODEM_PACKET_WAIT_CRCL,
} PktState;

/*
[SOH::1] [SEQ:0x00:1] [nSEQ:0xFF:1] [filename:"foo.c":] [filesize:"1064":] [pad:00:118] [CRC::2]
*/
typedef struct
{
	PktState	ePktState;
	uint16		nDataIdx;

	uint8		eResp;		// Protocol Response.
	uint8		nSeq;		// Seq Number.
	uint8*		pPayload;
	uint16		nSizePayload;	///< Payload size (128 | 1024)
	uint16		nRxCrc;
	uint8		aRxBuf[YMODEM_BUF_LENGTH];
} YPacket;

typedef struct
{
	YmodemType eType;
	YMState	eYmState;
	uint32	nPrvTick;
	uint32	nTimeOut;	// response timeout.

	uint8	szFileName[MAX_FILE_NAME_LEN];
	uint32	nFileLen;	// 전송될 파일 크기.
	uint32	nReceived;	// 현재까지 받은 데이터 양.

	uint8*	pRxData;

	YPacket	stRxPkt;
} YRcvCtx;

bool ym_RcvPkt(YPacket *pstPkt, uint8_t nNewData);

#define DBG_YM(...)				HAL_DbgLog(__VA_ARGS__)

inline void TxResp(uint8 nCode)
{
	UART_TxD(nCode);
	DBG_YM(" RSP(%X) ", nCode);
}

void ym_Reset(YRcvCtx *pstModem)
{
	pstModem->eYmState = YMODEM_STATE_WAIT_HEAD;
	pstModem->stRxPkt.ePktState = YMODEM_PACKET_WAIT_FIRST;
	pstModem->stRxPkt.pPayload = &pstModem->stRxPkt.aRxBuf[3];
	pstModem->pRxData = &pstModem->stRxPkt.aRxBuf[3];
	pstModem->nPrvTick = Sched_GetTick();
	pstModem->nTimeOut = 300;	// 10ms tick --> 3 secs.
}

/*
받은 packet에서 file정보 (file name, file size 를 뽑아온다.)
*/
bool ym_GetFileInfo(YRcvCtx *pstModem)
{
	bool bSucc = true;
	bool bValid = false;
	uint16_t nBufIdx;

	for (int i = 0; i < MAX_FILE_NAME_LEN; i++)
	{
		pstModem->szFileName[i] = pstModem->pRxData[i];
		if (pstModem->szFileName[i] == 0x0)
		{
			nBufIdx = i + 1;
			bValid = true;
			break;
		}
	}
//	DBG_YM("F:%s ", pstModem->szFileName);

	if (bValid && (nBufIdx < MAX_FILE_NAME_LEN))
	{
		for (int i = nBufIdx; i < MAX_FILE_NAME_LEN; i++)
		{
			if (pstModem->pRxData[i] == 0x20)
			{
				pstModem->pRxData[i] = 0x00;
				break;
			}
		}

		pstModem->nFileLen = (uint32_t)strtoul((const char *)&pstModem->pRxData[nBufIdx], (char **)NULL, (int)0);
	}
//	DBG_YM("S:%d ", pstModem->nFileLen);

	return bSucc;
}

bool ym_Rx(YRcvCtx *pstModem, YmHandle pfRxHandle, void* pParam)
{
	bool bRet = false;
	uint32_t nThisLen;
	uint8 nRcvData;
	uint8 bUpdated = UART_RxD(&nRcvData);

	if (bUpdated && (true == ym_RcvPkt(&pstModem->stRxPkt, nRcvData)))
	{
		if (pstModem->eYmState != YMODEM_STATE_WAIT_HEAD)
		{
			if (pstModem->stRxPkt.eResp == YMODEM_CAN)
			{
				pstModem->eYmState = YMODEM_STATE_WAIT_CANCEL;
			}
		}
		pstModem->nPrvTick = Sched_GetTick();

		DBG_YM("State: %d ", pstModem->eYmState);

		switch (pstModem->eYmState)
		{
			case YMODEM_STATE_WAIT_HEAD:
			{
				if (pstModem->stRxPkt.eResp == YMODEM_EOT)
				{
					TxResp(YMODEM_NACK);
					pstModem->eYmState = YMODEM_STATE_WAIT_LAST;
				}
				else if (pstModem->stRxPkt.nSeq == 0x00)
				{
					ym_GetFileInfo(pstModem);
					nThisLen = pstModem->nFileLen;
					pfRxHandle(pstModem->pRxData, &nThisLen, YS_HEADER, pParam);
					TxResp(YMODEM_ACK);
					TxResp(YMODEM_C);

					pstModem->nReceived = 0;
					pstModem->eYmState = YMODEM_STATE_WAIT_FIRST;
					pstModem->eType = YMODEM_TYPE_START;
					bRet = true;
				}
				break;
			}

			case YMODEM_STATE_WAIT_FIRST:
			{
				if (pstModem->stRxPkt.eResp == YMODEM_EOT)
				{
					TxResp(YMODEM_NACK);
					pstModem->eYmState = YMODEM_STATE_WAIT_LAST;
				}
				else if (pstModem->stRxPkt.nSeq == 0x01)
				{
					pstModem->nReceived = 0;

					nThisLen = (pstModem->nFileLen - pstModem->nReceived);
					if (nThisLen > pstModem->stRxPkt.nSizePayload)
					{
						nThisLen = pstModem->stRxPkt.nSizePayload;
					}
					pfRxHandle(pstModem->pRxData, &nThisLen, YS_DATA, pParam);

					pstModem->nReceived += nThisLen;

					TxResp(YMODEM_ACK);

					pstModem->eYmState = YMODEM_STATE_WAIT_DATA;
					pstModem->eType = YMODEM_TYPE_DATA;
					bRet = true;
				}
				break;
			}
			case YMODEM_STATE_WAIT_DATA:
			{
				if (pstModem->stRxPkt.eResp == YMODEM_EOT)
				{
					TxResp(YMODEM_NACK);
					pstModem->eYmState = YMODEM_STATE_WAIT_LAST;
				}
				else
				{
					nThisLen = (pstModem->nFileLen - pstModem->nReceived);
					if (nThisLen > pstModem->stRxPkt.nSizePayload)
					{
						nThisLen = pstModem->stRxPkt.nSizePayload;
					}
					pfRxHandle(pstModem->pRxData, &nThisLen, YS_DATA, pParam);
					pstModem->nReceived += nThisLen;

					TxResp(YMODEM_ACK);
					pstModem->eType = YMODEM_TYPE_DATA;
					bRet = true;
				}
				break;
			}

			case YMODEM_STATE_WAIT_LAST:
			{
				TxResp(YMODEM_ACK);
				TxResp(YMODEM_C);
				pstModem->eYmState = YMODEM_STATE_WAIT_END;
				break;
			}
			case YMODEM_STATE_WAIT_END:
			{
				pfRxHandle(NULL, NULL, YS_DONE, pParam);
				TxResp(YMODEM_ACK);
				pstModem->eYmState = YMODEM_STATE_WAIT_HEAD;
				pstModem->eType = YMODEM_TYPE_END;
				bRet = true;
				break;
			}
			case YMODEM_STATE_WAIT_CANCEL:
			{
				pfRxHandle(NULL, NULL, YS_FAIL, pParam);
				TxResp(YMODEM_ACK);
				pstModem->eYmState = YMODEM_STATE_WAIT_HEAD;
				pstModem->eType = YMODEM_TYPE_CANCEL;
				bRet = true;
				break;
			}
		}
	}
	else
	{
		if (Sched_GetTick() - pstModem->nPrvTick >= pstModem->nTimeOut)
		{
			pstModem->nPrvTick = Sched_GetTick();
			DBG_YM("Timeout\n");
			TxResp(YMODEM_C);
		}
	}
	return bRet;
}

bool ym_RcvPkt(YPacket *pstPkt, uint8 nNewData)
{
	bool bRet = false;

	switch (pstPkt->ePktState)
	{
		case YMODEM_PACKET_WAIT_FIRST:
			if (nNewData == YMODEM_SOH)
			{
				pstPkt->nSizePayload = DATA_BYTE_SMALL;
				pstPkt->eResp = nNewData;
				pstPkt->ePktState = YMODEM_PACKET_WAIT_SEQ1;
				DBG_YM("SOH ");
			}
			else if (nNewData == YMODEM_STX)
			{
				pstPkt->nSizePayload = DATA_BYTE_BIG;
				pstPkt->eResp = nNewData;
				pstPkt->ePktState = YMODEM_PACKET_WAIT_SEQ1;
				DBG_YM("STX ");
			}
			else if (nNewData == YMODEM_EOT)
			{
				pstPkt->eResp = nNewData;
				bRet = true;
				DBG_YM("EOT ");
			}
			else if (nNewData == YMODEM_CAN)
			{
				pstPkt->eResp = nNewData;
				DBG_YM("CAN ");
				bRet = true;
			}
			else
			{
				// ??
			}
			break;

		case YMODEM_PACKET_WAIT_SEQ1:
			pstPkt->nSeq = nNewData;
			pstPkt->ePktState = YMODEM_PACKET_WAIT_SEQ2;
			DBG_YM("SEQ2 ");
			break;

		case YMODEM_PACKET_WAIT_SEQ2:
			if (pstPkt->nSeq == (uint8)(~nNewData))
			{
				pstPkt->nDataIdx = 0;
				pstPkt->ePktState = YMODEM_PACKET_WAIT_DATA;
				DBG_YM("SEQ:%d ", pstPkt->nSeq);
			}
			else
			{
				pstPkt->ePktState = YMODEM_PACKET_WAIT_FIRST;
			}
			break;

		case YMODEM_PACKET_WAIT_DATA:
			pstPkt->pPayload[pstPkt->nDataIdx] = nNewData;
			pstPkt->nDataIdx++;
			if (pstPkt->nDataIdx >= pstPkt->nSizePayload)
			{
				pstPkt->ePktState = YMODEM_PACKET_WAIT_CRCH;
				DBG_YM("CRCH ");
			}
			break;

		case YMODEM_PACKET_WAIT_CRCH:
			pstPkt->nRxCrc = (nNewData << 8);
			pstPkt->ePktState = YMODEM_PACKET_WAIT_CRCL;
			DBG_YM("CRCL ");
			break;

		case YMODEM_PACKET_WAIT_CRCL:
			pstPkt->nRxCrc |= (nNewData << 0);
			pstPkt->ePktState = YMODEM_PACKET_WAIT_FIRST;
			uint16 nCRC = UT_Crc16(pstPkt->pPayload, pstPkt->nSizePayload);

			if (nCRC == pstPkt->nRxCrc)
			{
				bRet = true;
			}
			DBG_YM("CRC Done %X %X %X\n", pstPkt->eResp, nCRC, pstPkt->nRxCrc);
			break;
	}

	return bRet;
}

#if 0
uint8 gaBuff[2048];
uint8 gaName[128];
uint32 gnCurPtr;
uint32 gnFileLen;

bool _Handle(uint8* pBuf, uint32* pnBytes, YmStep eStep)
{
	DBG_YM("Hdl: %d, %X, %X ", eStep, pBuf, *pnBytes);
//	return true;

	switch(eStep)
	{
		case YS_HEADER:
		{
			_DbgLog("H:%X, %X ", pBuf, *pnBytes);
			strcpy((uint8*)(gaName), pBuf);
			gnFileLen = *pnBytes;
			gnCurPtr = 0;
			break;
		}
		case YS_DATA:
		{
			_DbgLog("D: %X, %X ", pBuf, *pnBytes);
			if( gnCurPtr + *pnBytes)
			{
				memcpy((uint8*)(gaBuff + gnCurPtr), pBuf, (uint32)(*pnBytes));
				gnCurPtr += *pnBytes;
			}
			break;
		}
		case YS_FAIL:
		case YS_DONE:
		default:
		{
			_DbgLog("E: %d, %d, %d", eStep, gnFileLen, gnCurPtr);
			break;
		}
	}
	return true;
}
#endif

static YRcvCtx gstYM;

YmodemType YM_DoRx(YmHandle pfRxHandle, void* pParam)
{
	UT_Printf("Send File to Device\n");
	ym_Reset(&gstYM);
	YmodemType eYT = YMODEM_TYPE_END;
	while(1)
	{
		if(ym_Rx(&gstYM, pfRxHandle, pParam))
		{
			eYT = gstYM.eType;
			if ((YMODEM_TYPE_END == eYT)
				|| (YMODEM_TYPE_CANCEL == eYT)
				|| (YMODEM_TYPE_ERROR == eYT))
			{
				break;
			}
		}
	}
	return eYT;
}

void ym_Cmd(uint8 argc, char* argv[])
{
	if (argc < 2)
	{
		UT_Printf("%s r <Addr>\n", argv[0]);
		UT_Printf("%s t <Addr> <Size>\n", argv[0]);
	}
	else
	{
		uint32 nAddr = UT_GetInt(argv[1]);
		if ('t' == argv[1][0])
		{
			UT_Printf("Start receive in PC\n");
			ym_Reset(&gstYM);
		}
		else 
		{
			UT_Printf("Wrong command or parameter\n");
		}
	}
}

void YM_Init(void)
{
	CLI_Register("ym", ym_Cmd);
}
