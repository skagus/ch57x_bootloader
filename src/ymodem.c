#include "types.h"
#include "macro.h"
#include "cli.h"
#include "ymodem.h"
#include "hal.h"

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

typedef enum
{
	YMODEM_TYPE_START,
	YMODEM_TYPE_DATA,
	YMODEM_TYPE_END,
	YMODEM_TYPE_CANCEL,
	YMODEM_TYPE_ERROR,
} YmodemType;


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
} ymodem_t;

bool ym_RcvPkt(YPacket *pstPkt, uint8_t nNewData);

void ym_Reset(ymodem_t *pstModem)
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
bool ym_GetFileInfo(ymodem_t *pstModem)
{
	bool bSucc = true;
	bool bValid = false;
	uint16_t nBufIdx;

	for (int i = 0; i < MAX_FILE_NAME_LEN; i++)
	{
		pstModem->szFileName[i] = pstModem->stRxPkt.pPayload[i];
		if (pstModem->szFileName[i] == 0x00)
		{
			nBufIdx = i + 1;
			bValid = true;
			break;
		}
	}

	if (bValid == true)
	{
		for (int i = nBufIdx; i < MAX_FILE_NAME_LEN; i++)
		{
			if (pstModem->stRxPkt.pPayload[i] == 0x20)
			{
				pstModem->stRxPkt.pPayload[i] = 0x00;
				break;
			}
		}

		pstModem->nFileLen = (uint32_t)strtoul((const char *)&pstModem->stRxPkt.pPayload[nBufIdx], (char **)NULL, (int)0);
	}

	return bSucc;
}

bool ym_Rx(ymodem_t *pstModem, YmHandle pfRxHandle)
{
	bool bRet = false;
	uint32_t nThisLen;
	uint8 nRcvData;
	uint8 bUpdated = UART_RxD(&nRcvData);

	if ((0 != bUpdated) && ym_RcvPkt(&pstModem->stRxPkt, nRcvData) == true)
	{
		if (pstModem->eYmState != YMODEM_STATE_WAIT_HEAD)
		{
			if (pstModem->stRxPkt.eResp == YMODEM_CAN)
			{
				pstModem->eYmState = YMODEM_STATE_WAIT_CANCEL;
			}
		}

		switch (pstModem->eYmState)
		{
			case YMODEM_STATE_WAIT_HEAD:
			{
				if (pstModem->stRxPkt.eResp == YMODEM_EOT)
				{
					UART_TxD(YMODEM_NACK);
					pstModem->eYmState = YMODEM_STATE_WAIT_LAST;
				}
				else if (pstModem->stRxPkt.nSeq == 0x00)
				{
					pstModem->nReceived = 0;
					ym_GetFileInfo(pstModem);
					pfRxHandle(pstModem->pRxData, pstModem->nFileLen, YS_HEADER);
					UART_TxD(YMODEM_ACK);
					UART_TxD(YMODEM_C);

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
					UART_TxD(YMODEM_NACK);
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
					pfRxHandle(pstModem->pRxData, nThisLen, YS_DATA);

					pstModem->nReceived += nThisLen;

					UART_TxD(YMODEM_ACK);

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
					UART_TxD(YMODEM_NACK);
					pstModem->eYmState = YMODEM_STATE_WAIT_LAST;
				}
				else
				{
					nThisLen = (pstModem->nFileLen - pstModem->nReceived);
					if (nThisLen > pstModem->stRxPkt.nSizePayload)
					{
						nThisLen = pstModem->stRxPkt.nSizePayload;
					}
					pfRxHandle(pstModem->pRxData, nThisLen, YS_DATA);
					pstModem->nReceived += nThisLen;

					UART_TxD(YMODEM_ACK);
					pstModem->eType = YMODEM_TYPE_DATA;
					bRet = true;
				}
				break;
			}

			case YMODEM_STATE_WAIT_LAST:
			{
				UART_TxD(YMODEM_ACK);
				UART_TxD(YMODEM_C);
				pstModem->eYmState = YMODEM_STATE_WAIT_END;
				break;
			}
			case YMODEM_STATE_WAIT_END:
			{
				pfRxHandle(NULL, 0, YS_DONE);
				UART_TxD(YMODEM_ACK);
				pstModem->eYmState = YMODEM_STATE_WAIT_HEAD;
				pstModem->eType = YMODEM_TYPE_END;
				bRet = true;
				break;
			}
			case YMODEM_STATE_WAIT_CANCEL:
			{
				pfRxHandle(NULL, 0, YS_FAIL);
				UART_TxD(YMODEM_ACK);
				pstModem->eYmState = YMODEM_STATE_WAIT_HEAD;
				pstModem->eType = YMODEM_TYPE_CANCEL;
				bRet = true;
				break;
			}
		}
	}
	else
	{
		if (pstModem->stRxPkt.ePktState == YMODEM_PACKET_WAIT_FIRST)
		{
			if (Sched_GetTick() - pstModem->nPrvTick >= pstModem->nTimeOut)
			{
				pstModem->nPrvTick = Sched_GetTick();
				UART_TxD(YMODEM_C);
			}
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
			}
			if (nNewData == YMODEM_STX)
			{
				pstPkt->nSizePayload = DATA_BYTE_BIG;
				pstPkt->eResp = nNewData;
				pstPkt->ePktState = YMODEM_PACKET_WAIT_SEQ1;
			}
			if (nNewData == YMODEM_EOT)
			{
				pstPkt->eResp = nNewData;
				bRet = true;
			}
			if (nNewData == YMODEM_CAN)
			{
				pstPkt->eResp = nNewData;
				bRet = true;
			}
			break;

		case YMODEM_PACKET_WAIT_SEQ1:
			pstPkt->nSeq = nNewData;
			pstPkt->ePktState = YMODEM_PACKET_WAIT_SEQ2;
			break;

		case YMODEM_PACKET_WAIT_SEQ2:
			if (pstPkt->nSeq == (uint8_t)(~nNewData))
			{
				pstPkt->nDataIdx = 0;
				pstPkt->ePktState = YMODEM_PACKET_WAIT_DATA;
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
			}
			break;

		case YMODEM_PACKET_WAIT_CRCH:
			pstPkt->nRxCrc = (nNewData << 8);
			pstPkt->ePktState = YMODEM_PACKET_WAIT_CRCL;
			break;

		case YMODEM_PACKET_WAIT_CRCL:
			pstPkt->nRxCrc |= (nNewData << 0);
			pstPkt->ePktState = YMODEM_PACKET_WAIT_FIRST;
			uint16 crc = crc16(pstPkt->pPayload, pstPkt->nSizePayload);

			if (crc == pstPkt->nRxCrc)
			{
				bRet = true;
			}
			break;
	}

	return bRet;
}


void dumpData(uint8* aBuff, uint32 nLength)
{
	for(uint32 i=0; i< nLength; i++)
	{
		if(0 == (i % 32))
		{
			CLI_Printf("\r\n");
		}
		CLI_Printf(" %2X", aBuff[i]);
	}
}

uint8 gaBuff[2048];
uint8 gaName[128];
uint32 gnCurPtr;

bool _Handle(uint8* pBuf, uint32* pnBytes, YmStep eStep)
{
	HAL_DbgLog("%d, %X, %X\n", eStep, pBuf, *pnBytes);
	switch(eStep)
	{
		case YS_HEADER:
		{
//			memcpy(gaName, pBuf, *pnBytes);
			HAL_DbgLog("%d, %X, %X\n", eStep, pBuf, *pnBytes);
			gnCurPtr = 0;
			break;
		}
		case YS_DATA:
		{
//			memcpy(gaBuff + gnCurPtr, pBuf, *pnBytes);
			HAL_DbgLog("%d, %X, %X\n", eStep, pBuf, *pnBytes);
			gnCurPtr += *pnBytes;
			break;
		}
		case YS_FAIL:
		case YS_DONE:
		{
			HAL_DbgLog("%d, %X\n", eStep, pBuf);
			break;
		}
	}
	memcpy(gaBuff + gnCurPtr, pBuf, *pnBytes);
}

static ymodem_t gstYM;

void ym_Cmd(uint8 argc, char* argv[])
{
	if (argc < 2)
	{
		CLI_Printf("%s r <Addr>\r\n", argv[0]);
		CLI_Printf("%s t <Addr> <Size>\r\n", argv[0]);
	}
	else
	{
		uint32 nAddr = CLI_GetInt(argv[1]);
		if ('r' == argv[1][0])
		{
			CLI_Printf("TX in PC\r\n");
			ym_Reset(&gstYM);

			while(1)
			{
				if(ym_Rx(&gstYM, _Handle))
				{
					YmodemType eYT = gstYM.eType;
					if ((YMODEM_TYPE_END == eYT)
						|| (YMODEM_TYPE_CANCEL == eYT)
						|| (YMODEM_TYPE_ERROR == eYT))
					{
						break;
					}
				}
			}
		}
		else if ('t' == argv[1][0])
		{
			CLI_Printf("Start receive in PC\r\n");
			ym_Reset(&gstYM);
		}
		else if ('i' == argv[1][0])
		{
			CLI_Printf("Result: %d\n", gstYM.eType);
			if(YMODEM_TYPE_END == gstYM.eType)
			{
				dumpData(gaBuff, gstYM.nFileLen);
			}
		}
		else
		{
			CLI_Printf("Wrong command or parameter\r\n");
		}
	}
}

bool YM_DoRx(YmHandle pfRxHandle)
{
	ym_Reset(&gstYM);

	while(1)
	{
		if(ym_Rx(&gstYM, pfRxHandle))
		{
			YmodemType eYT = gstYM.eType;
			if ((YMODEM_TYPE_END == eYT)
				|| (YMODEM_TYPE_CANCEL == eYT)
				|| (YMODEM_TYPE_ERROR == eYT))
			{
				break;
			}
		}
	}
}

void YM_Init(void)
{
	CLI_Register("ym", ym_Cmd);
}
