#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "CH57x_common.h"
#include "types.h"
#include "macro.h"
#include "util.h"
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

#define MAX_FILE_NAME_LEN	(128)
#define DATA_BYTE_SMALL		(128)
#define DATA_BYTE_BIG		(1024)
#define EXTRA_SIZE			(5)
//#define YMODEM_BUF_LENGTH	(DATA_BYTE_BIG + EXTRA_SIZE)

#define DBG_YM(...)			HAL_DbgLog(__VA_ARGS__)

typedef enum
{
	PKT_WAIT_HEADER,	// SOH or SOX.
	PKT_WAIT_DATA,	// from Seq ~ CRC.
	PKT_WAIT_TAIL,
	NUM_PKT_WAIT,
} PktState;

typedef enum
{
	PR_SUCC,	///< Good data.
	PR_EOT,		///< Single byte: EOT received.
	PR_CANCEL,	///< 
	PR_CRC_ERR,	///< CRC Error. or en expected.
	PR_ERROR,
} PktRet;

typedef enum _YRet
{
	YR_DONE,
	YR_CANCEL,
	YR_ERROR,
	YR_TIMEOUT,
	NUM_YR,
} YRet;

typedef enum _YState
{
	Y_IDLE,
	Y_READY,
	Y_RUN,
	NUM_Y,
} YState;

/*
[SOH::1] [SEQ:0x00:1] [nSEQ:0xFF:1] [filename:"foo.c":] [filesize:"1064":] [pad:00:118] [CRC::2]
*/

typedef struct
{
	PktState	ePktState;	// Current packet state.
	PktRet		eRet;		// Packet RX return.

	uint8		nSeqNo;		///< Sequence number. (from Sender)
	uint16		nCntRx;		///< Received data count in current state.

	uint16		nSize;		///< total RX size (from Sender)
	uint16		nRcvCRC;	///< Received CRC. (from sender)
	uint8		aBuf[DATA_BYTE_BIG];
} PktCtx;

typedef struct _YCtx
{
	YMState		eState;
	YRet		eRet;
	uint32		nPrvTick;	///< To Check timeout.
	uint32		nCntTO;		///< Timeout counter.

	uint32		nFileLen;
	uint32		nFileOff;	///< File�� �뷮��, ���� packet�� base.
	char		szFileName[MAX_FILE_NAME_LEN];

	uint8		nSeqNo;		///< Sent or Rcvd packet No.
	uint8		nCntPkt;	///< # of data packet to transfer.
	union
	{
		PktCtx		stPktCtx;
		uint8		aTxBuf[DATA_BYTE_BIG + EXTRA_SIZE];
	};
	

	YReq 		stReq;
} RunCtx;

typedef struct _YmCtx 
{
	YState		eState;
	RunCtx		stRun;
	YReq		stReq;
} YmCtx;

inline void TxResp(uint8 nCode)
{
	UART_TxD(nCode);
	DBG_YM("->(%X)\n", nCode);
}

void _PktReset(PktCtx* pPkt)
{
	pPkt->ePktState = PKT_WAIT_HEADER;
	pPkt->eRet = PR_SUCC;
	pPkt->nCntRx = 0;
}

void _PrepareCtx(RunCtx* pRun)
{
	pRun->eState = YS_META;
	pRun->nPrvTick = 0;
	pRun->nCntTO = 0;
	pRun->nSeqNo = 0;
	pRun->eRet = YR_DONE;
}

/*
Get file information from received packet (file name, file size)
*/
bool ym_GetFileInfo(RunCtx* pYmCtx, uint8* pData)
{
	bool bSucc = true;
	uint32 nLen = strlen((char*)pData) + 1; // including \0
	strncpy(pYmCtx->szFileName, (char*)pData, nLen);
	DBG_YM("F:%s(%d),", pYmCtx->szFileName, nLen);

	pYmCtx->nFileLen = (uint32)strtoul((char*)(pData + nLen), (char **)NULL, (int)0);
	pYmCtx->nFileOff = 0;
	DBG_YM("L:%d\n", pYmCtx->nFileLen);

	return bSucc;
}

/**
 * @return true if done receiving.
*/
bool ym_RcvPkt(PktCtx *pPktCtx, uint8 nNewData)
{
	bool bDone = false;
	// CRC calculation in 128 or 1024 bytes.
	switch (pPktCtx->ePktState)
	{
		case PKT_WAIT_HEADER:
		{
			pPktCtx->nCntRx++;
			if(1 == pPktCtx->nCntRx) // Header.
			{
				if (nNewData == YMODEM_SOH)
				{
					pPktCtx->nSize = DATA_BYTE_SMALL;
					DBG_YM("SO:");
				}
				else if (nNewData == YMODEM_STX)
				{
					pPktCtx->nSize = DATA_BYTE_BIG;
					DBG_YM("ST:");
				}
				else if (nNewData == YMODEM_EOT)
				{
					pPktCtx->eRet = PR_EOT;
					DBG_YM("EO:");
					bDone = true;
				}
				else if (nNewData == YMODEM_CAN)
				{
					pPktCtx->eRet = PR_CANCEL;
					DBG_YM("CA:");
					bDone = true;
				}
				else
				{
					pPktCtx->eRet = PR_ERROR;
					DBG_YM("unexpected for 1st:%X\n", nNewData);
					bDone = true;
				}
			}
			else if(2 == pPktCtx->nCntRx) // Seq No.
			{
				pPktCtx->nSeqNo = nNewData;
			}
			else if(3 == pPktCtx->nCntRx) // Seq No inversion.
			{
				uint8 nInvSeq = ~nNewData;
				if (pPktCtx->nSeqNo != nInvSeq)
				{
					DBG_YM("Seq # error %X, %X\n", pPktCtx->nSeqNo, nNewData);
					pPktCtx->eRet = PR_ERROR;
					bDone = true;
				}
				pPktCtx->ePktState = PKT_WAIT_DATA;
				pPktCtx->nCntRx = 0;
			}
			break;
		}
		case PKT_WAIT_DATA:
		{
			pPktCtx->aBuf[pPktCtx->nCntRx] = nNewData;
			pPktCtx->nCntRx++;
			if(pPktCtx->nCntRx >= pPktCtx->nSize)
			{
				pPktCtx->ePktState = PKT_WAIT_TAIL;
				pPktCtx->nCntRx = 0;
			}
			break;
		}
		case PKT_WAIT_TAIL:
		{
			pPktCtx->nCntRx++;
			if(1 == pPktCtx->nCntRx)
			{
				pPktCtx->nRcvCRC = nNewData << 8;
			}
			else
			{
				uint16 nCalcCRC = UT_Crc16(pPktCtx->aBuf, pPktCtx->nSize);
				pPktCtx->nRcvCRC |= nNewData;
				if(pPktCtx->nRcvCRC != nCalcCRC)
				{
					DBG_YM("CRC %X, %X\n", pPktCtx->nRcvCRC, nCalcCRC);
					pPktCtx->eRet = PR_CRC_ERR;
				}
				bDone = true;
			}
			break;
		}
		default:
		{
			break;
		}
	}

	return bDone;
}

/**
 * return true if All sequence is done.l
*/
bool ym_HandleRxPkt(RunCtx *pRun, YReq* pReq)
{
	bool bDone = false;
	PktCtx* pPktCtx = &(pRun->stPktCtx);
	YMState ePrv = pRun->eState;

	switch (pRun->eState)
	{
		case YS_META:
		{
			if (PR_SUCC == pPktCtx->eRet)
			{
				DBG_YM("HDR:%d, %X\n", pPktCtx->nSeqNo, pPktCtx->aBuf[0]);
				if(0 == pPktCtx->aBuf[0]) // End packet case.
				{
					TxResp(YMODEM_ACK);
					pRun->eRet = YR_DONE;
					pRun->eState = YS_END;
					bDone = true;
				}
				else
				{
					TxResp(YMODEM_ACK);	// ACK for packet.(start for New file.)
					ym_GetFileInfo(pRun, pPktCtx->aBuf);
					if(NULL != pReq->pfHandle)
					{
						uint32 nLen = pRun->nFileLen;
						pReq->pfHandle((uint8*)pRun->szFileName, &nLen, YS_META, pReq->pParam);
					}
					TxResp(YMODEM_C);	// Go continue.
					pRun->eState = YS_DATA;
				}
			}
			else if(PR_EOT == pPktCtx->eRet)
			{
				TxResp(YMODEM_ACK); // Rcv EOT (No new file!!)
				TxResp(YMODEM_C); // Give me END packet.
			}
			else if(PR_CANCEL == pPktCtx->eRet)
			{
				TxResp(YMODEM_NACK);
				pRun->eRet = YR_CANCEL;
				pRun->eState = YS_END;
				bDone = true;
			}
			else if(PR_CRC_ERR == pPktCtx->eRet)
			{
				TxResp(YMODEM_NACK);
				pRun->eRet = YR_ERROR;
				pRun->eState = YS_END;
				bDone = true;
			}
			else
			{
				bDone = true;
			}
			break;
		}

		case YS_DATA: // maybe data...
		{
			if (PR_SUCC == pPktCtx->eRet)
			{
				DBG_YM("DR:%d\n", pPktCtx->nSeqNo);
				if(0 != pPktCtx->nSeqNo)
				{
					uint32 nLen = MIN((uint32)(pPktCtx->nSize),
									(pRun->nFileLen - pRun->nFileOff));
					pReq->pfHandle(pPktCtx->aBuf, &nLen, YS_DATA, pReq->pParam);
					pRun->nFileOff += nLen;
				}
				TxResp(YMODEM_ACK);
			}
			else if(PR_EOT == pPktCtx->eRet) // Next file.
			{
				TxResp(YMODEM_NACK);
				pRun->eState = YS_META;
			}
			else if(PR_CANCEL == pPktCtx->eRet)
			{
				TxResp(YMODEM_ACK);
				pRun->eRet = YR_CANCEL;
				pRun->eState = YS_END;
				bDone = true;
			}
			else if(PR_CRC_ERR == pPktCtx->eRet)
			{
				TxResp(YMODEM_NACK);  ///< request Resend.
			}
			else
			{
				bDone = true;
			}
			break;
		}

		case YS_END:
		default:
		{
			bDone = true;
			break;
		}
	}
	
	DBG_YM("St:%d->%X (%d)\n", ePrv, pRun->eState, pPktCtx->eRet);
	return bDone;
}

bool _RxHandle(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam)
{
	UNUSED(pParam);

	switch(eStep)
	{
		case YS_META:
		{
			DBG_YM("H:%s, %d\n", pBuf, *pnBytes); // File name and File Length.
			break;
		}
		case YS_DATA:
		{
			DBG_YM("D:%X, %d\n", pBuf, *pnBytes);
			break;
		}
		default:
		{
			DBG_YM("?:%X, %X\n", pBuf, pnBytes);
			break;
		}
	}
	return true;
}

bool _TxHandle(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam)
{
	UNUSED(pParam);

	switch(eStep)
	{
		case YS_META:
		{
			sprintf((char*)pBuf, "%s", "test.log");
			*pnBytes = 1024 * 2 + 100;
			break;
		}
		case YS_DATA:
		{
			uint32 nOff = *pnBytes;
			memset(pBuf, '0' + (nOff / 1024), 1024);
			break;
		}
		default:
		{
			DBG_YM("?:%X, %X\n", pBuf, pnBytes);
			break;
		}
	}
	return true;
}

uint32 _SendFirst(uint8* pBase, YmHandle pfTxHandle, void* pParam)
{
	DBG_YM("First->\n");
	uint8* pData = pBase;
	*pData = YMODEM_SOH; pData++;
	*pData = 0; pData++;
	*pData = 0xFF; pData++;

	int32 nLen;
	pfTxHandle(pData, (uint32*)&nLen, YS_META, pParam);	// Get header.
	uint32 nSizeOff = strlen((char*)pData) + 1;
	sprintf((char*)(pData + nSizeOff), "%d", (int)nLen);
	uint16 nCRC = UT_Crc16(pData, DATA_BYTE_SMALL);
	pData += DATA_BYTE_SMALL;
	*pData =(nCRC >> 8); pData++;
	*pData =(nCRC & 0xFF);

	UART0_SendString(pBase, DATA_BYTE_SMALL + EXTRA_SIZE);

	return nLen;
}

void _SendData(uint8* pBase, YmHandle pfTxHandle, uint8 nSeqNo, void* pParam)
{
	DBG_YM("Data(%d) ->\n", nSeqNo);

	uint8* pData = pBase;
	*pData = YMODEM_STX; pData++;
	*pData = nSeqNo; pData++;
	*pData = ~nSeqNo; pData++;
	uint32 nThisLen = DATA_BYTE_BIG;
	pfTxHandle(pData, &nThisLen, YS_DATA, pParam); // Get data.
	uint16 nCRC = UT_Crc16(pData, DATA_BYTE_BIG);
	pData += DATA_BYTE_BIG;
	*pData =(nCRC >> 8); pData++;
	*pData =(nCRC & 0xFF);

	UART0_SendString(pBase, DATA_BYTE_BIG + EXTRA_SIZE);
}

void _SendNull(uint8* pBase)
{
	DBG_YM("Null ->\n");
	uint8* pData = pBase;
	*pData = YMODEM_SOH; pData++;
	*pData = 0; pData++;
	*pData = 0xFF; pData++;
	memset(pData, 0x0, DATA_BYTE_SMALL);
	uint16 nCRC = UT_Crc16(pData, DATA_BYTE_SMALL);
	pData += DATA_BYTE_SMALL;
	*pData =(nCRC >> 8); pData++;
	*pData =(nCRC & 0xFF);
	UART0_SendString(pBase, DATA_BYTE_SMALL + EXTRA_SIZE);
}


uint8 _WaitResp(void)
{
	uint8 nRxData;
	while(0 == UART_RxD((char*)&nRxData));
	return nRxData;
}

void YM_DoTx(YmHandle pfTxHandle, void* pParam)
{
	UT_Printf("Data to Device to Host\n");
	char nRxData;
	//_Reset(&gstYM);

	PktCtx stPkt;

	while(UART_RxD(&nRxData)); // RCV buffer����.
	DBG_YM("Empty Rcv\n");
	while(YMODEM_C != _WaitResp());
	DBG_YM("Start\n");

	uint32 nLen = _SendFirst(stPkt.aBuf, pfTxHandle, pParam);

	DBG_YM("1st\n");
	while(YMODEM_ACK != _WaitResp());	// ACK.
	DBG_YM("1st Ack\n");
	while(YMODEM_C != _WaitResp());	// C
	DBG_YM("1st C\n");

	uint8 nCntPkt = nLen / 1024;
	uint8 nSeq = 0;
	while(nSeq < nCntPkt)
	{
		_SendData(stPkt.aBuf, pfTxHandle, nSeq, pParam);
		while(YMODEM_ACK != _WaitResp());
		DBG_YM("Data Ack %d\n", nSeq);
		nSeq++;
	}

	UART_TxD(YMODEM_EOT);
	while(YMODEM_NACK != _WaitResp());	// NAK.
	DBG_YM("EOT NACK\n");

	UART_TxD(YMODEM_EOT);
	while(YMODEM_ACK != _WaitResp());	// ACK.
	DBG_YM("EOT ACK\n");
	while(YMODEM_C != _WaitResp());	// C
	DBG_YM("EOT C\n");

	_SendNull(stPkt.aBuf);
	while(YMODEM_ACK !=_WaitResp());
	DBG_YM("END ACK\n");
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
		UNUSED(nAddr);
		if ('r' == argv[1][0])
		{
			YReq stReq;
			stReq.bRx = true;
			stReq.pfHandle = _RxHandle;
			stReq.pParam = NULL;
			YM_Request(&stReq);
		}
#if 0		
		else if ('R' == argv[1][0])
		{
			YM_DoRx(_RxHandle, &nAddr);
		}
#endif
		else if ('t' == argv[1][0])
		{
			YReq stReq;
			stReq.bRx = false;
			stReq.pfHandle = _TxHandle;
			stReq.pParam = NULL;
			YM_Request(&stReq);
		}
		else 
		{
			UT_Printf("Wrong command or parameter\n");
		}
	}
}


void cbf_RxUartYm(uint8 tag, uint8 result)
{
	UNUSED(tag);
	UNUSED(result);
	Sched_TrigAsyncEvt(BIT(EVT_UART_YM));
}

YmCtx gstCtx;

bool _ymRx(RunCtx* pRun, YReq* pReq)
{
	bool bDone = false;
	uint8 nRxData;
	bool bRcv = false;
	while(UART_RxD((char*)&nRxData))
	{
		bRcv = true;
		PktCtx* pPktCtx = &(pRun->stPktCtx);

		if (ym_RcvPkt(pPktCtx, nRxData))
		{
			DBG_YM("PD:%d, %d\n", pPktCtx->eRet, pPktCtx->nCntRx);
			if(ym_HandleRxPkt(pRun, pReq))
			{
				DBG_YM("Done: %d\n", pRun->eRet);
				bDone = true;
				break;
			}
			_PktReset(pPktCtx);
		}
		pRun->nPrvTick = Sched_GetTick();
		pRun->nCntTO = 0;
	}
	if(false == bRcv)
	{
		if(pRun->nCntTO > (YM_TIMEOUT / YM_RETRY_PERIOD))
		{
			pRun->eRet = YR_TIMEOUT;
			pRun->eState = YS_END;
			bDone = true;
		}
		else
		{
			UART_TxD(YMODEM_C);
		}
		pRun->nCntTO++;
	}
	else
	{
		pRun->nCntTO = 0;
	}
	return bDone;
}

bool _ymTx(RunCtx* pRun, YReq* pReq)
{
	bool bDone = false;
	uint8 nRxData;
	if(false == UART_RxD((char*)&nRxData))  // empty receive buffer.
	{
		pRun->nCntTO++;
		if(pRun->nCntTO > YM_TIMEOUT / YM_RETRY_PERIOD)
		{
			pRun->eRet = YR_TIMEOUT;
			pRun->eState = YS_END;
			bDone = true;
		}
		DBG_YM("TO:%d\n", pRun->nCntTO);
		return bDone;
	}

	DBG_YM("RX:%X, %d\n", nRxData, pRun->eState);
	pRun->nCntTO = 0;

	switch(pRun->eState)
	{
		case YS_META:
		{
			if(((YMODEM_C == nRxData) && (0 == pRun->nSeqNo))
				|| (YMODEM_NACK == nRxData))
			{
				// TxMetaPkt(pRun);
				uint32 nLen = _SendFirst(pRun->aTxBuf, pReq->pfHandle, pReq->pParam);
				pRun->nSeqNo = 1;
				pRun->nCntPkt = DIV_UP(nLen, DATA_BYTE_BIG);
			}
			else if(YMODEM_ACK == nRxData)
			{
				pRun->eState = YS_DATA;
				Sched_Yield();
			}
			break;
		}
		case YS_DATA:
		{
			if(YMODEM_C == nRxData)
			{
				pRun->nSeqNo = 0;
				// Data packet은 1부터 시작.
				_SendData(pRun->aTxBuf, pReq->pfHandle, pRun->nSeqNo + 1, pReq->pParam);
			}
			else if(YMODEM_ACK == nRxData)
			{
				pRun->nSeqNo++;
				if(pRun->nSeqNo >= pRun->nCntPkt)
				{
					DBG_YM("EO>\n");
					UART_TxD(YMODEM_EOT);
					pRun->nSeqNo = 0;
					pRun->eState = YS_END;
				}
				else
				{
					_SendData(pRun->aTxBuf, pReq->pfHandle, pRun->nSeqNo + 1, pReq->pParam);
				}
			}
			else if(YMODEM_NACK == nRxData)
			{
				_SendData(pRun->aTxBuf, pReq->pfHandle, pRun->nSeqNo, pReq->pParam);
			}
			else if(YMODEM_CAN == nRxData)
			{
				DBG_YM("CN->\n");
				UART_TxD(YMODEM_CAN);
				UART_TxD(YMODEM_CAN);
				UART_TxD(YMODEM_CAN);
				UART_TxD(YMODEM_CAN);
				pRun->eRet = YR_CANCEL;
				bDone = true;
			}
			break;
		}
		case YS_END:
		{
			if((YMODEM_NACK == nRxData) && (0 == pRun->nSeqNo))
			{
				DBG_YM("EO>\n");
				UART_TxD(YMODEM_EOT);
				pRun->nSeqNo ++;
			}
			else if((YMODEM_ACK == nRxData) && (1 == pRun->nSeqNo))
			{
				pRun->nSeqNo ++;
			}
			else if((YMODEM_C == nRxData) && (2 == pRun->nSeqNo))
			{
				pRun->nSeqNo++;
				_SendNull(pRun->aTxBuf);
			}
			else if((YMODEM_ACK == nRxData) && (3 == pRun->nSeqNo))
			{
				pRun->nSeqNo++;
				pRun->eRet = YR_DONE;
				bDone = true;
			}
			else
			{
				pRun->eRet = YR_ERROR;
				bDone = true;
			}
		}
	}
	return bDone;
}

void ym_Run(Evts bmEvt)
{
	UNUSED(bmEvt);
	RunCtx* pRun = &(gstCtx.stRun);
	YReq* pReq = &(gstCtx.stReq);

	if(Y_RUN == gstCtx.eState)
	{
		bool bDone = (pReq->bRx)
				? _ymRx(pRun, pReq)
				: _ymTx(pRun, pReq);

		if(bDone)
		{
			DBG_YM("Final %d\n", pRun->eRet);
			char nRxData;
			while(UART_RxD(&nRxData));  // empty receive buffer.

			gstCtx.eState = Y_IDLE;
			pReq->pfHandle(NULL, NULL, YS_END, pReq->pParam);
			CLI_RegUartEvt();
			Sched_TrigSyncEvt(BIT(EVT_YMODEM));
			Sched_Wait(BIT(EVT_YMODEM), 0);	// timeout
		}
		else // End of Transfer.
		{
			Sched_Wait(BIT(EVT_UART_YM), YM_RETRY_PERIOD);
		}
	}
	else if(Y_READY == gstCtx.eState) // Start running.
	{
		char nRxData;
		while(UART_RxD(&nRxData));  // empty receive buffer.
		UART_SetCbf(cbf_RxUartYm, NULL);
		if(pReq->bRx)
		{
			UT_Printf("Transfer Host --> Device\n");
			_PktReset(&(pRun->stPktCtx));
		}
		else // TX
		{
			UT_Printf("Transfer Host <-- Device\n");
		}
		_PrepareCtx(pRun);
		gstCtx.eState = Y_RUN;
		Sched_Yield(); // Call again.
	}
	else // Y Idle.
	{
		Sched_Wait(BIT(EVT_YMODEM), 0);
	}
}


bool YM_Request(YReq* pstReq)
{
	if(Y_IDLE == gstCtx.eState)
	{
		gstCtx.stReq = *pstReq;
		gstCtx.eState = Y_READY;
		Sched_TrigSyncEvt(BIT(EVT_YMODEM)); // to start YModem.
		return true;
	}
	return false;
}


void YM_Init(void)
{
	CLI_Register("ym", ym_Cmd);
	Sched_Register(TID_YMODEM, ym_Run);
}
