#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
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
//#define YMODEM_BS 0x08

#define MAX_FILE_NAME_LEN	(128)
#define DATA_BYTE_SMALL		(128)
#define DATA_BYTE_BIG		(1024)
#define EXTRA_SIZE			(5)
#define YMODEM_BUF_LENGTH	(DATA_BYTE_BIG + EXTRA_SIZE)

#define DBG_YM(...)			HAL_DbgLog(__VA_ARGS__)

typedef enum
{
	PKT_WAIT_HEADER,	// SOH or SOX.
	PKT_WAIT_DATA,	// from Seq ~ CRC.
	PKT_WAIT_TAIL,
	PKT_DONE,
} PktState;

typedef enum
{
	PR_SUCC,	///< Good data.
	PR_EOT,		///< Single byte: EOT received.
	PR_CANCEL,	///< 
	PR_CRC_ERR,	///< CRC Error. or en expected.
	PR_ERROR,
} PktRet;

/*
[SOH::1] [SEQ:0x00:1] [nSEQ:0xFF:1] [filename:"foo.c":] [filesize:"1064":] [pad:00:118] [CRC::2]
*/

typedef struct
{
	PktState	ePktState;	// Current packet state.
	PktRet		eRet;		// Packet RX return.
	uint16		nCntRx;		///< Received data count in current state.

	uint16		nSize;		///< total RX size (from Sender)
	uint8		nSeqNo;		///< Sequence number. (from Sender)
	uint16		nRcvCRC;	///< Received CRC. (from sender)
	uint8*		pDataBuf;	///< Only for data. (from sender)
} PktCtx;

typedef struct
{
	YMState		eState;
	uint32		nPrvTick;	///< To Check timeout.
	uint32		nCntTO;		///< Timeout counter.

	uint32		nFileLen;
	uint32		nFileOff;	///< File의 용량중, 현재 packet의 base.
	char		szFileName[MAX_FILE_NAME_LEN];
	uint8		aBuf[YMODEM_BUF_LENGTH + 10];
	PktCtx		stPktCtx;
} YmCtx;


inline void TxResp(uint8 nCode)
{
	UART_TxD(nCode);
	DBG_YM("->(%X)\n", nCode);
}

void _PktReset(PktCtx* pCtx, uint8* pBuf)
{
	pCtx->ePktState = PKT_WAIT_HEADER;
	pCtx->nCntRx = 0;
	pCtx->eRet = PR_SUCC;

	pCtx->pDataBuf = pBuf;
	pCtx->nCntRx = 0;
}

void _Reset(YmCtx* pstModem)
{
	pstModem->eState = YS_INIT;
	pstModem->nPrvTick = Sched_GetTick();
}

/*
받은 packet에서 file정보 (file name, file size 를 뽑아온다.)
*/
bool ym_GetFileInfo(YmCtx* pYmCtx, uint8* pData)
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


bool ym_RcvPkt(PktCtx *pPktCtx, uint8 nNewData)
{
	bool bDone = false;
	// CRC calculation in 128 or 1024 bytes.
	switch (pPktCtx->ePktState)
	{
		case PKT_WAIT_HEADER:
		{
			if(0 == pPktCtx->nCntRx)
			{
				if (nNewData == YMODEM_SOH)
				{
					pPktCtx->nSize = DATA_BYTE_SMALL;
//					DBG_YM("SO:");
				}
				else if (nNewData == YMODEM_STX)
				{
					pPktCtx->nSize = DATA_BYTE_BIG;
//					DBG_YM("ST:");
				}
				else if (nNewData == YMODEM_EOT)
				{
					pPktCtx->eRet = PR_EOT;
//					DBG_YM("EO:");
					bDone = true;
				}
				else if (nNewData == YMODEM_CAN)
				{
					pPktCtx->eRet = PR_CANCEL;
//					DBG_YM("CA:");
					bDone = true;
				}
				else
				{
					pPktCtx->eRet = PR_ERROR;
//					DBG_YM("UN:%X ", nNewData);
					// Nothing to do.
				}
			}
			else if(1 == pPktCtx->nCntRx)
			{
				pPktCtx->nSeqNo = nNewData;
			}
			else if(2 == pPktCtx->nCntRx)
			{
				uint8 nInvSeq = ~nNewData;
				if (pPktCtx->nSeqNo != nInvSeq)
				{
					DBG_YM("Seq # error %X, %X\n", pPktCtx->nSeqNo, nNewData);
					pPktCtx->eRet = PR_ERROR;
				}
				pPktCtx->ePktState = PKT_WAIT_DATA;
				pPktCtx->nCntRx = 0;
				break;
			}
			pPktCtx->nCntRx++;
			break;
		}
		case PKT_WAIT_DATA:
		{
			pPktCtx->pDataBuf[pPktCtx->nCntRx] = nNewData;
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
			if(0 == pPktCtx->nCntRx)
			{
				pPktCtx->nRcvCRC = nNewData << 8;
				pPktCtx->nCntRx++;
			}
			else
			{
				pPktCtx->nRcvCRC |= nNewData;
				uint16 nCalcCRC = UT_Crc16(pPktCtx->pDataBuf, pPktCtx->nSize);
				if(pPktCtx->nRcvCRC != nCalcCRC)
				{
					DBG_YM("CRC %X, %X\n", pPktCtx->nRcvCRC, nCalcCRC);
					pPktCtx->eRet = PR_CRC_ERR;
				}
				bDone = true;
			}
			break;
		}
		default: // PKT_DONE
		{
			// Nothing to do..
			break;
		}
	}

	return bDone;
}

bool ym_Rx(YmCtx *pstModem, YmHandle pfRxHandle, void* pParam)
{
	bool bDone = false;
	uint8 nRcvData;
	uint8 bUpdated = UART_RxD((char*)&nRcvData);
	PktCtx* pPktCtx = &(pstModem->stPktCtx);
	pPktCtx->pDataBuf = pstModem->aBuf;

	if(YS_INIT == pstModem->eState)
	{
		_PktReset(pPktCtx, pstModem->aBuf);
		pstModem->eState = YS_HEADER;
	}

	if (bUpdated)
	{
		pstModem->nPrvTick = Sched_GetTick();
		pstModem->nCntTO = 0;
		if (ym_RcvPkt(pPktCtx, nRcvData))
		{
			YMState ePrv = pstModem->eState;

			switch (pstModem->eState)
			{
				case YS_HEADER:
				{
					if (PR_SUCC == pPktCtx->eRet)
					{
						DBG_YM("HDR:%d, %X\n", pPktCtx->nSeqNo, pPktCtx->pDataBuf[0]);
						if(0 == pPktCtx->pDataBuf[0]) // No file name information.
						{
							TxResp(YMODEM_ACK);
							bDone = true;
							pstModem->eState = YS_END;
						}
						else
						{
							TxResp(YMODEM_ACK);	// ACK for packet.(start for New file.)
							ym_GetFileInfo(pstModem, pPktCtx->pDataBuf);
							if(NULL != pfRxHandle)
							{
								uint32 nLen = pstModem->nFileLen;
								pfRxHandle((uint8*)pstModem->szFileName, &nLen, YS_HEADER, pParam);
							}
							TxResp(YMODEM_C);	// Go continue.
							pstModem->eState = YS_DATA;
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
						pstModem->eState = YS_CANCEL;
					}
					else if(PR_CRC_ERR == pPktCtx->eRet)
					{
						TxResp(YMODEM_NACK);
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
						if((0 != pPktCtx->nSeqNo) && (NULL != pfRxHandle))
						{
							uint32 nLen = MIN((uint32)(pPktCtx->nSize),
											(pstModem->nFileLen - pstModem->nFileOff));
							pfRxHandle(pPktCtx->pDataBuf, &nLen, YS_DATA, pParam);
							pstModem->nFileOff += nLen;
						}
						TxResp(YMODEM_ACK);
					}
					else if(PR_EOT == pPktCtx->eRet) // Next file.
					{
						TxResp(YMODEM_NACK);
						pstModem->eState = YS_HEADER;
					}
					else if(PR_CANCEL == pPktCtx->eRet)
					{
						TxResp(YMODEM_ACK);
						pstModem->eState = YS_CANCEL;
					}
					else if(PR_CRC_ERR == pPktCtx->eRet)
					{
						TxResp(YMODEM_NACK);
					}
					else
					{
						bDone = true;
					}
					break;
				}

				case YS_INIT:
				case YS_CANCEL:
				case YS_ERROR:
				default:
				{
					bDone = true;
					break;
				}
			}
			
			DBG_YM("St:%d->%X (%d)\n", ePrv, pstModem->eState, pPktCtx->eRet);
			_PktReset(pPktCtx, pstModem->aBuf);
		}
	}
	else if ((Sched_GetTick() - pstModem->nPrvTick) >= YM_TIMEOUT)
	{
		if((pstModem->eState <= YS_DATA) && (pstModem->nCntTO < 5))
		{
			pstModem->nPrvTick = Sched_GetTick();
			pstModem->nCntTO ++;
			DBG_YM("TO");
			TxResp(YMODEM_C);
		}
		else
		{
			bDone = true;
		}
	}

	return bDone;
}

bool _Handle(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam)
{
	UNUSED(pParam);

	switch(eStep)
	{
		case YS_HEADER:
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

static YmCtx gstYM;

YRet YM_DoRx(YmHandle pfRxHandle, void* pParam)
{
	UT_Printf("Send File to Device\n");
	_Reset(&gstYM);
	YRet eYRet = YR_DONE;
	char nRxData;
	while(UART_RxD(&nRxData));

	while(1)
	{
		if(ym_Rx(&gstYM, pfRxHandle, pParam))
		{
			YMState eState = gstYM.eState;
			if(eState == YS_CANCEL) eYRet = YR_CANCEL;
			else if(eState == YS_ERROR) eYRet = YR_ERROR;

			break;
		}
	}
	return eYRet;
}

#if 0
YRet YM_DoTx(YmHandle pfTxHandle, void* pParam)
{
	UT_Printf("Data to Device to Host\n");
	char nRxData;
	while(UART_RxD(&nRxData));
	_Reset(&gstYM);
	YRet eYRet = YR_DONE;
	Pkt* pPkt = (Pkt*)gstYM.aBuf;

//////////// Prepare 1st Packet ///////////////
	uint32 nLen = 1030;
//	pfTxHandle(pPkt->SMALL.aData, &nLen, YS_HEADER, pParam);
	sprintf((char*)(pPkt->SMALL.aData), "%s", "test.log");

	gstYM.nFileLen = nLen;
	while(UART_RxD(&nRxData));
	if(YMODEM_C != nRxData) goto END;
	
	nLen = strlen((char*)pPkt->SMALL.aData) + 1;
	sprintf((char*)(pPkt->SMALL.aData + nLen), "%d", (int)gstYM.nFileLen);
	pPkt->nCode = YMODEM_SOH;
	pPkt->nSeq = 0;
	pPkt->nInvSeq = 0xFF;
	uint16 nCRC = UT_Crc16(pPkt->SMALL.aData, DATA_BYTE_SMALL);
	pPkt->SMALL.nCrcL = (nCRC & 0xFF);
	pPkt->SMALL.nCrcH = (nCRC >> 8);
	UART0_SendString(pPkt, DATA_BYTE_SMALL + EXTRA_SIZE);

	return eYRet;
END:
	return YR_ERROR;
}
#endif

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
		if ('r' == argv[1][0])
		{
			YM_DoRx(_Handle, &nAddr);
		}
		else if ('t' == argv[1][0])
		{
			UT_Printf("Start receive in PC\n");
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
