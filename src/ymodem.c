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
	PKT_WAIT_1ST,	// SOH or SOX.
	PKT_WAIT_REST,	// from Seq ~ CRC.
	PKT_DONE,
} PktState;

typedef enum
{
	PR_NONE,
	PR_SUCC,	///< Good data.
	PR_EOT,		///< Single byte: EOT received.
	PR_CANCEL,	///< 
	PR_CRC_ERR,	///< CRC Error. or en expected.
	PR_ERROR,
} PktRet;

/*
[SOH::1] [SEQ:0x00:1] [nSEQ:0xFF:1] [filename:"foo.c":] [filesize:"1064":] [pad:00:118] [CRC::2]
*/
typedef struct _Pkt
{
	uint8 nCode;
	uint8 nSeq;
	uint8 nInvSeq;
	uint8 nData;
} Pkt;

typedef struct
{
	PktState	ePktState;	// Current packet state.
	PktRet		eRet;		// Packet RX return.
	uint16		nSize;		///< total RX size.
	uint16		nCntRx;		///< Received data count.(include head)
	uint8*		pBuf;
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
	pCtx->ePktState = PKT_WAIT_1ST;
	pCtx->eRet = PR_NONE;
	pCtx->pBuf = pBuf;
	pCtx->nCntRx = 0;
}

void _Reset(YmCtx* pstModem)
{
	pstModem->eState = YS_HEADER;
	pstModem->nPrvTick = Sched_GetTick();
	_PktReset(&(pstModem->stPktCtx), pstModem->aBuf);
}

/*
받은 packet에서 file정보 (file name, file size 를 뽑아온다.)
*/
bool ym_GetFileInfo(YmCtx* pYmCtx, Pkt* pPkt)
{
	bool bSucc = true;
	uint32 nLen = strlen((char*)&pPkt->nData) + 1; // including \0
	strncpy(pYmCtx->szFileName, (char*)&pPkt->nData, nLen);
	DBG_YM("F:%s(%d),", pYmCtx->szFileName, nLen);

	pYmCtx->nFileLen = (uint32)strtoul((char*)(&pPkt->nData + nLen), (char **)NULL, (int)0);
	pYmCtx->nFileOff = 0;
	DBG_YM("L:%d\n", pYmCtx->nFileLen);

	return bSucc;
}


bool ym_RcvPkt(PktCtx *pPktCtx, uint8 nNewData)
{
	bool bDone = false;
	Pkt* pPkt = (Pkt*)pPktCtx->pBuf;
	// CRC calculation in 128 or 1024 bytes.
	switch (pPktCtx->ePktState)
	{
		case PKT_WAIT_1ST:
		{
			pPktCtx->nCntRx = 0;
			pPktCtx->pBuf[pPktCtx->nCntRx] = nNewData;
			pPktCtx->nCntRx++;
			if (nNewData == YMODEM_SOH)
			{
				pPktCtx->nSize = DATA_BYTE_SMALL + EXTRA_SIZE;
				pPktCtx->ePktState = PKT_WAIT_REST;
				DBG_YM("SO:");
			}
			else if (nNewData == YMODEM_STX)
			{
				pPktCtx->nSize = DATA_BYTE_BIG + EXTRA_SIZE;
				pPktCtx->ePktState = PKT_WAIT_REST;
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
				DBG_YM("UN:%X ", nNewData);
				// Nothing to do.
			}
			break;
		}
		case PKT_WAIT_REST:
		{
			pPktCtx->pBuf[pPktCtx->nCntRx] = nNewData;
			pPktCtx->nCntRx++;
			if(pPktCtx->nCntRx >= pPktCtx->nSize)
			{
				bDone = true;
				uint32 nDataLen = pPktCtx->nSize - EXTRA_SIZE;
				uint16 nCalcCRC = UT_Crc16(&pPkt->nData, nDataLen);
				uint16 nReadCRC = (uint16)((&pPkt->nData)[nDataLen]) << 8;
				nReadCRC |= (uint16)nNewData;
				pPktCtx->eRet = (nReadCRC == nCalcCRC) ? PR_SUCC : PR_CRC_ERR;
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
	Pkt* pPkt = (Pkt*)pstModem->aBuf;

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
						DBG_YM("HR:%d\n", pPkt->nSeq);
						if(0 == pPkt->nData) // No rest file.
						{
							TxResp(YMODEM_ACK);
							bDone = true;
							pstModem->eState = YS_END;
						}
						else
						{
							TxResp(YMODEM_ACK);	// ACK for packet.(start for New file.)
							ym_GetFileInfo(pstModem, pPkt);
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
//						pstModem->eState = YS_HEADER;
					}
					else if(PR_CANCEL == pPktCtx->eRet)
					{
						TxResp(YMODEM_NACK);
						pstModem->eState = YS_CANCEL;
					}
					else if(PR_CRC_ERR == pPktCtx->eRet)
					{
						TxResp(YMODEM_NACK);
//						pstModem->eState = YS_HEADER;
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
						DBG_YM("DR:%d\n", pPkt->nSeq);
						if((0 != pPkt->nSeq) && (NULL != pfRxHandle))
						{
							uint32 nLen = MIN((uint32)(pPktCtx->nCntRx - EXTRA_SIZE),
											(pstModem->nFileLen - pstModem->nFileOff));
							pfRxHandle(&(pPkt->nData), &nLen, YS_DATA, pParam);
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

				case YS_CANCEL:
				case YS_ERROR:
				case YS_END:
				{
					bDone = true;
					break;
				}
			}
			
			DBG_YM("State:%d -> %X (%d)\n", ePrv, pstModem->eState, pPktCtx->eRet);
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
