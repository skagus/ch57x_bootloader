
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "CH57x_common.h"
#include "CH573SFR.h"
#include "CH57X_usbdev.h"
#include "usb_cdc.h"

#define EN_SNIFF					(0)
#define EN_DBG						(0)

#define EP_SIZE_BULK				(MAX_PACKET_SIZE)
#define EP_SIZE_INT					(DEFAULT_ENDP0_SIZE)
#define UART_FIFO_SIZE				(64)
#define NUM_CDC						(2)

typedef struct _RingBuf
{
	uint8_t		aRing[UART_FIFO_SIZE];
	uint16_t		nWrIdx;
	uint16_t		nRdIdx;
	uint16_t		nCnt;
} RingBuf;

__attribute__((aligned(4))) uint8_t  gaBuf4EP0[64 + 64 + 64]; //ep0(64)+ep4_out(64)+ep4_in(64)
__attribute__((aligned(4))) uint8_t  gaBuf4EP1[64 + 64];      //ep1_out(64)+ep1_in(64)
__attribute__((aligned(4))) uint8_t  gaBuf4EP2[64 + 64];      //ep2_out(64)+ep2_in(64)
__attribute__((aligned(4))) uint8_t  gaBuf4EP3[64 + 64];      //ep3_out(64)+ep3_in(64)

RingBuf gUartFifoB = {.nWrIdx = 0,.nRdIdx = 0,.nCnt = 0,};
RingBuf gUartFifoA = {.nWrIdx = 0,.nRdIdx = 0,.nCnt = 0,};

inline void UEP_DONE_D2H(uint8_t nEP)
{
	if(1 == nEP)
	{
		R8_UEP1_T_LEN = 0;
		R8_UEP1_CTRL = (R8_UEP1_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
	else if(2 == nEP)
	{
		R8_UEP2_T_LEN = 0;
		R8_UEP2_CTRL = (R8_UEP2_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
	else if(3 == nEP)
	{
		R8_UEP3_T_LEN = 0;
		R8_UEP3_CTRL = (R8_UEP3_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
	else if(4 == nEP)
	{
		R8_UEP4_T_LEN = 0;
		R8_UEP4_CTRL = (R8_UEP4_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
}

/**
 * Same as DevEPx_IN_Deal
*/
inline void UEP_TRIG_D2H(uint8_t nEP, uint8_t nTxLen)
{
	if(1 == nEP)
	{
		R8_UEP1_T_LEN = nTxLen;
		R8_UEP1_CTRL = R8_UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
	else if(2 == nEP)
	{
		R8_UEP2_T_LEN = nTxLen;
		R8_UEP2_CTRL = R8_UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
	else if(3 == nEP)
	{
		R8_UEP3_T_LEN = nTxLen;
		R8_UEP3_CTRL = R8_UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
	else if(4 == nEP)
	{
		R8_UEP4_T_LEN = nTxLen;
		R8_UEP4_CTRL = R8_UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
}

/**
 * Just send response, NOT resource free.
*/
inline void UEP_RESP_H2D(uint8_t nEP)
{
	if(1 == nEP)
	{
		R8_UEP1_CTRL = (R8_UEP1_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
	else if (2 == nEP)
	{
		R8_UEP2_CTRL = (R8_UEP2_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
	else if(3 == nEP)
	{
		R8_UEP3_CTRL = (R8_UEP3_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
	else if(4 == nEP)
	{
		R8_UEP4_CTRL = (R8_UEP4_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
}

inline void UEP_DONE_H2D(uint8_t nEP)
{
	if(1 == nEP)
	{
		R8_UEP1_CTRL = R8_UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
	else if(2 == nEP)
	{
		R8_UEP2_CTRL = R8_UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
	else if(3 == nEP)
	{
		R8_UEP3_CTRL = R8_UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
	else if(4 == nEP)
	{
		R8_UEP4_CTRL = R8_UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
}

inline uint16_t uart_fifo_length(RingBuf* pstFifo)
{
	return pstFifo->nCnt;
}

inline uint8_t uart_fifo_is_empty(RingBuf* pstFifo)
{
	return (0 == pstFifo->nCnt);
}

// Called from ISR ONLY.
inline uint8_t uart_fifo_is_full(RingBuf* pstFifo)
{
	return (uart_fifo_length(pstFifo) == UART_FIFO_SIZE);
}

// Called from ISR ONLY.
inline void uart_fifo_put(RingBuf* pstFifo, uint8_t nData)
{
	if (!uart_fifo_is_full(pstFifo))
	{
		pstFifo->aRing[pstFifo->nWrIdx] = nData;
		pstFifo->nWrIdx = (pstFifo->nWrIdx + 1) % UART_FIFO_SIZE;
		pstFifo->nCnt++;
	}
}

inline void uart_fifo_get_without_check(RingBuf* pstFifo, uint8_t* pData)
{
	while(uart_fifo_is_empty(pstFifo));
//	EA = 0;
	*pData = pstFifo->aRing[pstFifo->nRdIdx];
	pstFifo->nRdIdx = (pstFifo->nRdIdx + 1) % UART_FIFO_SIZE;
	pstFifo->nCnt--;
//	EA = 1;
}


typedef struct _CdcDev
{
	uint8_t		anLineCoding[7];	// CDC configuration information.
	uint8_t		bRunD2H;
	uint8_t		nH2DSize;
	uint8_t		nNext2Uart;			// index pointing char in EP. (to out via UART)
	uint8_t		nEP;
	uint8_t*	pInBuf;
	uint8_t*	pOutBuf;
	RingBuf*	pFifo;
} CdcDev;

static CdcDev gaCdc[NUM_CDC] = 
{
	{ // A,
		.anLineCoding = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 },
		.bRunD2H = 0,
		.nH2DSize = 0,
		.nNext2Uart = 0,
		.nEP = 2,
		.pInBuf = gaBuf4EP2,
		.pOutBuf = gaBuf4EP2 + EP_SIZE_BULK,
		.pFifo = &gUartFifoA,
	},
	{ // B,
		.anLineCoding = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 },
		.bRunD2H = 0,
		.nH2DSize = 0,
		.nNext2Uart = 0,
		.nEP = 3,
		.pInBuf = gaBuf4EP3,
		.pOutBuf = gaBuf4EP3 + EP_SIZE_BULK,
		.pFifo = &gUartFifoB,
	},
};

/*******************************************************************************
* Description    : USB device mode endpoint configuration, simulation compatible HID device, in addition to endpoint 0 control transmission, also includes endpoint 2 batch upload
*******************************************************************************/

uint32_t get_baud_rate(uint8_t* acm_line_code)
{
	uint32_t nBaudRate = 0;

	nBaudRate |= acm_line_code[3];
	nBaudRate <<= 8;
	nBaudRate |= acm_line_code[2];
	nBaudRate <<= 8;
	nBaudRate |= acm_line_code[1];
	nBaudRate <<= 8;
	nBaudRate |= acm_line_code[0];
	return nBaudRate;
}

typedef struct _XferCtx
{
	uint8_t* pXferAddr;	// Next buffer address to xfer next turn.
	uint16_t nXferSize;	// Rest xfer size.
	uint8_t	nCfg;		// Temporary CFG value.
	uint8_t	nUsbAddr;	// Address of this device.
} XferCtx;

XferCtx gXferCtx = {NULL, 0, 0, 0};

USB_SETUP_REQ gstUsbLastSetupReq;	// Previous Setup request copy.

void handleIrqReset(void)
{
	R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

	R8_USB_DEV_AD = 0;
	R8_USB_INT_FG |= RB_UIF_BUS_RST | RB_UIF_TRANSFER | RB_UIF_SUSPEND;
	gXferCtx.nCfg = 0;
}

/***
 * Handle data sending.
 */
const uint8_t* gaNum2Hex = "0123456789ABCDEF";
void handleIrqD2H(uint8_t nEP, uint8_t nDbgVal)
{
	HAL_DbgLog("\tD->H: %d\n", nEP);

	if(0 == nEP)
	{
		switch(gstUsbLastSetupReq.bRequest)
		{
			case USB_GET_DESCRIPTOR:
			{
				uint16_t nTxLen = gXferCtx.nXferSize;

				if (0 == nTxLen)
				{
					/* nothing need sending, force ending setup transfer */
					R8_UEP0_T_LEN = 0;
					R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				}
				else if (nTxLen > EP_SIZE_INT)
				{
					nTxLen = EP_SIZE_INT;
				}

				memcpy(gaBuf4EP0, gXferCtx.pXferAddr, nTxLen);
				R8_UEP0_T_LEN = nTxLen;
				R8_UEP0_CTRL ^= RB_UEP_T_TOG;	// Re-Transfer.
//				R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				gXferCtx.pXferAddr += nTxLen;
				gXferCtx.nXferSize -= nTxLen;

				HAL_DbgLog("\tS D2H %d\n", nTxLen);

				break;
			}
			case USB_SET_ADDRESS:
			{
				R8_USB_DEV_AD = (R8_USB_DEV_AD & RB_UDA_GP_BIT) | gXferCtx.nUsbAddr;
				R8_UEP0_T_LEN = 0;
				R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			default:
			{
				R8_UEP0_T_LEN = 0;
				R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
		}
	}
	else if(2 == nEP)
	{
		UEP_DONE_D2H(nEP);
		gaCdc[0].bRunD2H = 0;
#if (EN_DBG == 1)
		uart_fifo_put(&gUartFifoB, '-');
		uart_fifo_put(&gUartFifoB, gaNum2Hex[nDbgVal / 16]);
		uart_fifo_put(&gUartFifoB, gaNum2Hex[nDbgVal % 16]);
		uart_fifo_put(&gUartFifoB, '\n');
#endif
	}
	else if(3 == nEP)
	{
		UEP_DONE_D2H(nEP);
		gaCdc[1].bRunD2H = 0;
	}
}


/**
 * Handle data receiving.
 **/
void handleIrqH2D(uint8_t nEP)
{
	CdcDev *pstCdc = NULL;
	uint8_t bmInt = R8_USB_INT_FG;
	HAL_DbgLog("\tH->D: %d\n", nEP);

	if (bmInt & RB_U_TOG_OK)  /* Out of sync packets will be dropped. */
	{
		if(0 == nEP)
		{
//			HAL_DbgLog("Setup Cont 0: %X\n", gstUsbLastSetupReq.bRequest);
			switch (gstUsbLastSetupReq.bRequest)
			{
				case SET_LINE_CODING:
				{
					if ((gstUsbLastSetupReq.wIndex & 0xFF) == 0) /* interface 0 is CDC0 */
					{
						pstCdc = &gaCdc[0];
						uint32_t baud = get_baud_rate(gaBuf4EP0);
//						TH1 = 256 - FREQ_SYS / baud / 16;
					}
					else if ((gstUsbLastSetupReq.wIndex & 0xFF) == 2) /* interface 2 is CDC1 */
					{
						pstCdc = &gaCdc[1];
						uint32_t baud = get_baud_rate(gaBuf4EP0);
//						SBAUD1 = 256 - FREQ_SYS / 16 / baud;
					}
					else if ((gstUsbLastSetupReq.wIndex & 0xFF) == 4) /* interface 2 is CDC2 */
					{
						pstCdc = &gaCdc[2];
					}

					if (NULL != pstCdc)
					{
						memcpy(pstCdc->anLineCoding, gaBuf4EP0, R8_USB_RX_LEN);
						R8_UEP0_T_LEN = 0;
						R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;
					}
					else
					{
						R8_UEP0_T_LEN = 0;
						R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
					}
					break;
				}
				default:
				{
					R8_UEP0_T_LEN = 0;
					R8_UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				}
			}
		}
		else if(2 == nEP)
		{
			gaCdc[0].nH2DSize = R8_USB_RX_LEN;
			UEP_RESP_H2D(nEP);
		}
		else if(3 == nEP)
		{
			gaCdc[1].nH2DSize = R8_USB_RX_LEN;
			UEP_RESP_H2D(nEP);
		}
	}
}

/**
 * Get Descriptor information into gpSetupXferPoint/gnSetupXferSize
 **/
int parepareDesc(uint8_t nDescType, uint8_t nStrIdx)
{
	const uint8_t *pstSrc = NULL;
	int nSize = 0;

	switch (nDescType)
	{
		case USB_DEVICE_DESCRIPTOR_TYPE: /* device descriptor */
		{
			pstSrc = gDevDesc_CDC_ACM2;
			nSize = sizeof(gDevDesc_CDC_ACM2);
			break;
		}
		case USB_CONFIGURATION_DESCRIPTOR_TYPE: /* config descriptor */
		{
			pstSrc = gCfgDesc_CDC_ACM2;
			nSize = sizeof(gCfgDesc_CDC_ACM2);
			break;
		}
		case USB_STRING_DESCRIPTOR_TYPE: /* string descriptor */
		{
			switch (nStrIdx)
			{
				case IDX_LANGUAGE_DESCRIPTOR_TYPE:
					pstSrc = gLangDesc;
					nSize = sizeof(gLangDesc);
					break;
				case IDX_MANUFACTURER_DESCRIPTOR_TYPE: /* iManufacturer */
					pstSrc = gszManuDesc;
					nSize = sizeof(gszManuDesc);
					break;
				case IDX_PRODUCT_DESCRIPTOR_TYPE:  /* iProduct */
					pstSrc = gszProdDesc;
					nSize = sizeof(gszProdDesc);
					break;
				case IDX_SERIAL_DESCRIPTOR_TYPE: /* iSerial */
					pstSrc = gszSerialDesc;
					nSize = sizeof(gszSerialDesc);
					break;
				default:
					return -1;
			}
			break;
		}
		default:
			return -1;
	}

	// 꼭 필요함.
	if(nSize < gXferCtx.nXferSize)
	{
		gXferCtx.nXferSize = (uint16_t)nSize;
	}
	gXferCtx.pXferAddr = pstSrc;

	return 0;
}

int handleSetupStd(USB_SETUP_REQ* pstReq)
{
	int nRet = 0;
	switch (pstReq->bRequest)
	{
		case USB_GET_DESCRIPTOR:
			nRet = parepareDesc(pstReq->wValue >> 8, pstReq->wValue & 0xFF);
			break;
		case USB_SET_ADDRESS:
			/* new address is addressed after device ACK */
			gXferCtx.nUsbAddr = pstReq->wValue & 0xFF;
			gXferCtx.pXferAddr = NULL;
			gXferCtx.nXferSize = 0;
			break;
		case USB_GET_CONFIGURATION:
			gXferCtx.nXferSize = sizeof(gXferCtx.nCfg);
			gXferCtx.pXferAddr = &gXferCtx.nCfg;
			break;
		case USB_SET_CONFIGURATION:
			gXferCtx.nCfg = pstReq->wValue & 0xFF;
			gXferCtx.pXferAddr = NULL;
			gXferCtx.nXferSize = 0;
			break;
		case USB_GET_INTERFACE:
			break;
		default:
			nRet = -1;
	}

	HAL_DbgLog("Std Desc (%X, %X, %X) -> %d\n",
			pstReq->bRequest, pstReq->wValue >> 8, pstReq->wValue & 0xFF,
			gXferCtx.nXferSize);

	return nRet;
}

int handleSetupVendor(USB_SETUP_REQ* pstReq)
{
	switch (pstReq->bRequest)
	{
		HAL_DbgLog("SV: %X\n", pstReq->bRequest);

		case GET_LINE_CODING:
		{
			if((pstReq->wIndex & 0xFF) <= 4)
			{
				int nCdcId = (pstReq->wIndex & 0xFF) / 2;
				gXferCtx.pXferAddr = gaCdc[nCdcId].anLineCoding;
//				gXferCtx.nXferSize = sizeof(gaCdc[nCdcId].anLineCoding);
			}
			else
			{
				return -1;
			}
			break;
		}
		case SET_CONTROL_LINE_STATE:
		case SET_LINE_CODING:
			gXferCtx.pXferAddr = NULL;
			gXferCtx.nXferSize = 0;
			/* setting data packet defined in EP0 packet out */
			break;
		default:
			return -1;
	}

	HAL_DbgLog("Ven Desc (%X, %X) -> %d\n",
			pstReq->bRequest, pstReq->wIndex, gXferCtx.nXferSize);

	return 0;
}

/**
 * Setup handler called only for EP0
 **/
void handleIrqSetup(uint8_t nEP)
{
	HAL_DbgLog("Rcv Setup: %d, %d\n", R8_USB_RX_LEN, sizeof(USB_SETUP_REQ));
	// 80, 6, 100, 0, 40
	if (0 != nEP)
	{
		return;
	}

	USB_SETUP_REQ* pstReq = (USB_SETUP_REQ*)gaBuf4EP0;
	int bFailed = -1;

	if(R8_USB_RX_LEN == (sizeof(USB_SETUP_REQ)))
//	if(0 != R8_USB_RX_LEN)
	{
		gXferCtx.nXferSize = (uint16_t)pstReq->wLength;
		memcpy(&gstUsbLastSetupReq, pstReq, sizeof(gstUsbLastSetupReq));

		if ((pstReq->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_STANDARD)
		{
			bFailed = handleSetupStd(pstReq);
		}
		else
		{
			bFailed = handleSetupVendor(pstReq);
		}
	}

	if (bFailed  || (gXferCtx.nXferSize > 0 && (!gXferCtx.pXferAddr)))) /* STALL request */
	{
		R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
		return;
	}

	int nTxSize = (gXferCtx.nXferSize > EP_SIZE_INT) ?
			EP_SIZE_INT :
			gXferCtx.nXferSize;

	memcpy(gaBuf4EP0, gXferCtx.pXferAddr, nTxSize);

	if((0 == gXferCtx.nXferSize) && (nTxSize < EP_SIZE_INT))
	{
		gXferCtx.pXferAddr = NULL;
	}
	HAL_DbgLog("\tS D2H %d\n", nTxSize);

	gXferCtx.pXferAddr += nTxSize;
	gXferCtx.nXferSize -= nTxSize;
	R8_UEP0_T_LEN = nTxSize;
	R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;


}

void HandleUSB(void)
{
	uint8_t bmIntFlag = R8_USB_INT_FG;
	// 0x2A : SIE_FREE | HST_SOF | TRANSFER,
//	HAL_DbgLog("IF: %X, IS:%X --> ", bmIntFlag, R8_USB_INT_ST);
	
	if (bmIntFlag & RB_UIF_TRANSFER) // Transfer done (partial or full)
	{
		uint8_t nDbg = R8_USB_MIS_ST;
		uint8_t nIntSt = R8_USB_INT_ST;
		uint8_t nEP = nIntSt & MASK_UIS_ENDP;

		switch (nIntSt & MASK_UIS_TOKEN)
		{
			case UIS_TOKEN_SETUP:
				handleIrqSetup(nEP);
				break;
			case UIS_TOKEN_IN:
				handleIrqD2H(nEP, nDbg);
				break;
			case UIS_TOKEN_OUT:
				handleIrqH2D(nEP);
				break;
		}
		R8_USB_INT_FG = RB_UIF_TRANSFER; /* clear interrupt */
	}
	else if (bmIntFlag & RB_UIF_BUS_RST)
	{
		HAL_DbgLog("Reset\n");
		handleIrqReset();
	}
	else if (bmIntFlag & RB_UIF_SUSPEND)
	{
		HAL_DbgLog("Suspend\n");
		R8_USB_INT_FG = RB_UIF_SUSPEND;
	}
	else
	{
		R8_USB_INT_FG = 0xFF;
	}
}

__attribute__((interrupt("machine")))
__attribute__((section(".highcode")))
void USB_IRQHandler(void)
{
	HandleUSB();
}

void uart0_isr_call(void)
{
#if 0
	if (RI)
	{
		uint8_t nRcv = SBUF;
		uart_fifo_put(&gUartFifoA, nRcv);
#if (EN_SNIFF == 1)
		uart_fifo_put(&gUartFifoSniff, nRcv);
#endif
		RI = 0;
	}
#endif
}

void uart1_isr_call(void)
{
#if 0
	if (U1RI)
	{
		uint8_t nRcv = SBUF1;
		uart_fifo_put(&gUartFifoB, nRcv);
		U1RI = 0;
	}
#endif
}

void handle_cdc(int nCdcId, CdcDev* pCdc)
{
	static uint8_t anTimeChk[NUM_CDC] = {0,0};

	int nTxLen;
	uint8_t* aOutBuf = pCdc->pOutBuf;
	uint8_t* aInBuf = pCdc->pInBuf;

	nTxLen = uart_fifo_length(pCdc->pFifo);
	if ((nTxLen > 0) && (0 == pCdc->bRunD2H))
	{
		if ((anTimeChk[nCdcId] > 20)
			|| (nTxLen >= (EP_SIZE_BULK / 2)))
		{
			uint8_t nOrgLen = nTxLen;
			if(nTxLen > EP_SIZE_BULK - 1)
			{
				nTxLen = EP_SIZE_BULK - 1;
			}

			for (int i = 0; i < nTxLen; i++)
			{
				uint8_t cRcv;
				uart_fifo_get_without_check(pCdc->pFifo, &cRcv);
				aOutBuf[i] = cRcv;
			}
			pCdc->bRunD2H = 1;
#if 0
			if(1 == pCdc->nEP)
			{
				uart_fifo_put(&gUartFifoSniff, ',');
				uart_fifo_put(&gUartFifoSniff, 'S');
				uart_fifo_put(&gUartFifoSniff, '0' + nTxLen / 10);
				uart_fifo_put(&gUartFifoSniff, '0' + nTxLen % 10);
			}
#endif
#if (EN_DBG == 1) // debug
			if(0 == nCdcId)
			{
				aOutBuf[0] = '#';
				uart_fifo_put(&gUartFifoB, '0' + nTxLen / 10);
				uart_fifo_put(&gUartFifoB, '0' + nTxLen % 10);
			}
#endif
			UEP_TRIG_D2H(pCdc->nEP, nTxLen);
			anTimeChk[nCdcId] = 0;
		}
		else
		{
			anTimeChk[nCdcId]++;
		}
	}

	if (pCdc->nH2DSize > 0)
	{
		uint8_t nData = aInBuf[pCdc->nNext2Uart++];
#if 0
		if(0 == nCdcId)
		{
			CH554UART0SendByte(nData);
#if (EN_SNIFF == 1)
			uart_fifo_put(&gUartFifoB, nData);
#endif
		}
		else if(1 == nCdcId)
		{
			CH554UART1SendByte(nData);
		}
#endif
		if (--pCdc->nH2DSize == 0)
		{
			pCdc->nNext2Uart = 0;
			/* gstCdc0: ready, continue recving */
			UEP_DONE_H2D(pCdc->nEP);
		}
	}	
}

void CDC_Init()
{
	R8_USB_CTRL = 0x00; // 先设定模式,取消 RB_UC_CLR_ALL

	R8_UEP4_1_MOD = RB_UEP4_RX_EN | RB_UEP4_TX_EN | RB_UEP1_RX_EN | RB_UEP1_TX_EN; // 端点4 OUT+IN,端点1 OUT+IN
	R8_UEP2_3_MOD = RB_UEP2_RX_EN | RB_UEP2_TX_EN | RB_UEP3_RX_EN | RB_UEP3_TX_EN; // 端点2 OUT+IN,端点3 OUT+IN

	R16_UEP0_DMA = (uint16_t)(uint32_t)pEP0_RAM_Addr;
	R16_UEP1_DMA = (uint16_t)(uint32_t)pEP1_RAM_Addr;
	R16_UEP2_DMA = (uint16_t)(uint32_t)pEP2_RAM_Addr;
	R16_UEP3_DMA = (uint16_t)(uint32_t)pEP3_RAM_Addr;

	R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
	R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
	R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK | RB_UEP_AUTO_TOG;
	R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;

	R8_USB_DEV_AD = 0x00;
	R8_USB_CTRL = RB_UC_DEV_PU_EN | RB_UC_INT_BUSY | RB_UC_DMA_EN; // 启动USB设备及DMA，在中断期间中断标志未清除前自动返回NAK
	R16_PIN_ANALOG_IE |= RB_PIN_USB_IE | RB_PIN_USB_DP_PU;         // 防止USB端口浮空及上拉电阻
	R8_USB_INT_FG = 0xFF;                                          // 清中断标志
	R8_UDEV_CTRL = RB_UD_PD_DIS | RB_UD_PORT_EN;                   // 允许USB端口
	R8_USB_INT_EN = RB_UIE_SUSPEND | RB_UIE_BUS_RST | RB_UIE_TRANSFER;
}

void DebugInit(void)
{
	GPIOA_SetBits(GPIO_Pin_9);
	GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);
	GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA);
	UART1_DefInit();
}

void main()
{
	SetSysClock(CLK_SOURCE_PLL_60MHz);
	DebugInit();
	HAL_DbgLog("\n\nSTART...\n");

	pEP0_RAM_Addr = gaBuf4EP0;
	pEP1_RAM_Addr = gaBuf4EP1;
	pEP2_RAM_Addr = gaBuf4EP2;
	pEP3_RAM_Addr = gaBuf4EP3;

	USB_DeviceInit();
	PFIC_EnableIRQ(USB_IRQn);

	while(1)
	{
		mDelaymS(10);
		if(gXferCtx.nCfg)
		{
	//		handle_cdc(0, &gaCdc[0]);
	//		handle_cdc(1, &gaCdc[1]);
		}
	}
}

