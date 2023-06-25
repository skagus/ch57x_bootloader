#include <stdarg.h>
#include <stdio.h>
#include "types.h"
#include "macro.h"
#include "util.h"
#include "CH57x_common.h"
#include "sched.h"
#include "hal.h"
/**
 * 모든 HW dependancy를 여기에 넣는다.
*/
#define SIZE_BUF	(128)
volatile char gaRcv[SIZE_BUF];
volatile uint16 gnPush;
volatile uint16 gnPop;
Cbf gRxCbf;
Cbf gTxCbf;

uint8 UART_RxD(char* pCh)
{
	if(gnPop != gnPush)
	{
		*pCh = gaRcv[gnPop];
		gnPop = (gnPop + 1) % SIZE_BUF;
		return 1;
	}
	return 0;
}

void UART_SetCbf(Cbf cbRx, Cbf cbTx)
{
	gRxCbf = cbRx;
	gTxCbf = cbTx;
}

void UART_Puts(char* szLine)
{
	UART0_SendString((uint8*)szLine, strlen(szLine));
}

void UART_TxD(char nCh)
{
	UART0_SendByte(nCh);
}

void HAL_DbgLog(char* szFmt, ...)
{
	char aBuf[128];
	va_list arg_ptr;
	va_start(arg_ptr, szFmt);
	vsprintf(aBuf, szFmt, arg_ptr);
	va_end(arg_ptr);

	UART1_SendString((uint8*)aBuf, strlen(aBuf));
//	while(R8_UART1_TFC != 0);
}

void HAL_DbgInit(void)
{
	GPIOA_SetBits(GPIO_Pin_9); // for uart signal level.
//	GPIOA_ModeCfg(GPIO_Pin_8, GPIO_ModeIN_PU);      // RXD
	GPIOA_ModeCfg(GPIO_Pin_9, GPIO_ModeOut_PP_5mA); // TXD
	UART1_DefInit();
	R8_UART1_THR = UART_FIFO_SIZE;
}

void UART_Init(uint32 nBPS)
{
	UNUSED(nBPS);
	GPIOB_SetBits(GPIO_Pin_7); // for uart signal level.
	GPIOB_ModeCfg(GPIO_Pin_4, GPIO_ModeIN_PU);      // RXD
	GPIOB_ModeCfg(GPIO_Pin_7, GPIO_ModeOut_PP_5mA); // TXD
	UART0_DefInit(); // 115200, 
	UART0_ByteTrigCfg(UART_7BYTE_TRIG);
	UART0_INTCfg(ENABLE, RB_IER_RECV_RDY);
	PFIC_EnableIRQ(UART0_IRQn);
}

__attribute__((naked))
uint32 RV_ecall(uint32 nP0, uint32 nP1, uint32 nP2, uint32 nP3)
{
	UNUSED(nP0);UNUSED(nP1);UNUSED(nP2);UNUSED(nP3);
	asm volatile("ecall");
	asm volatile("ret");
}

#if defined(WCH_INT)
__attribute__((naked))
#else
__attribute__((interrupt("machine")))
#endif
__attribute__((section(".highcode")))
void SW_Handler(void)
{
	UART_Puts("In SWH\n");
	Sched_TrigAsyncEvt(BIT(EVT_ECALL_DONE));
}

#if defined(WCH_INT)
__attribute__((naked))
#else
__attribute__((interrupt("machine")))
#endif
__attribute__((section(".highcode")))
void UART0_IRQHandler(void)
{
	switch(UART0_GetITFlag())
	{
		case UART_II_LINE_STAT: // UART interrupt by receiver line status
		{
			UART0_GetLinSTA();
			break;
		}

		case UART_II_RECV_RDY: // UART interrupt by receiver data available
		case UART_II_RECV_TOUT: // UART interrupt by receiver fifo timeout
		{
			while(R8_UART0_RFC)
			{
				if(gnPop != ((gnPush + 1) % SIZE_BUF))
				{
					gaRcv[gnPush] = UART0_RecvByte();
					gnPush = (gnPush + 1) % SIZE_BUF;
				}
				else
				{	// Hang in ISR without pop.
					UART0_RecvByte(); // Trash.
				}
			}
			if(NULL != gRxCbf)
			{
				gRxCbf(0, 0);
			}
			break;
		}

		case UART_II_THR_EMPTY: // UART interrupt by THR empty
		case UART_II_MODEM_CHG: // UART interrupt by modem status change
			break;

		default:
			break;
	}
#if defined(WCH_INT)
	asm volatile("mret");
#endif
}


///////////////////////////////////////////////
Cbf gTickHdr;

void TIMER_Init(Cbf cbHandle)
{
	TMR0_TimerInit(FREQ_SYS / 100);		// 10 ms / tick.
	TMR0_ITCfg(ENABLE, TMR0_3_IT_CYC_END);
	PFIC_EnableIRQ(TMR0_IRQn);
	gTickHdr = cbHandle;
}


#if defined(WCH_INT)
__attribute__((naked))
#else
__attribute__((interrupt("machine")))
#endif
__attribute__((section(".highcode")))
void TMR0_IRQHandler(void) // TMR0 ��ʱ�ж�
{
	if(TMR0_GetITFlag(TMR0_3_IT_CYC_END))
	{
		TMR0_ClearITFlag(TMR0_3_IT_CYC_END); // ����жϱ�־
	}
	gTickHdr(0, 0);
#if defined(WCH_INT)
	asm("mret");
#endif
}


#if defined(WCH_INT)
__attribute__((naked))
#else
__attribute__((interrupt("machine")))
#endif
__attribute__((section(".highcode")))
void DEF_IRQHandler(void) // TMR0 ��ʱ�ж�
{
	unsigned nSrc;
	asm("csrr %0, mcause":"=r"(nSrc));
	while(1)
	{
		HAL_DbgLog("DBG mcause:%X\n", nSrc);
	}
#if defined(WCH_INT)
	asm("mret");
#endif
}


#if defined(WCH_INT)
__attribute__((naked))
#else
__attribute__((interrupt("machine")))
#endif
__attribute__((section(".highcode")))
void EXC_IRQHandler(unsigned nParam0, unsigned nParam1, unsigned nParam2, unsigned nParam3)
{
	unsigned nSrc;
	asm("csrr %0, mcause":"=r"(nSrc));
	switch(nSrc)
	{
		case 0:		// Inst addr misaligned.
		case 1:		// Inst Access fault.
		case 2:		// Illegal inst.
		case 3:		// Break point.
		case 4:		// Load addr misaligned.
		case 5:		// Load access fault.
		case 6:		// Store/AMO addr misaligned.
		case 7:		// Store/AMO access fault.
		default:
		{
			UT_Printf("Fault cause:%X\n", nSrc);
			break;
		}
		case 8:		// ecall from U mode.
		case 9:		// ecall from S mode.
		case 0xA:	// ecall from H mode.
		case 0xB:	// ecall from M mode.
		{
			unsigned nPC;
			asm volatile("csrr %0, mepc":"=r"(nPC));
			UT_Printf("ECall :%X, P:%X, %X, %X, %X\n",
						nSrc, nParam0, nParam1, nParam2, nParam3);
			nPC += 4;  // Inc PC because it is NOT fault.
			asm volatile("csrw mepc, %0"::"r"(nPC));
			// WCH Fast interrupt때문에 parameter return은 안됨. 
			unsigned nRet = 0xFABC;
			asm volatile("add a0, %0, zero"::"r"(nRet));
			break;
		}
	}

#if defined(WCH_INT)
	asm("mret");
#endif
}
