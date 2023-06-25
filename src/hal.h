#pragma once
#include "types.h"


uint8 UART_RxD(char* pCh);
void UART_SetCbf(Cbf cbRx, Cbf cbTx);
void UART_Puts(char* szLine);
void UART_TxD(char nCh);
void UART_Init(uint32 nBPS);

void TIMER_Init(Cbf cbHandle);

void HAL_DbgLog(char* szLine, ...);
void HAL_DbgInit(void);

uint32 RV_ecall(uint32 nP0, uint32 nP1, uint32 nP2, uint32 nP3);
