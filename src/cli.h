#pragma once

#include "types.h"
#include "sched.h"

#define UART_BPS        (19200UL)

typedef void (CmdHandler)(uint8 argc, char* argv[]);

void CLI_Init(void);
void CLI_Register(char* szCmd, CmdHandler *pHandle);
