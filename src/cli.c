#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "macro.h"
#include "sched.h"
#include "util.h"
#include "cli.h"
#include "hal.h"
//////////////////////////////////
#define LEN_LINE            (64)
#define MAX_CMD_COUNT       (16)
#define MAX_ARG_TOKEN       (8)

#define COUNT_LINE_BUF       (8)

/**
 * history.
*/
char gaHistBuf[COUNT_LINE_BUF][LEN_LINE];
uint8 gnPrvHist;
uint8 gnHistRef;

typedef struct _CmdInfo 
{
	char* szCmd;
	CmdHandler* pHandle;
} CmdInfo;

CmdInfo gaCmds[MAX_CMD_COUNT];
uint8 gnCmds;

void cli_RunCmd(char* szCmdLine);

void cli_CmdHelp(uint8 argc, char* argv[])
{
	if(argc > 1)
	{
		uint32 nNum = UT_GetInt(argv[1]);
		UT_Printf("help with %08lx\n", nNum);
		char* aCh = (char*)&nNum;
		UT_Printf("help with %02X %02X %02X %02X\n", aCh[0], aCh[1], aCh[2], aCh[3]);
	}
	else
	{
		for(uint8 nIdx = 0; nIdx < gnCmds; nIdx++)
		{
			UT_Printf("%d: %s\n", nIdx, gaCmds[nIdx].szCmd);
		}
	}
}


void cli_TrigEcall(uint8 argc, char* argv[])
{
	UNUSED(argc);
	UNUSED(argv);
	RV_ecall(0,0,0,0);
}

void cli_CmdHistory(uint8 argc, char* argv[])
{
	if(argc > 1) // Run the indexed command.
	{
		uint8 nSlot = UT_GetInt(argv[1]);
		if(nSlot < COUNT_LINE_BUF)
		{
			if(strlen(gaHistBuf[nSlot]) > 0)
			{
				cli_RunCmd(gaHistBuf[nSlot]);
			}
		}
	}
	else // 
	{
		uint8 nIdx = gnPrvHist;
		do
		{
			nIdx = (nIdx + 1) % COUNT_LINE_BUF;
			if(strlen(gaHistBuf[nIdx]) > 0)
			{
				UT_Printf("%2d> %s\n", nIdx, gaHistBuf[nIdx]);
			}

		} while(nIdx != gnPrvHist);
	}
}

void cbf_RxUart(uint8 tag, uint8 result)
{
	UNUSED(tag);
	UNUSED(result);
	Sched_TrigAsyncEvt(BIT(EVT_UART));
}

void lb_NewEntry(char* szCmdLine)
{
	if(0 != strcmp(gaHistBuf[gnPrvHist], szCmdLine))
	{
		gnPrvHist = (gnPrvHist + 1) % COUNT_LINE_BUF;
		strcpy(gaHistBuf[gnPrvHist], szCmdLine);
	}
	gnHistRef = (gnPrvHist + 1) % COUNT_LINE_BUF;
}

uint8 lb_GetNextEntry(bool bInc, char* szCmdLine)
{
	uint8 nStartRef = gnHistRef;
	szCmdLine[0] = 0;
	int nGab = (true == bInc) ? 1 : -1;
	do
	{
		gnHistRef = (gnHistRef + COUNT_LINE_BUF + nGab) % COUNT_LINE_BUF;
		if(strlen(gaHistBuf[gnHistRef]) > 0)
		{
			strcpy(szCmdLine, gaHistBuf[gnHistRef]);
			return strlen(szCmdLine);
		}
	} while(nStartRef != gnHistRef);
	return 0;
}

uint32 cli_Token(char* aTok[], char* pCur)
{
	uint32 nIdx = 0;
	if (0 == *pCur) return 0;
	while (1)
	{
		while (' ' == *pCur) // remove front space.
		{
			pCur++;
		}
		aTok[nIdx] = pCur;
		nIdx++;
		while ((' ' != *pCur) && (0 != *pCur)) // remove back space
		{
			pCur++;
		}
		if (0 == *pCur) return nIdx;
		*pCur = 0;
		pCur++;
	}
}

void cli_RunCmd(char* szCmdLine)
{
	char* aTok[MAX_ARG_TOKEN];
//	UART_SetCbf(NULL, NULL);
	uint8 nCnt = 0;
	nCnt = cli_Token(aTok, szCmdLine);
	bool bExecute = false;
	if(nCnt > 0)
	{
		for(uint8 nIdx = 0; nIdx < gnCmds; nIdx++)
		{
			if(0 == strcmp(aTok[0], gaCmds[nIdx].szCmd))
			{
				gaCmds[nIdx].pHandle(nCnt, aTok);
				bExecute = true;
				break;
			}
		}
	}

	if(false == bExecute)
	{
		UT_Printf("Unknown command: %s\n", szCmdLine);
	}
	UART_SetCbf(cbf_RxUart, NULL);
}

uint32 gnPrvECall;

void cli_Run(Evts bmEvt)
{
	UNUSED(bmEvt);
	static uint8 nLen = 0;
	static char aLine[LEN_LINE];
	char nCh;

	while(UART_RxD(&nCh))
	{
		if (' ' <= nCh && nCh <= '~')
		{
			UART_TxD(nCh);
			aLine[nLen] = nCh;
			nLen++;
		}
		else if(('\n' == nCh) || ('\r' == nCh))
		{
			if(nLen > 0)
			{
				aLine[nLen] = 0;
				UART_Puts("\n");
				lb_NewEntry(aLine);
				cli_RunCmd(aLine);
				nLen = 0;
			}
			UART_Puts("\n$> ");
		}
		else if(0x7F == nCh) // backspace.
		{
			if(nLen > 0)
			{
				UART_Puts("\b \b");
				nLen--;
			}
		}
		else if(0x1B == nCh) // Escape sequence.
		{
			char nCh2, nCh3;
			while(0 == UART_RxD(&nCh2));
			if(0x5B == nCh2) // direction.
			{
				while(0 == UART_RxD(&nCh3));
				if(0x41 == nCh3) // up.
				{
					nLen = lb_GetNextEntry(false, aLine);
					UART_Puts("\r                          \r->");
					if(nLen > 0) UART_Puts(aLine);
				}
				else if(0x42 == nCh3) // down.
				{
					nLen = lb_GetNextEntry(true, aLine);
					UART_Puts("\r                          \r+>");
					if(nLen > 0) UART_Puts(aLine);
				}
			}
		}
		else
		{
			UT_Printf("~ %X\n", nCh);
		}
	}

	Sched_Wait(BIT(EVT_UART) | BIT(EVT_ECALL_DONE), 0);
}

/////////////////////

void CLI_Register(char* szCmd, CmdHandler *pHandle)
{
	gaCmds[gnCmds].szCmd = szCmd;
	gaCmds[gnCmds].pHandle = pHandle;
	gnCmds++;
}

void CLI_RegUartEvt()
{
	UART_SetCbf(cbf_RxUart, NULL);
}
///////////////////////
void CLI_Init(void)
{
	UART_Init(UART_BPS);
	UART_SetCbf(cbf_RxUart, NULL);
	CLI_Register("help", cli_CmdHelp);
	CLI_Register("hist", cli_CmdHistory);
	Sched_Register(TID_CONSOLE, cli_Run);
}
