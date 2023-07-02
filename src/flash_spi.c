
#include <string.h> // memset.
#include <stdio.h>
#include "CH573SFR.h"
#include "core_riscv.h"
#include "CH57x_gpio.h"
#include "CH57x_spi.h"
#include "macro.h"
#include "sched.h"
#include "util.h"
#include "cli.h"
#include "hal.h"
#include "ymodem.h"
#include "flash_spi.h"

#define MAT_CS				(GPIO_Pin_5)
#define MAT_CLK				(GPIO_Pin_13)
#define MAT_DOUT			(GPIO_Pin_14)
#define MAT_DIN				(GPIO_Pin_15)

#define SIZE_BUF				(16)

#define CMD_READ			0x03
#define CMD_ERASE			0x20
#define CMD_PGM				0x02
#define CMD_WREN			0x06
#define CMD_READWRSR		0x01
#define CMD_READRDSR		0x05
#define CMD_READ_PROT		0x3C
#define CMD_UNPROTECT		0x39
#define CMD_PROTECT			0x36
#define CMD_READID			0x9F

#define SECT_SIZE			(4096)
#define PAGE_SIZE			(256)
/**
 * For SPI DMA, data should be in SRAM..??
*/
#define DBG(...)			HAL_DbgLog(__VA_ARGS__)

inline void _IssueCmd(uint8* aCmds, uint8 nLen)
{
	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(aCmds, nLen);
	GPIOA_SetBits(MAT_CS);
}

inline void _WriteEnable(void)
{
	uint8 nCmd = CMD_WREN;
	_IssueCmd(&nCmd, 1);
}

inline uint8 _WaitBusy(void)
{
	uint8 nCmd = CMD_READRDSR;
	uint8 nResp = 0;
	GPIOA_ResetBits(MAT_CS);
	while(1)
	{
		SPI0_MasterTrans(&nCmd, 1);
		SPI0_MasterRecv(&nResp, 1);
		if(0 == (nResp & 0x01)) // check busy.
		{
			break;
		}
	}
	GPIOA_SetBits(MAT_CS);
	return nResp;
}


uint32 _ReadId(void)
{
	uint8 anCmd[4] = {CMD_READID,0,0,0};
	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(anCmd, 1);
	SPI0_MasterRecv(anCmd, 4);
	GPIOA_SetBits(MAT_CS);
	return *(uint32*)anCmd;
}

void _WriteStatus(void)
{
	uint8 anCmd[2];
	anCmd[0] = CMD_READWRSR;
	anCmd[1] = 0;
	_IssueCmd(anCmd, 2);
}

void _UnProtect(uint32 nAddr)
{
	uint8 anCmd[4];
	anCmd[0] = CMD_UNPROTECT;
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;
	_WriteEnable();
	_IssueCmd(anCmd, 4);
}

void _Protect(uint32 nAddr)
{
	uint8 anCmd[4];
	anCmd[0] = CMD_PROTECT;
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;
	_WriteEnable();
	_IssueCmd(anCmd, 4);
}

uint8 _ReadProt(uint32 nAddr)
{
	uint8 nResp;
	uint8 anCmd[4];
	anCmd[0] = CMD_READ_PROT;
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;

	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(anCmd, 4);
	SPI0_MasterRecv(&nResp, 1);
	GPIOA_SetBits(MAT_CS);

	return nResp;
}


uint8 _Erase(uint32 nSAddr)
{
	DBG("E:%X ", nSAddr);

	uint32 nAddr = ALIGN_UP(nSAddr, SECT_SIZE);
	uint8 anCmd[4] = {CMD_ERASE,};
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;

	_UnProtect(nAddr);
	_WriteEnable();
	_IssueCmd(anCmd, 4);
	uint8 nRet = _WaitBusy();
	DBG("->%X\n", nRet);
	return nRet;
}


uint8 _Write(uint8* pBuf, uint32 nAddr, uint32 nByte)
{
	DBG("W:%X,%d ", nAddr,nByte);

	uint8 anCmd[4];
	anCmd[0] = CMD_PGM;
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;

	_UnProtect(nAddr);
	_WriteEnable();
	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(anCmd, 4);
	SPI0_MasterTrans(pBuf, nByte);
	GPIOA_SetBits(MAT_CS);

	uint8 nRet = _WaitBusy();
	DBG("->%X\n", nRet);
	return nRet;
}

void _Read(uint8* pBuf, uint32 nAddr, uint32 nByte)
{
	DBG("R:%X,%d\n", nAddr,nByte);

	uint8 anCmd[4];
	anCmd[0] = CMD_READ;
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;

	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(anCmd, 4);
	SPI0_MasterRecv(pBuf, nByte);
	GPIOA_SetBits(MAT_CS);
}

void FLASH_Read(uint8* pBuf, uint32 nSAddr, uint32 nInLen)
{
	uint32 nAddr = nSAddr;
	uint32 nRest = nInLen;
	uint32 nLen = PAGE_SIZE - (nSAddr % PAGE_SIZE); // to fit align.
	if(nLen > nRest) nLen = nRest;
	while(nRest > 0)
	{
		_Read(pBuf, nAddr, nLen);
		pBuf += nLen;
		nAddr += nLen;
		nRest -= nLen;
		nLen = MIN(nRest, PAGE_SIZE);
	}
}

#include "CH57x_common.h"
uint8 FLASH_Write(uint8* pBuf, uint32 nSAddr, uint32 nInLen)
{
	uint8 nRet = 0xFF;
	uint32 nAddr = nSAddr;
	uint32 nRest = nInLen;
	uint32 nLen = MIN(PAGE_SIZE - (nSAddr % PAGE_SIZE), nRest); // to fit align.

	while(nRest > 0)
	{
		if(0 == (nAddr % SECT_SIZE))
		{
			nRet = _Erase(nAddr);
		}
		mDelaymS(100);
		nRet = _Write(pBuf, nAddr, nLen);
		pBuf += nLen;
		nAddr += nLen;
		nRest -= nLen;
		nLen = MIN(nRest, PAGE_SIZE);
	}
	return nRet;
}

uint8 FLASH_Erase(uint32 nSAddr, uint32 nLen)
{
	uint8 nRet = 0xFF;
	uint32 nAddr = ALIGN_UP(nSAddr, SECT_SIZE);
	uint32 nEAddr = ALIGN_DN(nSAddr + nLen, SECT_SIZE);
	while(nAddr < nEAddr)
	{
		nRet = _Erase(nAddr);
		nAddr += SECT_SIZE;
	}
	return nRet;
}

typedef struct _YmParam
{
	uint32 nAddr;
	uint32 nByte;
} YmParam;
YmParam gYParam;

bool _ProcWrite(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam)
{
	bool bRet = true;
	YmParam* pInfo = (YmParam*)pParam;
	switch(eStep)
	{
		case YS_META:
		{
			pInfo->nByte = *pnBytes;
			break;
		}
		case YS_DATA:
		{
			uint32 nLen = MIN(pInfo->nByte, *pnBytes);
			FLASH_Write(pBuf, pInfo->nAddr, nLen);
			pInfo->nAddr += nLen;
			pInfo->nByte -= nLen;
			break;
		}
		default:
		{
			break;
		}
	}
	return bRet;
}


bool _ProcRead(uint8* pBuf, uint32* pnBytes, YMState eStep, void* pParam)
{
	bool bRet = true;
	YmParam* pInfo = (YmParam*)pParam;
	switch(eStep)
	{
		case YS_META:
		{
			sprintf((char*)pBuf, "0x%X_fr.bin", (int)pInfo->nAddr);
			*pnBytes = pInfo->nByte;
			break;
		}
		case YS_DATA:
		{
			uint32 nLen = MIN(pInfo->nByte, *pnBytes);
			FLASH_Read(pBuf, pInfo->nAddr, nLen);
			pInfo->nAddr += nLen;
			pInfo->nByte -= nLen;
			break;
		}
		default:
		{
			break;
		}
	}
	return bRet;
}

void _printUsage(void)
{
	UT_Printf("\ti - show flash ID\n");
	UT_Printf("\te <Addr> <Size> - Erase flash\n");
	UT_Printf("\tr <Addr> <Size> - Read flash and show\n");
	UT_Printf("\tf <Addr> <Size> [Pat=0xAA] - Fill flash with pattern\n");
	UT_Printf("\tW <Addr> - Write received data (user Ymodem)\n");
	UT_Printf("\tR <Addr> <Size> - Read flash and send it via Ymodem\n");
}

void flash_Cmd(uint8 argc, char* argv[])
{
	if(argc < 2)
	{
		_printUsage();
		return;
	}
	
	uint8 aBuf[PAGE_SIZE];

	// flash <cmd> <addr> <size> <opt>
	char nCmd = argv[1][0];
	uint32 nAddr = 0;
	uint32 nByte = 0;

	if(argc >= 3)
	{
		nAddr = UT_GetInt(argv[2]);
	}
	if(argc >= 4)
	{
		nByte = UT_GetInt(argv[3]);
	}

	if (nCmd == 'i')
	{
		uint32 nId = _ReadId();
		UT_Printf("FLASH ID: %X\n", nId);
	}
	else if (nCmd == 'r' && argc >= 4) // r 8
	{
		nAddr = ALIGN_DN(nAddr, PAGE_SIZE);
		UT_Printf("Flash Read: %X, %d\n", nAddr, nByte);
		while(nByte > 0)
		{
			uint32 nThis = nByte > PAGE_SIZE ? PAGE_SIZE : nByte;
			FLASH_Read(aBuf, nAddr, nThis);
			UT_Printf("Read: %X, %d\n", nAddr, nThis);
			UT_DumpData(aBuf, nThis);
			nAddr += nThis;
			nByte -= nThis;
		}
	}
	else if(nCmd == 'f' && argc >= 4) // cmd w addr byte
	{
		uint8 nVal = (argc < 5) ? 0xAA : UT_GetInt(argv[4]);

		memset(aBuf, nVal, PAGE_SIZE);
		UT_Printf("FLASH Write: %X, %d, %X\n", nAddr, nByte, nVal);
		while(nByte > 0)
		{
			uint32 nThis = nByte > PAGE_SIZE ? PAGE_SIZE : nByte;
			uint8 nRet = FLASH_Write(aBuf, nAddr, nThis);
			UT_Printf("Page Write: %X, %d --> %X\n", nAddr, nThis, nRet);
			nAddr += PAGE_SIZE;
			nByte -= PAGE_SIZE;
		}
	}
	else if(nCmd == 'e' && argc >= 4) //
	{
		uint8 nRet = FLASH_Erase(nAddr, nByte);
		UT_Printf("FLASH Erase: %X, %d --> %X\n", nAddr, nByte, nRet);
	}
	else if(nCmd == 'W' && argc >= 3) // Write with Y modem.
	{
		gYParam.nAddr = nAddr;
		YReq stReq = {true, _ProcWrite, (void*)&gYParam};
		YM_Request(&stReq);
	}
	else if(nCmd == 'R' && argc >= 4) // Write with Y modem.
	{
		gYParam.nAddr = nAddr;
		gYParam.nByte = nByte;
		YReq stReq = {false, _ProcRead, (void*)&gYParam};
		YM_Request(&stReq);
	}
	else
	{
		_printUsage();
	}
}

void FLASH_Init(void)
{
	GPIOA_SetBits(MAT_CS);
	GPIOA_ModeCfg(MAT_CS | MAT_CLK | MAT_DOUT, GPIO_ModeOut_PP_5mA);
	SPI0_MasterDefInit();
	SPI0_CLKCfg(0xe8);

	CLI_Register("flash", flash_Cmd);
}

