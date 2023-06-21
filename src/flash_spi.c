
#include "CH573SFR.h"
#include "core_riscv.h"
#include "CH57x_gpio.h"
#include "CH57x_spi.h"
#include "macro.h"
#include "sched.h"
#include "cli.h"

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
uint8 gaBuf[PAGE_SIZE];

inline void _IssueCmd(uint8* aCmds, uint8 nLen)
{
	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(aCmds, nLen);
	GPIOA_SetBits(MAT_CS);
}

inline void _WriteEnable()
{
	uint8 nCmd = CMD_WREN;
	_IssueCmd(&nCmd, 1);
}

inline uint8 _WaitBusy()
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

void DumpData(uint8* aData, uint32 nByte)
{
	for(uint32 nIdx = 0; nIdx < nByte; nIdx++)
	{
		if(0 == (nIdx % 32)) CLI_Printf("\r\n");
		CLI_Printf("%02X ", aData[nIdx]);
	}
	CLI_Printf("\r\n");
}

void _ReadId()
{
	uint8 anCmd[4] = {CMD_READID,0,0,0};
	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(anCmd, 1);
	SPI0_MasterRecv(anCmd, 4);
	GPIOA_SetBits(MAT_CS);
	DumpData(anCmd, 4);
}

void _WriteStatus()
{
	uint8 anCmd[2];
	anCmd[0] = CMD_READWRSR;
	anCmd[1] = 0;
	_IssueCmd(anCmd, 2);
}

void _ReadProt(uint32 nAddr, bool bProtect)
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
	DumpData(&nResp, 1);
	CLI_Printf("Prot: %X, %X --> %d\r\n", nAddr, &nResp, bProtect);

	if((0 == bProtect) && (0xFF == nResp)) // unprotect.
	{
		_WriteEnable();
		anCmd[0] = CMD_UNPROTECT;
		_IssueCmd(anCmd, 4);
		CLI_Printf("Try unprotect\r\n");
	}
	else if((0 != bProtect) && (0xFF != nResp)) // try protect.
	{
		_WriteEnable();
		anCmd[0] = CMD_PROTECT;
		_IssueCmd(anCmd, 4);
		CLI_Printf("Try protect\r\n");
	}
}

void _Read(uint8* pBuf, uint32 nAddr, uint32 nByte)
{
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

uint8 _Write(uint8* pBuf, uint32 nAddr, uint32 nByte)
{
	_WriteEnable();

	uint8 anCmd[4];
	anCmd[0] = CMD_PGM;
	anCmd[1] = (nAddr >> 16) & 0xFF;
	anCmd[2] = (nAddr >> 8) & 0xFF;
	anCmd[3] = nAddr & 0xFF;

	GPIOA_ResetBits(MAT_CS);
	SPI0_MasterTrans(anCmd, 4);
	SPI0_MasterTrans(pBuf, nByte);
	GPIOA_SetBits(MAT_CS);

	// Wait done...
	return _WaitBusy();
}

uint8 _Erase(uint32 nInAddr, uint32 nInByte)
{
	uint32 nEAddr = ALIGN_DN(nInAddr + nInByte, SECT_SIZE);
	uint32 nAddr = ALIGN_UP(nInAddr, SECT_SIZE);
	uint8 anCmd[4] = {CMD_ERASE,};
	uint8 nResp;
	while(nEAddr >= nAddr)
	{
		CLI_Printf("ERS: %X\r\n", nAddr);

		_WriteEnable();

		anCmd[1] = (nAddr >> 16) & 0xFF;
		anCmd[2] = (nAddr >> 8) & 0xFF;
		anCmd[3] = nAddr & 0xFF;

		_IssueCmd(anCmd, 4);
		nResp = _WaitBusy();

		nAddr += SECT_SIZE;
	}
	return nResp;
}

void flash_Cmd(uint8 argc, char* argv[])
{
	if(argc < 2)
	{
		return;
	}

	// flash <cmd> <addr> <size> <opt>
	char nCmd = argv[1][0];
	uint32 nAddr = 0;
	uint32 nByte = 0;

	if(argc >= 3)
	{
		nAddr = CLI_GetInt(argv[2]);
	}
	if(argc >= 4)
	{
		nByte = CLI_GetInt(argv[3]);
	}

	if (nCmd == 'i')
	{
		_ReadId();
	}
	else if (nCmd == 'p' && argc >= 3)
	{
		_ReadProt(nAddr, nByte);
	}
	else if (nCmd == 'r' && argc >= 4) // r 8
	{
		nAddr = ALIGN_DN(nAddr, PAGE_SIZE);
		CLI_Printf("SPI Read: %X, %d\r\n", nAddr, nByte);
		while(nByte > 0)
		{
			uint32 nThis = nByte > PAGE_SIZE ? PAGE_SIZE : nByte;
			_Read(gaBuf, nAddr, nThis);
			CLI_Printf("SPI Read: %X, %d\r\n", nAddr, nThis);
			DumpData(gaBuf, nThis);
			nAddr += PAGE_SIZE;
			nByte -= PAGE_SIZE;
		}
	}
	else if(nCmd == 'w' && argc >= 4) // cmd w addr byte
	{
		uint8 nVal = (argc < 5) ? 0xAA : CLI_GetInt(argv[4]);

		memset(gaBuf, nVal, PAGE_SIZE);
		CLI_Printf("SPI Write: %X, %d, %X\r\n", nAddr, nByte, nVal);
		while(nByte > 0)
		{
			uint32 nThis = nByte > PAGE_SIZE ? PAGE_SIZE : nByte;
			uint8 nRet = _Write(gaBuf, nAddr, nThis);
			CLI_Printf("SPI Write: %X, %d --> %X\r\n", nAddr, nThis, nRet);
			nAddr += PAGE_SIZE;
			nByte -= PAGE_SIZE;
		}
	}
	else if(nCmd == 'e' && argc >= 4) //
	{
		uint8 nRet = _Erase(nAddr, nByte);
		CLI_Printf("SPI Erase: %X, %d --> %X\r\n", nAddr, nByte, nRet);
	}
	else
	{
		CLI_Printf("Wrong command\r\n");
	}
}

void FLASH_Init()
{
	GPIOA_SetBits(MAT_CS);
	GPIOA_ModeCfg(MAT_CS | MAT_CLK | MAT_DOUT, GPIO_ModeOut_PP_5mA);
	SPI0_MasterDefInit();
	SPI0_CLKCfg(0xe8);

	CLI_Register("flash", flash_Cmd);
//	Sched_Register(TID_DOT, dot_Run);
}

