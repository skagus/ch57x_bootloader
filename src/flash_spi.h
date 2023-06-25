#pragma once

#include "types.h"

void FLASH_Init(void);
uint8 FLASH_Erase(uint32 nInAddr, uint32 nInByte);
uint8 FLASH_Write(uint8* pBuf, uint32 nAddr, uint32 nByte);
void FLASH_Read(uint8* pBuf, uint32 nAddr, uint32 nByte);

