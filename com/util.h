#pragma once

#include "types.h"
void UT_DumpData(uint8* aData, uint32 nByte);
void UT_Printf(char* szFmt, ...);
uint32 UT_GetInt(char* szStr);
uint16 UT_Crc16(const uint8* aBuf, uint32 nLen);

