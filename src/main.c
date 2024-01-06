/********************************** (C) COPYRIGHT *******************************
 * File Name          : Main.c
 * Author             : WCH
 * Version            : V1.1
 * Date               : 2022/01/25
 * Description        : 친콰쇗휭HID구
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

#include "CH57x_common.h"
#include "version.h"
#include "sched.h"
#include "hal.h"
#include "util.h"
#include "cli.h"
#include "key.h"

int main(void)
{
	SetSysClock(CLK_SOURCE_PLL_60MHz);
	//	DelayMs(1000); // Wait 1 sec.

	HAL_DbgInit();
	Cbf cbfTick = Sched_Init();
	TIMER_Init(cbfTick);

	CLI_Init();
	KEY_Init();

	UT_Printf(VERSION);
	HAL_DbgLog("Hello\n");

	while(1)
	{
		Sched_Run();
	}
}

