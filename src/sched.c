#include "types.h"
#include "macro.h"
#include "sched.h"
#include "sched_conf.h"
#include "CH57x_common.h"

typedef struct
{
	Entry	pfTask;
	uint16  nTimeOut;
	Evts    bmWait;         // Events to wait...
} TaskInfo;

TaskInfo gaTasks[MAX_TASK];

uint8 gCurTask;		// Current running task.
TaskBtm gbRdyTask;	// Ready to run..
uint16 gnTick;

volatile Evts gbAsyncEvt;	// ISR���� �߻��� Event.
volatile uint16 gnAsyncTick;	// TickISR���� ++
//volatile uint32 gnCurTick;

void sched_TickISR(uint8 tag, uint8 result)
{
	gnAsyncTick ++;
//	gnCurTick ++;

//	static uint32 nIdx;
//	nIdx++;
}

/**
Get async event into scheduler.
This function called under interrupt disabled.
@return need to run.
*/
void sched_HandleAsync(Evts bmEvt, uint16 nTick)
{
	for(uint8 i = 0; i < NUM_TASK; i++)
	{
		if(gaTasks[i].bmWait & bmEvt)
		{
			gbRdyTask |= BIT(i);
			gaTasks[i].bmWait &= ~bmEvt;
			gaTasks[i].nTimeOut = 0;
		}
		else if((nTick > 0) && (gaTasks[i].nTimeOut > 0))
		{
			if(gaTasks[i].nTimeOut <= nTick)
			{
				gaTasks[i].nTimeOut = 0;
				gbRdyTask |= BIT(i);
			}
			else
			{
				gaTasks[i].nTimeOut -= nTick;
			}
		}
	}
}


void Sched_TrigAsyncEvt(Evts bmEvt)
{
	gbAsyncEvt |= bmEvt;
}


void Sched_TrigSyncEvt(Evts bmEvt)
{
	for(uint8 i=0; i < NUM_TASK; i++)
	{
		if(gaTasks[i].bmWait & bmEvt)
		{
			gbRdyTask |= BIT(i);
			gaTasks[i].bmWait &= ~bmEvt;
			gaTasks[i].nTimeOut = 0;
		}
	}
}

void Sched_Wakeup(uint8 nTaskID)
{
	gaTasks[nTaskID].bmWait = 0;
	gaTasks[nTaskID].nTimeOut = 0;
	gbRdyTask |= BIT(nTaskID);
}

void Sched_Wait(Evts bmEvt, uint16 nTime)
{
	if(0 == nTime && bmEvt == 0)
	{
		gbRdyTask |= BIT(gCurTask);
	}
	else
	{
		gaTasks[gCurTask].bmWait = bmEvt;
		gaTasks[gCurTask].nTimeOut = nTime;
	}
}

uint16 Sched_GetTick()
{
	return gnTick + gnAsyncTick;
}

void Sched_Register(TaskId nTaskID, Entry task) ///< Register tasks.
{
	TaskInfo* pTI = gaTasks + nTaskID;
	pTI->pfTask = task;
	gbRdyTask |= BIT(nTaskID);
}


Cbf Sched_Init()
{
	gbRdyTask = 0;
	gnTick = 0;
	return sched_TickISR;
}

void Sched_Run()
{
	Evts bmAsyncEvt;
	uint16 nTick;
//	__disable_interrupt();
	bmAsyncEvt = gbAsyncEvt;
	nTick = gnAsyncTick;
	gbAsyncEvt = 0;
	gnAsyncTick = 0;
//	__enable_interrupt();

	gnTick += nTick;
	sched_HandleAsync(bmAsyncEvt, nTick);

	TaskBtm bmRdy = gbRdyTask;
	gbRdyTask = 0;

	while (bmRdy)
	{
		if (BIT(gCurTask) & bmRdy)
		{
			bmRdy &= ~BIT(gCurTask);
			TaskInfo* pTask = gaTasks + gCurTask;
			pTask->bmWait = 0;

			pTask->pfTask(pTask->bmWait);
		}
		gCurTask = (gCurTask + 1) % MAX_TASK;
	}
}
