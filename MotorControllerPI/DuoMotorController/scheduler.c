
/*
 * scheduler.c
 *
 * Created: 5/3/2023 5:11:23 PM
 *  Author: KASO
 */ 

#include "scheduler.h"

static task_t[SCHEDULER_TASK_QUEUE_SIZE] = {0};

void Task_InitScheduler()
{
	
}

void Task_StartScheduler()
{
	
	
}

void Task_StopScheduler()
{
	
	
}

task_t* Task_Create(uint16_t period_ms, task_callback_t callback, void* args)
{
	
}

void Task_Start(task_t* hTask)
{
	if (NULL == hTask)
	{
		return;
	}
	
	hTask->state = TASK_STATE_WAITING;
}

void Task_Stop(task_t* hTask)
{
	if (NULL == hTask)
	{
		return;
	}
	
	hTask->state = TASK_STATE_STOPPED;
}