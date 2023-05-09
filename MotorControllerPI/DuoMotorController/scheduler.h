
/*
 * scheduler.h
 *
 * Created: 5/3/2023 5:04:09 PM
 *  Author: KASO
 */ 

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#define SCHEDULER_TASK_QUEUE_SIZE 5

typedef void (*task_callback_t)(void*);

typedef enum {
	TASK_STATE_STOPPED = 0,
	TASK_STATE_READY = 1,
	TASK_STATE_WAITING = 2
} task_state_e;

typedef struct {
	uint16_t period_ms;
	task_state_e state;
	task_callback_t callback;
	void* args;
} task_t;

void Task_InitScheduler();
void Task_StartScheduler();
void Task_StopScheduler();
task_t* Task_Create(uint16_t period_ms, task_callback_t callback, void* args);
void Task_Start(task_t* hTask);
void Task_Stop(task_t* hTask);

#endif