/*
 * pid.c
 *
 * Created: 4/24/2023 8:59:13 PM
 *  Author: KASO
 */ 

#include "pid.h"

#ifndef NULL
#define NULL (void*)0x00
#endif

#ifndef PRIVATE
#define PRIVATE static
#endif

PRIVATE void PID_SetError(pid_t* hPID, pid_error_state_e error)
{
	if(NULL == hPID)
	{
		return;
	}

	hPID->bIsError = TRUE;
	hPID->error_state = error;
}

void PID_Init(pid_t* hPID, float Kp, float Td, float Ti, float output_min, float output_max)
{
	if (NULL == hPID)
	{
		return;
	}

	hPID->current_output = 0;
	hPID->previous_output = 0;
	hPID->current_error = 0;
	hPID->previous_error = 0;
	hPID->pre_previous_error = 0;
	hPID->current_time_delta = 0;
	hPID->current_integral_error = 0;

	hPID->Kp = Kp;
	hPID->Td = Td;
	hPID->Ti = Ti;

	hPID->output_min = output_min;
	hPID->output_max = output_max;

	hPID->bIsError = FALSE;
	hPID->error_state = PID_ERROR_NO_ERROR;

	if(output_max <= output_min)
	{
		PID_SetError(hPID, PID_ERROR_OUTPUT_MAX_MIN);
	}

	if (Ti == 0)
	{
		PID_SetError(hPID, PID_ERROR_TI_ZERO_DIV);
	}
	
}

float PID_Advance(pid_t* hPID, float timestep, float error)
{
	if (NULL == hPID || hPID->bIsError)
	{
		return 0;
	}

	if(PID_CheckError(hPID, NULL))
	{
		return 0;
	}

	if (timestep == 0)
	{
		PID_SetError(hPID, PID_ERROR_TIMESTEP_ZERO_DIV);
	}

	hPID->current_time_delta = timestep;
	
	hPID->pre_previous_error = hPID->previous_error;
	hPID->previous_error = hPID->current_error;
	hPID->current_error = error;

	hPID->previous_output = hPID->current_output;

	float current_error_term = hPID->Kp * (1 + hPID->Ti * hPID->current_time_delta);
	float previous_error_term = -1 * hPID->Kp;

	hPID->current_output = current_error_term * hPID->current_error 
		+ previous_error_term * hPID->previous_error
		+ hPID->previous_output;

	if (hPID->current_output > hPID->output_max)
	{
		hPID->current_output = hPID->output_max;
	}
	
	if (hPID->current_output < hPID->output_min)
	{
		hPID->current_output = hPID->output_min;
	}

	return hPID->current_output;
}

bool PID_CheckError(pid_t* hPID, pid_error_state_e* p_error_container)
{
	if (NULL == hPID)
	{
		return TRUE;
	}
	else if (NULL == p_error_container)
	{
		return hPID->bIsError;
	}

	*p_error_container = hPID->error_state;
	return hPID->bIsError;
}