/*
 * pid.h
 *
 * Created: 4/24/2023 8:58:17 PM
 *  Author: KASO
 */ 


#ifndef PID_H_
#define PID_H_


#ifndef bool
#define bool char
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

typedef enum {
	PID_ERROR_INVALID_HANDLE = -1,
	PID_ERROR_NO_ERROR = 0,
	PID_ERROR_TIMESTEP_ZERO_DIV = 1,
	PID_ERROR_TI_ZERO_DIV = 2,
	PID_ERROR_OUTPUT_MAX_MIN = 3
} pid_error_state_e;

typedef struct{
	/* STATE */
	float current_output;
	float previous_output;
	float current_error;
	float previous_error;
	float pre_previous_error;
	float current_time_delta;
	float current_integral_error;

	/* PARAMETERS */
	float Kp;
	float Td;
	float Ti;

	float output_max;
	float output_min;

	/* ERROR */
	bool bIsError;
	pid_error_state_e error_state;

} pid_t;

void PID_Init(pid_t* hPID, float Kp, float Td, float Ti, float output_min, float output_max);
float PID_Advance(pid_t* hPID, float timestep, float error);
bool PID_CheckError(pid_t* hPID, pid_error_state_e* p_error_container);
void PID_ClearAccumulatedValues(pid_t* hPID);


#endif /* PID_H_ */