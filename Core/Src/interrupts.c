#include "stm32f4xx_ll_exti.h"
#include "driver_setup.h"
#include "hoverserial.h"

#define D_LOOP_TIME_MS 10



// loop for updating sensors data


void TIM1_UP_TIM10_IRQHandler()
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM10))
	{

		encoder_MeasureRPM(&enc_L, D_LOOP_TIME_MS);
		encoder_MeasureRPM(&enc_R, D_LOOP_TIME_MS);
		currentMeasurement_MeasueCurrent(&curr_mot_L);
		currentMeasurement_MeasueCurrent(&curr_mot_R);
		SafetyContoller_UpdateSafetyInfo(&safety);
		inputSourceSelector_SetPidInput(&input_source, &pid_L, &pid_R);


		LL_TIM_ClearFlag_UPDATE(TIM10);

	}
}

extern uint8_t driver_standby_flag, driver_configured;

// loop for running PID and actuating motors
void TIM1_TRG_COM_TIM11_IRQHandler()
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM11))
	{
		if(driver_configured && !driver_standby_flag)
		if(1)
		{
			//Receive(&byte);
			int32_t rpm_L = encoder_GetRPM(&enc_L);
			int32_t rpm_R = encoder_GetRPM(&enc_R);

			int32_t curr_L = currentMeasurement_GetCurrent(&curr_mot_L);
			int32_t curr_R = currentMeasurement_GetCurrent(&curr_mot_R);


			double pid_L_out = pid_ComputeOutput(&pid_L, rpm_L, curr_L);
			double pid_R_out = pid_ComputeOutput(&pid_R, rpm_R, curr_R);


			if(bldc == D_MOTOR_TYPE_BLDC){
				inputFormat_t sp = ppm_GetControlSignal(&ppm);

				Send(sp.rot_vel , sp.lin_vel );
			//	motor_Actuate_UART(pid_L_out,pid_R_out);
			}


			if(bldc == D_MOTOR_TYPE_STEPPER || bldc == D_MOTOR_TYPE_DC){
			motor_Actuate(&motor_L, pid_L_out);
			motor_Actuate(&motor_R, pid_R_out);
			}


		}
		LL_TIM_ClearFlag_UPDATE(TIM11);
	}
}

//external interrupt for ppm read
void EXTI15_10_IRQHandler()
{
	ppm_Interrupt(&ppm);
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
}
