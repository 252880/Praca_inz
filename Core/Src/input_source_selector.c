#include "input_source_selector.h"

void inputSourceSelector_Init(inputSourceSelector_t* controller, inputSourceSelectorConfigStruct_t* config)
{
	controller->config = config;
	return;
}

void inputSourceSelector_SetPidInput(inputSourceSelector_t* selector, pid_controller_t* pid_L, pid_controller_t* pid_R)
{
	int32_t lin_vel, rot_vel;

	if(LL_GPIO_IsInputPinSet(selector->config->inputSourceGpio.port, selector->config->inputSourceGpio.pin) == D_INPUT_SOURCE_PPM)
	{
		inputSourceSelector_UpdatePpmInput(selector);

		lin_vel = selector->ppm_input.lin_vel * (int16_t)selector->ppm_input.max_vel_proc / 100;
		rot_vel = selector->ppm_input.rot_vel * (int16_t)selector->ppm_input.max_vel_proc / 100;


	//in_vel = 200 * (int16_t)selector->ppm_input.max_vel_proc / 100;
	 // rot_vel = 200 * (int16_t)selector->ppm_input.max_vel_proc / 100;

		if(ppm_GetControlType(selector->config->ppm) ==  D_PPM_CONTROL_TYPE_AUTO)
		{
			inputSourceSelector_UpdateUsbInput(selector);
			lin_vel = selector->usb_input.lin_vel;
			rot_vel = selector->usb_input.rot_vel;
		}

		if(ppm_GetMotorPowerState(selector->config->ppm))
			SafetyController_TurnOnMotorPower(selector->config->safetyController);
		else
		{
			SafetyController_TurnOffMotorPower(selector->config->safetyController);
			lin_vel = 0;
			rot_vel = 0;
		}
	}
	else
	{
		inputSourceSelector_UpdateUsbInput(selector);
		lin_vel = selector->usb_input.lin_vel;
		rot_vel = selector->usb_input.rot_vel;
	}

	if(SafetyController_GetSafetyLevel1Flag(selector->config->safetyController))
	{
		int32_t low_bound = SafetyController_GetLevel1minRPM(selector->config->safetyController);
		int32_t up_bound = SafetyController_GetLevel1maxRPM(selector->config->safetyController);

		if(lin_vel > up_bound)
			lin_vel = up_bound;
		if(lin_vel < low_bound)
			lin_vel = low_bound;

		if(rot_vel > up_bound)
			rot_vel = up_bound;
		if(rot_vel < low_bound)
			rot_vel = low_bound;
	}
	if(SafetyController_GetSafetyLevel0Flag(selector->config->safetyController))
	{
		int32_t low_bound = SafetyController_GetLevel0minRPM(selector->config->safetyController);
		int32_t up_bound = SafetyController_GetLevel0maxRPM(selector->config->safetyController);

		if(lin_vel > up_bound)
			lin_vel = up_bound;
		if(lin_vel < low_bound)
			lin_vel = low_bound;

		if(rot_vel > up_bound)
			rot_vel = up_bound;
		if(rot_vel < low_bound)
			rot_vel = low_bound;
	}




	pid_ChangeSetpoint(pid_L, lin_vel - rot_vel / 2);
	pid_ChangeSetpoint(pid_R, lin_vel + rot_vel / 2);

}
void inputSourceSelector_UpdatePpmInput(inputSourceSelector_t* selector)
{
	selector->ppm_input = ppm_GetControlSignal(selector->config->ppm);
}

extern inputFormat_t usb_input;
void inputSourceSelector_UpdateUsbInput(inputSourceSelector_t* selector)
{
	selector->usb_input = usb_input;
}
