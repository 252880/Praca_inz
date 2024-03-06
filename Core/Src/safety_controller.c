#include "safety_controller.h"

void SafetyController_Init(safetyController_t* controller, safetyControllerConfigStruct_t* config)
{
	controller->config = config;
	controller->SafetyLevel0Flag = 0;
	controller->SafetyLevel1Flag = 0;
}

void SafetyContoller_UpdateSafetyInfo(safetyController_t* controller)
{
	SafetyController_Level0Handler(controller, !LL_GPIO_IsInputPinSet(controller->config->level0_gpio.port, controller->config->level0_gpio.pin));
	SafetyController_Level1Handler(controller, !LL_GPIO_IsInputPinSet(controller->config->level1_gpio.port, controller->config->level1_gpio.pin));
	return;
}
void SafetyController_Level0Handler(safetyController_t* controller, uint8_t state)
{
	controller->SafetyLevel0Flag = state;
	return;
}
void SafetyController_Level1Handler(safetyController_t* controller, uint8_t state)
{
	controller->SafetyLevel1Flag = state;
	return;
}
uint8_t SafetyController_GetSafetyLevel0Flag(safetyController_t* controller)
{
	return controller->SafetyLevel0Flag;
}
uint8_t SafetyController_GetSafetyLevel1Flag(safetyController_t* controller)
{
	return controller->SafetyLevel1Flag;
}
int16_t SafetyController_GetLevel0minRPM(safetyController_t* controller)
{
	return controller->config->level0_minRPM;
}
int16_t SafetyController_GetLevel0maxRPM(safetyController_t* controller)
{
	return controller->config->level0_maxRPM;
}
int16_t SafetyController_GetLevel1minRPM(safetyController_t* controller)
{
	return controller->config->level1_minRPM;
}
int16_t SafetyController_GetLevel1maxRPM(safetyController_t* controller)
{
	return controller->config->level1_maxRPM;
}
void SafetyController_TurnOnMotorPower(safetyController_t* controller)
{
	LL_GPIO_SetOutputPin(controller->config->MotorPower_gpio.port, controller->config->MotorPower_gpio.pin);
}

void SafetyController_TurnOffMotorPower(safetyController_t* controller)
{
	LL_GPIO_ResetOutputPin(controller->config->MotorPower_gpio.port, controller->config->MotorPower_gpio.pin);
}

