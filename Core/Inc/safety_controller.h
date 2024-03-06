#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#include "structs.h"

typedef struct
{
	GPIO_t MotorPower_gpio;
	GPIO_t level0_gpio;
	GPIO_t level1_gpio;
	int16_t level0_minRPM;
	int16_t level0_maxRPM;
	int16_t level1_minRPM;
	int16_t level1_maxRPM;
}safetyControllerConfigStruct_t;

typedef struct
{
	safetyControllerConfigStruct_t* config;
	uint8_t SafetyLevel0Flag;
	uint8_t SafetyLevel1Flag;
}safetyController_t;

void SafetyController_Init(safetyController_t* controller, safetyControllerConfigStruct_t* config);
void SafetyContoller_UpdateSafetyInfo(safetyController_t* controller);
void SafetyController_Level0Handler(safetyController_t* controller, uint8_t state);
void SafetyController_Level1Handler(safetyController_t* controller, uint8_t state);
int16_t SafetyController_GetLevel0minRPM(safetyController_t* controller);
int16_t SafetyController_GetLevel0maxRPM(safetyController_t* controller);
int16_t SafetyController_GetLevel1minRPM(safetyController_t* controller);
int16_t SafetyController_GetLevel1maxRPM(safetyController_t* controller);
uint8_t SafetyController_GetSafetyLevel0Flag(safetyController_t* controller);
uint8_t SafetyController_GetSafetyLevel1Flag(safetyController_t* controller);
void SafetyController_TurnOnMotorPower(safetyController_t* controller);
void SafetyController_TurnOffMotorPower(safetyController_t* controller);

#endif
