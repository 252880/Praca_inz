#ifndef INPUT_SOURCE_SELECTOR_H
#define INPUT_SOURCE_SELECTOR_H

#include "structs.h"
#include "safety_controller.h"
#include "pid.h"
#include "ppm.h"

#define D_INPUT_SOURCE_PPM 0
#define D_INPUT_SOURCE_USB 1

typedef struct
{
	GPIO_t inputSourceGpio;
	safetyController_t* safetyController;
	ppm_t* ppm;
}inputSourceSelectorConfigStruct_t;

typedef struct
{
	inputSourceSelectorConfigStruct_t* config;
	inputFormat_t ppm_input;
	inputFormat_t usb_input;
}inputSourceSelector_t;

void inputSourceSelector_Init(inputSourceSelector_t* selector, inputSourceSelectorConfigStruct_t* config);
void inputSourceSelector_UpdatePpmInput(inputSourceSelector_t* selector);
void inputSourceSelector_UpdateUsbInput(inputSourceSelector_t* selector);
void inputSourceSelector_SetPidInput(inputSourceSelector_t* selector, pid_controller_t* pid_L, pid_controller_t* pid_R);

#endif
