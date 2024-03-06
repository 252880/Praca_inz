#ifndef DRIVER_SETUP_H
#define DRIVER_SETUP_H

#include <input_source_selector.h>
#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "structs.h"
#include "current_measurement.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "ppm.h"
#include "safety_controller.h"
#include "hoverserial.h"
#include "ibus.h"
#include "sbus.h"



extern uint8_t byte;	//wymagane dla hoverserial (przerwanie - inicjalizacja)
extern motor_t motor_L, motor_R;
extern pid_controller_t pid_L, pid_R;
extern encoder_t enc_L, enc_R;
extern currentMeasurement_t curr_mot_L, curr_mot_R;
extern ppm_t ppm;
extern safetyController_t safety;
extern inputSourceSelector_t input_source;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t bldc;

void motorsSetup(uint8_t actuation_type, uint8_t motor_type);
typedef pidCoefficients_t pidC;
void pidSetup(pidC inL, pidC outL, pidC inR, pidC outR, uint32_t bound);
void encoderSetup(uint32_t enc_spr_L, uint32_t enc_spr_R);
void currentMeasurementSetup(double gainAV);
void ppmSetup(uint8_t ppm_no_of_channels, uint16_t ppm_max_t_us, uint16_t ppm_min_t_us,  uint16_t max_vel, uint8_t rc_type);
void usbSetup();
void safetyControllerSetup(int16_t l0_min, int16_t l0_max, int16_t l1_min, int16_t l1_max);
void inputSourceSelectorSetup();
void loopsSetup();
void MX_DMA_Init();
void MX_USART1_UART_Init(uint8_t rc_type);
void driverSetup(driverConfig_t* config);

#endif
