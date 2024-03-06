#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "main.h"
#include "driver_setup.h"
#include "ibus.h"

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03

extern uint16_t failsafe_status;
extern uint8_t buf[25];
extern uint16_t CH[18];



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#endif /* INC_SBUS_H_ */
