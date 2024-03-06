
#ifndef __IBUS_H__
#define __IBUS_H__

#include "main.h"
#include <stdbool.h>
#include "driver_setup.h"


#define IBUS_UART				(&huart1)
#define IBUS_USER_CHANNELS		6


/* Defines */
#define IBUS_LENGTH				0x20	// 32 bytes
#define IBUS_COMMAND40			0x40	// Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14


/* Main Functions */
void ibus_init();
bool ibus_read(uint16_t* ibus_data);


extern uint16_t ibus_data[IBUS_USER_CHANNELS];

/* Sub Functions */
bool ibus_is_valid();
bool ibus_checksum();
void ibus_update(uint16_t* ibus_data);
void ibus_soft_failsafe(uint16_t* ibus_data, uint8_t fail_safe_max);
void ibus_reset_failsafe();




#endif /* __IBUS_H__ */
