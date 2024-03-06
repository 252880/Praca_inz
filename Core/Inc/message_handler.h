#ifndef INC_MESSAGE_HANDLER_H_
#define INC_MESSAGE_HANDLER_H_
#include "stm32f4xx_ll_system.h"
#include "message_frame.h"
#include "messages.h"

typedef struct{
	uint8_t message_id;
	uint8_t* payload;
	uint32_t payload_length;
}messageHandler_t;

void messageHandler_RegisterMessage(messageHandler_t* handler, uint8_t id, uint8_t* payload, uint32_t payload_length);
void messageHandler_ApplyMessage(messageHandler_t* handler);

#endif
