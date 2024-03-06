#ifndef INC_MESSAGE_FRAME_H_
#define INC_MESSAGE_FRAME_H_

#pragma pack(push,1)

#define D_MESSAGE_FRAME_START_BYTE 0x39
#define D_MESSAGE_HEADER_LENGTH 3

typedef struct
{
	uint8_t start_byte;
	uint8_t message_id;
	uint8_t message_length;
}messageHeader_t;

typedef struct
{
	messageHeader_t header;
	uint8_t* payload;
}messageFrame_t;

#pragma pack(pop)

#endif
