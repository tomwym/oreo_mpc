#ifndef CANSOCK_H
#define CANSOCK_H

#ifdef TML_LIB_EN
#include "../include/TML_lib_light.h"
#endif

#include <stdbool.h>

#define EYE_CAN_ID
#define NECK_CAN_ID

#define IFACE_NAME          "can0"
#define SOCK_TIMEOUT_SEC    1       // sec

#ifndef TML_LIB_EN
typedef struct {
	uint32_t identifier;
	uint8_t length;
	uint8_t CAN_data[8];
} CAN_MSG;
#endif

// Init Function
// Returns file descriptor on success
// Else returns -1
int InitCanSock(uint32_t canId, char* ifName, int timeoutMs);

// Clean up socket
void CleanCanSock(int fd);

// Send CAN Message
int SendMessage(CAN_MSG * CAN_TX_message, int fd);

// Receive CAN Message
int ReceiveMessage(CAN_MSG * CAN_RX_message, int fd);

// Receive CAN Message - Payload Only
int ReceivePayload(void* payload, int fd);

// Set recv timeout
void SetRecvTimeout(int timeoutMs);

// Poll for new CAN message
//bool PollMessage();

#endif