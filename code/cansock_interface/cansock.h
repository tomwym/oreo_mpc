#ifndef CANSOCK_H
#define CANSOCK_H

#ifdef TML_LIB_EN
#include "../include/TML_lib_light.h"
#endif

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
int InitCanSock(uint8_t hostId, char* ifName, int timeoutMs);

// Add filter to can socket
void AddFilter(int fd, uint8_t id, uint8_t isGroup);

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