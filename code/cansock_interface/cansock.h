#ifndef CANSOCK_H
#define CANSOCK_H

#include <stdint.h>

typedef struct {
	uint32_t identifier;
	uint8_t length;
	uint8_t CAN_data[8];
	uint8_t type;
} CAN_MSG;

typedef enum {
	STANDARD_FRAME,
	EXTENDED_FRAME,
} CAN_TYPE;

// Init Function
// Returns file descriptor on success
// Else returns -1
int InitCanSock(char* ifName, int timeoutMs);

// Add filter to can socket
void AddFilter(int fd, uint32_t id, uint32_t mask);

// Clean up socket
void CleanCanSock(int fd);

// Send CAN Message
int SendMessage(CAN_MSG * CAN_TX_message, int fd);

// Receive CAN Message
int ReceiveMessage(CAN_MSG * CAN_RX_message, int fd);

// Set recv timeout
void SetRecvTimeout(int timeoutMs);

// Poll for new CAN message
//bool PollMessage();

// Empty the receive buffer
void FlushCanSock(int fd);

#endif