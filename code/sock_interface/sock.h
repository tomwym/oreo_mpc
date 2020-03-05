#ifndef CANSOCK_H
#define CANSOCK_H

#include <stdint.h>
#include <netinet/in.h>


typedef struct {
	uint8_t length;
	uint8_t RS232_data[15];
}RS232_MSG;

typedef union addr{
    struct sockaddr_storage ss;
    struct sockaddr_in s4;
    struct sockaddr_in6 s6;
} addr_t;

// Init Function
// Returns file descriptor on success
// Else returns -1
int InitSock(addr_t* formatAddr, const char* address, int timeoutMs);

// Add filter to can socket
void AddFilter(int fd, uint32_t id, uint32_t mask);

// Clean up socket
void CleanSock(int fd);

// Send CAN Message
int SendMessage(RS232_MSG * msg, int fd);

// Receive CAN Message
int ReceiveMessage(RS232_MSG * msg, int fd);

// Empty the receive buffer
void FlushCanSock(int fd);

#endif