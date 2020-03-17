#ifndef CANSOCK_H
#define CANSOCK_H

#include <stdint.h>
#include <netinet/in.h>

#define MAX_RS232_BYTES 15 // bytes
typedef struct {
	uint8_t length;
	uint8_t RS232_data[MAX_RS232_BYTES];
} RS232_MSG;

typedef union addr{
    struct sockaddr_storage ss;
    struct sockaddr_in s4;
    struct sockaddr_in6 s6;
} addr_t;

// Init Function
// Returns file descriptor on success
// Else returns -1
int InitSock(const char* localAddr, uint16_t localPort, const char* remoteAddr, uint16_t remotePort, int timeoutMs);

// Connect socket to new remote
int ConnectSock(const char* remoteAddr, uint16_t remotePort, int fd);

// Clean up socket
void CleanSock(int* fd);

// Send RS232 message
int SendMessage(RS232_MSG* msg, int fd);

// Receive RS232 message
int ReceiveMessage(RS232_MSG* msg, int fd);

// Empty the receive buffer
void FlushCanSock(int fd);

#endif