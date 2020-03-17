/*
Purpose: To provide usage of socketCAN library provided by Linux
*/

// _DEFAULT_SOURCE defined for ifreq struct
#define _DEFAULT_SOURCE
#include <stdint.h>
#include "sock.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/ioctl.h>
#include <net/if.h>

#include <sys/socket.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

#define SEC_TO_MSEC                   (1000)
#define MSEC_TO_USEC                  (1000)
#define DEFAULT_TIMEOUT_SEC           (1)
#define DEFAULT_TIMEOUT_USEC          (0)
#define DEFAULT_PORT                  (100000)

// Create udp socket to tx/rx with the motor controller
int InitSock(const char* localAddr, uint16_t localPort, const char* remoteAddr, uint16_t remotePort, int timeoutMs)
{
    // Initialize
    int fd = -1;
    int sockErr;
    addr_t addrStruct;
    
    // Make sure input is valid
    if(strlen(localAddr) == 0 || strlen(remoteAddr) == 0) {
        printf("Invalid input addresses\n");
        return -1;
    }

    // Only supporting ipv4 addresses
    if(inet_pton(AF_INET, localAddr, &(addrStruct.s4.sin_addr)) == 1) {
        addrStruct.s4.sin_family = AF_INET;
        addrStruct.s4.sin_port = htons(localPort);
    } else {
        return -1;
    }

    // Socket creation
    int fd = socket(addrStruct.ss.ss_family, SOCK_DGRAM, 0);
    if(fd < 0) {
        sockErr = errno;
        printf("Socket creation failed with err=%d\n", sockErr);
        return -1;
    }

    // Allow address re-use local address and bind
    int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const void*)&on, sizeof(on));
    int timeout = timeoutMs;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    if(bind(fd, (const struct sockaddr*)&addrStruct, (socklen_t)sizeof(struct sockaddr_in))) {
        sockErr = errno;
        printf("Failed to bind socket to local addr with err=%d\n", sockErr);
        return -1;
    }

    // Only supporting ipv4 addresses
    memset(&addrStruct, 0, sizeof(addrStruct));
    if(inet_pton(AF_INET, remoteAddr, &(addrStruct.s4.sin_addr)) == 1) {
        addrStruct.s4.sin_family = AF_INET;
        addrStruct.s4.sin_port = htons(DEFAULT_PORT);
    } else {
        printf("Failed to format remote address %s into IP address", remoteAddr);
        return -1;
    }

    // Connect to remote
    // Connect() on a udp socket sets the default address to send messages to
    if(connect(fd, (const struct sockaddr*)&addrStruct, (socklen_t)sizeof(struct sockaddr_in))) {
        sockErr = errno;
        printf("Failed to connect to remote address with err=%d\n", sockErr);
        return -1;
    }

    return 0;
}

// Connect socket to new remote
int ConnectSock(const char* remoteAddr, uint16_t remotePort, int fd)
{
    addr_t addrStruct;
    int sockErr;

    // Make sure input is valid
    if(strlen(remoteAddr) == 0) {
        printf("Invalid input addresses\n");
        return -1;
    }

    // Only supporting ipv4 addresses
    memset(&addrStruct, 0, sizeof(addrStruct));
    if(inet_pton(AF_INET, remoteAddr, &(addrStruct.s4.sin_addr)) == 1) {
        addrStruct.s4.sin_family = AF_INET;
        addrStruct.s4.sin_port = htons(DEFAULT_PORT);
    } else {
        printf("Failed to format remote address %s into IP address", remoteAddr);
        return -1;
    }

    // Connect to remote
    // Connect() on a udp socket sets the default address to send messages to
    if(connect(fd, (const struct sockaddr*)&addrStruct, (socklen_t)sizeof(struct sockaddr_in))) {
        sockErr = errno;
        printf("Failed to connect to remote address with err=%d\n", sockErr);
        return -1;
    }

    return 0;
}

// Cleanup the socket
void CleanSock(int* fd)
{
    // Check socket
    if(fd < 0) {
        printf("No socket to close\n");
        return;
    }

    // Close the socket and free struct
    close(fd);
    *fd = -1;
}

// Send message
int SendMessage(RS232_MSG * msg, int fd)
{
    int sockErr = 0;
    
    // Check socket
    if(fd < 0) {
        printf("Invalid fd on SendMessage\n");
        return -1;
    }

    // Wait on socket to be able to write
    /*struct timeval timeout = {SOCK_TIMEOUT_SEC,0};
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if(select(FD_SETSIZE, NULL, &set, NULL, &(timeout)) < 1) {
        perror("select");
        return -1;
    }*/

    // Send the frame
    int sendBytes = send(fd, msg->RS232_data, msg->length, 0);
    if (sendBytes < 0) {
        sockErr = errno;
        printf("Error on send (err=%d)\n", sockErr);
        return -1;
    } else if(sendBytes != msg->length) {
        printf("Unable to send full message\n");
        return -1;
    }

    return 0;
}

// Receive CAN Message
int ReceiveMessage(RS232_MSG * msg, int fd)
{
    int sockErr = 0;
    // Check socket
    if(fd < 0) {
        printf("Invalid fd on ReceiveMessage\n");
        return -1;
    }

    // Wait on socket for available data
    /*struct timeval timeout = setTimeout;
    fd_set set;
    FD_ZERO(&set);
    FD_SET(fd, &set);
    if(select(FD_SETSIZE, &set, NULL, NULL, &(timeout)) < 1) {
        perror("select");
        return -1;
    }*/

    // Read message
    int recvBytes = recv(fd, msg->RS232_data, MAX_RS232_BYTES, 0);
    if(recvBytes < 0) {
        sockErr = errno;
        printf("Error on recv (err=%d)\n", sockErr);
        return -1;
    }
    msg->length = recvBytes;

    return 0;
}

// Empty the receive buffer
void FlushSock(int fd)
{
    fd_set set;
    uint8_t recvBytes;
    RS232_MSG msg;

    FD_ZERO(&set);
    FD_SET(fd, &set);
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 0};
    while(select(FD_SETSIZE, NULL, &set, NULL, &(timeout)) > 0) {
        recvBytes = recv(fd, msg.RS232_data, msg.length, 0);
        if (recvBytes < 0) {
            perror("can raw socket read");
            return;
        }

        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
    }
    
    return;
}