/*
Purpose: To provide usage of socketCAN library provided by Linux
*/

// _DEFAULT_SOURCE defined for ifreq struct
#define _DEFAULT_SOURCE
#include <stdint.h>
#include "cansock.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>

#define MAX_CAN_ID                    (0x800)
#define SEC_TO_MSEC                   (1000)
#define MSEC_TO_USEC                  (1000)
#define DEFAULT_TIMEOUT_SEC           (1)
#define DEFAULT_TIMEOUT_USEC          (0)
#define FILTER_ID(id, grp)            ((uint32_t)id << 13)|(grp << 21)
#define FILTER_MASK                   (0x003FE000)


// Create CAN socket to tx/rx with the motor controller
int InitCanSock(char* ifName, int timeoutMs)
{      
    // Init the struct
    int fd = -1;
    struct ifreq ifr;
    struct sockaddr_can addr;

    // Open the CAN socket
	if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
        printf("Could not open socket\n");
		return -1;
	}

    // Find interface index from the name
    addr.can_family = AF_CAN;
    strcpy(ifr.ifr_name, ifName);
    if(ioctl(fd, SIOCGIFINDEX, &ifr)) {
        perror("SIOCGIFINDEX");
        printf("Could not find interface %s\n", ifr.ifr_name);
        close(fd);
        return -1;
    }
    addr.can_ifindex = ifr.ifr_ifindex;

    // Bind to specified interface
    if(bind(fd, (struct sockaddr *)(&addr), sizeof(addr))) {
        perror("bind");
        printf("Could not bind to interface %s\n", ifr.ifr_name);
        close(fd);
        return -1;
    }

    // Socket configuration
    struct timeval timeout = {.tv_sec = timeoutMs/SEC_TO_MSEC,. tv_usec = (timeoutMs%SEC_TO_MSEC)*MSEC_TO_USEC};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Perhaps want to maximize the recv/send buffer
    // using setsockopt with SO_RCVBUF SO_SNDBUF

    // Turn on and turn off loopback

    return fd;
}

// Apply filter to can socket
void AddFilter(int fd, uint32_t id, uint32_t mask)
{
    struct can_filter filter;
    
    filter.can_id = id;
    filter.can_mask = mask;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
}

// Clean up socket
void CleanCanSock(int fd)
{
    // Check socket
    if(fd < 0) {
        printf("No socket to close\n");
        return;
    }

    // Close the socket and free struct
    close(fd);
}

// Send CAN Message
int SendMessage(CAN_MSG * CAN_TX_message, int fd)
{
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

    // Format CAN message
    struct can_frame frame;
    frame.can_id = CAN_TX_message->identifier;
    if(CAN_TX_message->type == EXTENDED_FRAME)
        frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = CAN_TX_message->length;
    memset(frame.data, 0, CAN_MAX_DLEN);
    memcpy(frame.data, CAN_TX_message->CAN_data, frame.can_dlc);

    // Send the frame
    int sendBytes = 0;
    sendBytes = write(fd, &frame, sizeof(frame));
    if(sendBytes != sizeof(frame)) {
        perror("write");
        printf("Send CAN message error\n");
        return -1;
    }

    return 0;
}

// Receive CAN Message
int ReceiveMessage(CAN_MSG * CAN_RX_message, int fd)
{
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
    
    // Read CAN bus message
    struct can_frame frame;
    int recvBytes = 0;
    recvBytes = read(fd, &frame, sizeof(struct can_frame));
    if (recvBytes < 0) {
        perror("can raw socket read");
        return -1;
    }

    // Ensure received full CAN frame
    if (recvBytes < sizeof(struct can_frame)) {
        fprintf(stderr, "read: incomplete CAN frame\n");
        return -1;
    }

    // Copy relevant info to struct
    CAN_RX_message->identifier = frame.can_id;
    CAN_RX_message->identifier &= ~(CAN_EFF_FLAG & CAN_RTR_FLAG & CAN_ERR_FLAG);
    CAN_RX_message->length = frame.can_dlc;
    memcpy(&(CAN_RX_message->CAN_data), &(frame.data), CAN_RX_message->length);

    return 0;
}

// Set recv timeout
/*void SetRecvTimeout(int timeoutMs)
{
    setTimeout.tv_sec = timeoutMs/SEC_TO_MSEC;
    setTimeout.tv_usec = (timeoutMs%SEC_TO_MSEC)*MSEC_TO_USEC;
}*/

// Empty the receive buffer
void FlushCanSock(int fd)
{
    fd_set set;
    struct can_frame frame;
    uint8_t recvBytes;

    FD_ZERO(&set);
    FD_SET(fd, &set);
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 0};
    while(select(FD_SETSIZE, NULL, &set, NULL, &(timeout)) > 0) {
        recvBytes = read(fd, &frame, sizeof(struct can_frame));
        if (recvBytes < 0) {
            perror("can raw socket read");
            return;
        }

        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
    }
}