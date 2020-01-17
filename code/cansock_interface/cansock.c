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

#define MAX_CAN_ID          (0x800)
#define SEC_TO_MSEC         (1000)
#define MSEC_TO_USEC        (1000)
#define DEFAULT_TIMEOUT_SEC     1
#define DEFAULT_TIMEOUT_USEC    0

static struct timeval setTimeout;

// Create CAN socket to tx/rx with the motor controller
int InitCanSock(uint32_t canId, char* ifName, int timeoutMs)
{      
    // Init the struct
    int fd = -1;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct can_filter filter;

    // Open the CAN socket
	if ((fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("socket");
        printf("Could not open socket\n");
		return -1;
	}

    // Set socket to recv from specified can ID only
    if(canId >= MAX_CAN_ID) {
        printf("Invalid CAN Id for socket init\n");
        return -1;
    }
    filter.can_id = canId;
    filter.can_mask = CAN_SFF_MASK;
    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));

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

    return fd;
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
    CAN_RX_message->length = frame.can_dlc;
    memcpy(&(CAN_RX_message->CAN_data), &(frame.data), CAN_MAX_DLEN);

    return 0;
}

// Receive CAN Message
int ReceivePayload(void* payload, int fd)
{
    CAN_MSG msg;
    int ret = 0;
    memset(&msg, 0, sizeof(msg));
    ret = ReceiveMessage(&msg, fd);
    if(ret < 0)
        return ret;
    
    memcpy(payload, msg.CAN_data, msg.length);
    return 0;
}

// Set recv timeout
/*void SetRecvTimeout(int timeoutMs)
{
    setTimeout.tv_sec = timeoutMs/SEC_TO_MSEC;
    setTimeout.tv_usec = (timeoutMs%SEC_TO_MSEC)*MSEC_TO_USEC;
}*/