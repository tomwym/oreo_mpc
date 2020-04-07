#include "libmacaque_RS232/macaque_linux.h"
#include "TML_RS232_lib/include/TML_RS232_lib.h"
#include "sock_interface/sock.h"

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
//#define WIN_TEST
#ifdef WIN_TEST
#include <WinSock2.h>
#include <WS2tcpip.h>
#pragma comment(lib,"ws2_32.lib") // Winsock library
#else
#include <unistd.h>
#endif

#define BUFF_SIZE       (255)
#define LOOPBACK_IP     "127.0.0.1"

static char* get_msg_str(RS232_MSG* frame, char* buff)
{
    snprintf(buff, BUFF_SIZE, "0x");
    char* ptr = buff+2;
    for(int i = 0; i < frame->length; i++) {
        snprintf(ptr, BUFF_SIZE, " %02x", frame->RS232_data[i]);
        ptr+=sizeof(frame->RS232_data[i]) + 2;
    }

    return buff;
}

static bool msg_comp(RS232_MSG* frame, RS232_MSG* exp_frame) 
{
    // Simple check
    if(frame->length != exp_frame->length) {
        printf("Size: Frame=%d Expected=%d\n", frame->length, exp_frame->length);
        return false;
    }
    
    // Compare byte-by-byte 
    if(memcmp(frame, exp_frame, frame->length) == 0) {
        return true;
    }

    char frame_str[BUFF_SIZE];
    char exp_frame_str[BUFF_SIZE];
    printf("Mismatch: Frame=%s Expected=%s\n", get_msg_str(frame, frame_str), get_msg_str(exp_frame, exp_frame_str));
    
    return false;
}

typedef struct {
    char* local_ip;
    uint16_t local_port;
    char* dest_ip;
    uint16_t dest_port;
    int fd;
    char* cmd_file;
} test_handle_t;

typedef enum {
    ARG_APP_NAME,
    ARG_DEV,
    ARG_FILE,
} cmd_args_t;

int main(int argc, char* argv[])
{
    test_handle_t dev;
    if(argc <= 1) {
        printf("Invalid input args for startup app\n");
        return -1;
    }

    if(strcmp(argv[1], "neck") == 0 || (int)(argv[1] - '0') == 1) {
#ifdef WIN_TEST
        dev.dest_ip = LOCAL_IP;
        dev.dest_port = NECK_LOCAL_PORT;
        dev.local_ip = NECK_IP;
        dev.local_port = COMM_PORT;
#else
        dev.dest_ip = LOOPBACK_IP;
        dev.dest_port = NECK_LOCAL_PORT;
        dev.local_ip = LOOPBACK_IP;
        dev.local_port = COMM_PORT;
#endif
    } else if (strcmp(argv[1], "eye") == 0 || (int)(argv[1] - '0') == 2) {
#ifdef WIN_TEST
        dev.dest_ip = LOCAL_IP;
        dev.dest_port = EYE_LOCAL_PORT;
        dev.local_ip = EYE_IP;
        dev.local_port = COMM_PORT;
#else
        dev.dest_ip = LOOPBACK_IP;
        dev.dest_port = EYE_LOCAL_PORT;
        dev.local_ip = LOOPBACK_IP;
        dev.local_port = COMM_PORT;
#endif
    } else {
        printf("Invalid input args for startup app\n");
        return -1;
    }
    dev.cmd_file = strdup(argv[2]);

#ifdef WIN_TEST
    WSADATA wsa;
    printf("\n Initialising Winsock...");
    if(WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        printf("Failed. Error Code: %d\n", WSAGetLastError());
        return -1;
    }
    printf("Initialized\n");

    dev.fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(dev.fd == INVALID_SOCKET) {
        printf("Could not create socket: %d", WSAGetLastError());
        return -1;
    }
    printf("Socket created\n");

    // reuse address
    int optval = 1;
    setsockopt(dev.fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

    struct sockaddr_in localAddr, remoteAddr;
    if(inet_pton(AF_INET, dev.local_ip, &localAddr.sin_addr.s_addr) != 1) {
        printf("Could not format local ip address err=%d\n", WSAGetLastError());
        return -1;
    }
    localAddr.sin_addr = AF_INET;
    localAddr.sin_port = htons(dev.local_port);

    if(bind(dev.fd, (struct sockaddr*)localAddr, sizeof(localAddr))) {
        printf("Could not bind to local addr err=%d\n", WSAGetLastError());
        return -1;
    }
    
    if(inet_pton(AF_INET, dev.remote_ip, &remoteAddr.sin_addr.s_addr) != 1) {
        printf("Could not format remote ip address err=%d\n", WSAGetLastError());
        return -1;
    }
    remoteAddr.sin_family = AF_INET;
    remoteAddr.sin_port = htons(dev.remote_port);
    if(connect(dev.fd, (struct sockaddr*)remoteAddr, sizeof(remoteAddr) != 0)) {
        printf("Could not connect to RPI err=%d\n", WSAGetLastError());
        return -1;
    }
#else
    dev.fd = InitSock(dev.local_ip, dev.local_port, dev.dest_ip, dev.dest_port, 1000);
    if(dev.fd < 0) {
        printf("Could not open socket\n");
        return -1;
    }
#endif

    // Startup
    const uint8_t* recv_conn[NUM_CONN_MSG] = {CONN_MSG1, CONN_MSG2, CONN_MSG3, CONN_MSG4};
    const uint8_t recv_size[NUM_CONN_MSG] = {sizeof(CONN_MSG1), sizeof(CONN_MSG2), sizeof(CONN_MSG3), sizeof(CONN_MSG4)};
    const uint8_t conn_resp[NUM_CONN_MSG] = {CONN_RESP1, CONN_RESP2, CONN_RESP3, CONN_RESP4};
#ifdef WIN_TEST
    int recv_size;
    for(int i = 0; i < NUM_CONN_MSG; i++) {
        RS232_MSG recv_msg;
        memset(&recv_msg.RS232_data, 0, MAX_RS232_BYTES);
        recv_size = recv(dev.fd, recv_msg.RS232_data, MAX_RS232_BYTES, 0);
        if(recv_size <= 0) {
            printf("Failed to recv %d conn msg\n", i);
            return -1;
        }

        RS232_MSG exp_msg;
        exp_msg.length = recv_size[i];
        memset(exp_msg.RS232_data, 0, MAX_RS232_BYTES);
        memcpy(exp_msg.RS232_data, recv_conn[i], exp_msg.length);
        if(!msg_comp(&exp_msg, &recv_msg)) {
            printf("Recvd unexpected conn msg %d\n", i);
            return -1;
        }

        RS232_MSG send_msg;
        memset(send_msg.RS232_data, 0, MAX_RS232_BYTES);
        send_msg.length = sizeof(conn_resp[i]);
        memcpy(send_msg.RS232_data, &(conn_resp[i]), send_msg.length);
        if(send(dev.fd, send_msg.RS232_data, send_msg.length, 0)) {
            printf("Failed to send conn msg %d\n", i);
            return -1;
        }
    }
#else
    for(int i = 0 ; i < NUM_CONN_MSG; i++) {
        RS232_MSG recv_msg;
        memset(recv_msg.RS232_data, 0, MAX_RS232_BYTES);
        if(ReceiveMessage(&recv_msg, dev.fd) < 0) {
            printf("Failed to receive conn msg %d\n", i);
            return -1;
        }

        RS232_MSG exp_msg;
        exp_msg.length = recv_size[i];
        memset(exp_msg.RS232_data, 0, MAX_RS232_BYTES);
        memcpy(exp_msg.RS232_data, recv_conn[i], exp_msg.length);
        if(!msg_comp(&exp_msg, &recv_msg)) {
            printf("Recvd unexpected conn msg %d\n", i);
            return -1;
        }

        RS232_MSG send_msg;
        memset(send_msg.RS232_data, 0, MAX_RS232_BYTES);
        send_msg.length = sizeof(conn_resp[i]);
        memcpy(send_msg.RS232_data, &(conn_resp[i]), send_msg.length);
        if(SendMessage(&send_msg, dev.fd)) {
            printf("Failed to send conn msg %d\n", i);
            return -1;
        }
    }
    printf("Test finished startup sequence\n");
#endif
}