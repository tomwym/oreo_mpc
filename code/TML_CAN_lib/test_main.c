/*
Purpose: To test the new macaque library housed on the raspberry Pi

*/

#include "macaque_linux.h"
#include "cansock_interface/cansock.h"
#include "TML_CAN_lib/include/TML_CAN_lib.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define EYE_IDX     (0)
#define NECK_IDX    (1)

#define END_OPTION  (100)

typedef enum {
    ARG_BITRATE = 1,
    ARG_HOSTID,
    ARG_CONTROL_TYPE,
    NUM_ARGS,
} ARGS_t;

static void PrintCmdMenu()
{
    printf("\n\n---CMD MENU---\n");
    printf("0 - GOTO\n");
    printf("1 - GETDATA_16\n");
    printf("2 - GETDATA_32\n");
    printf("3 - GETDATA2_16\n");
    printf("4 - GETDATA2_32\n");
    printf("5 - GETDATA2_32\n");
    printf("6 - SETDATA_16\n");
    printf("7 - SETDATA_32\n");
    printf("8 - STA\n");
    printf("9 - AXISON\n");
    printf("10 - AXISOFF\n");
    printf("11 - UPDATEMODE_0\n");
    printf("12 - UPDATEMODE_1\n");
    printf("13 - MOTIONMODE_PP\n");
    printf("14 - MOTIONMODE_PP1\n");
    printf("15 - MOTIONMODE_PP3\n");
    printf("16 - SET_POSABS\n");
    printf("17 - SET_POSREL\n");
    printf("18 - SET_MASTERID\n");
    printf("100 - EXIT\n");
    printf("\n\n---CMD MENU END---\n");
}

static void Goto()
{
    uint8_t dev, addr;
    printf("Select device (0-eye, 1-neck): ");
    scanf("%u", &(dev));
    printf("Select location (0-motion, 1-wait): ");
    scanf("%u", &(addr));
    if(dev == EYE_IDX) {
        if(addr == 0) {
            SetEyePollData();
        } else if(addr == 1) {
            ResetEyePollData();
        }
    } else if(dev == NECK_IDX) {
        if(addr == 0) {
            SetNeckPollData();
        } else if(addr == 1) {
            ResetNeckPollData();
        }
    }
}

static void GetData(msgType_t cmd)
{
    uint8_t dest, dev;
    uint16_t addr;
    printf("Enter destination axis (hex): \n");
    scanf("%x", dest);
    printf("Enter register address (hex): ");
    scanf("%x", addr);
    printf("Enter device (0-eye, 1-neck): ");
    scanf("%u", dev);

    if(dev == EYE_IDX) {
        GetDataMemEye(cmd, dest, addr);
    } else if(dev == NECK_IDX) {
        GetDataMemNeck(cmd, dest, addr);
    }

    // Wait for response
    sleep(1);
}

static void SetData(msgType_t cmd)
{
    uint8_t dest, dev;
    uint16_t addr;
    uint32_t data;
    printf("Enter destination axis (hex): ");
    scanf("%x", dest);
    printf("Enter register address (hex): ");
    scanf("%x", addr);
    printf("Enter data to write (hex): ");
    scanf("%x", data);
    printf("Enter device (0-eye, 1-neck): ");
    scanf("%u", dev);

    if(dev == EYE_IDX) {
        SetDataMemEye(cmd, dest, addr, data);
    } else if(dev == NECK_IDX) {
        SetDataMemNeck(cmd, dest, addr, data);
    }
}

static void MotorCommand(msgType_t cmd)
{
    uint8_t dev, dest;
    printf("Enter device (0-eye, 1-neck): ");
    scanf("%u", dev);
    printf("Enter destination axis (hex): ");
    scanf("%x", dest);
    if(dev == EYE_IDX) {
        SendMotorCommandEye(cmd, dest, 0);
    } else {
        SendMotorCommandNeck(cmd, dest, 0);
    }

}

int main(int argc, char *argv[])
{
    // Get the arguments
    if(argc > NUM_ARGS || argc < 1) {
        printf ("Invalid number of arguments\n");
        return -1;
    }
    
    // Open a socket
    int fd = InitCanSock("can0", 1);


    while(1) {
        // Get user input
        uint8_t cmd;
        printf("Please enter command\n");
        if(scanf("%d", cmd) != 1) {
            PrintCmdMenu();
            continue;
        }
        switch(cmd) {
            case CMD_GOTO:
                printf("CMD_GOTO\n");
                Goto();
                break;
            case CMD_GETDATA_16:
                printf("CMD_GETDATA_16\n");
                GetData(cmd);
                break;
            case CMD_GETDATA_32:
                printf("CMD_GETDATA_32\n");
                GetData(cmd);
                break;
            case CMD_GETDATA2_16:
                printf("CMD_GETDATA2_16\n");
                GetData(cmd);
                break;
            case CMD_GETDATA2_32:
                printf("CMD_GETDATA2_32\n");
                GetData(cmd);
                break;
            case CMD_SETDATA_16:
                printf("CMD_SETDATA_16\n");
                SetData(cmd);
                break;
            case CMD_SETDATA_32:
                printf("CMD_SETDATA_32\n");
                SetData(cmd);
                break;
            case CMD_STA:
                printf("CMD_STA\n");
                MotorCommand(cmd);
                break;
            case CMD_AXISON:
                printf("CMD_AXISON\n");
                MotorCommand(cmd);
                break;
            case CMD_AXISOFF:
                printf("CMD_AXISOFF\n");
                MotorCommand(cmd);
                break;
            case CMD_UPDATEMODE_0:
                printf("CMD_UPDATEMODE_0\n");
                MotorCommand(cmd);
                break;
            case CMD_UPDATEMODE_1:
                printf("CMD_UPDATEMODE_1\n");
                MotorCommand(cmd);
                break;
            case CMD_MOTIONMODE_PP:
                printf("CMD_MOTIONMODE_PP\n");
                MotorCommand(cmd);
                break;
            case CMD_MOTIONMODE_PP1:
                print("CMD_MOTIONMODE_PP1\n");
                MotorCommand(cmd);
                break;
            case CMD_MOTIONMODE_PP3:
                printf("CMD_MOTIONMODE_PP3\n");
                MotorCommand(cmd);
                break;
            case CMD_SET_POSABS:
                printf("CMD_SET_POSABS\n");
                MotorCommand(cmd);
                break;
            case CMD_SET_POSREL:
                printf("CMD_SET_POSREL\n");
                MotorCommand(cmd);
                break;
            case CMD_SET_MASTERID:
                printf("CMD_SET_MASTERID\n");
                MotorCommand(cmd);
                break;
            case END_OPTION:
                printf("Exiting\n");
                goto done;
            default:
                printf("Invalid motor command\n");
                PrintCmdMenu();
                break;
        }
    }
    
    done:
    // Cleanup lib
    CleanCanSock(fd);
}