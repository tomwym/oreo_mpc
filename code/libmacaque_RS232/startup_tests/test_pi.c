#include "libmacaque_RS232/macaque_linux.h"
#include "TML_RS232_lib/include/TML_RS232_lib.h"
#include "sock_interface/sock.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BUFF_SIZE       (255)

void PrintEyeCal()
{
    eyeCalData_t* eyeCal = GetEyeCalData();
    printf("\n****Eye Cal Data****\n");
    printf("time: %.3f\n", eyeCal->time);
    printf("pos:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeCal->pos[i]);
    }
    printf("\n");
    printf("pos_err:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeCal->err[i]);
    }
    printf("\n");
    printf("tpos:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeCal->tpos[i]);
    }
    printf("\n");
    printf("complete:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3d", eyeCal->complete[i]);
    }
    printf("\n");
        printf("offset:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeCal->offset[i]);
    }
    printf("\n");
    printf("****Eye Cal Data****\n");
}

void PrintEyeData()
{
    eyeData_t* eyeData = GetEyeData();
    printf("\n****Eye Data****\n");
    printf("time: %.3f\n", eyeData->time);
    printf("pos_time:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeData->pos[i].time);
    }
    printf("\n");
    printf("pos:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeData->pos[i].pos);
    }
    printf("\n");
    printf("torque: ");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeData->torque[i]);
    }
    printf("\n");
    printf("****Eye Data****\n");
}

void PrintNeckData()
{
    neckData_t* neckData = GetNeckData();
    printf("\n****Neck Data****\n");
    printf("time:   %.3f\n", neckData->time);
    printf("pos_time: ");
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf(" %.3f", neckData->pos[i].time);
    }
    printf("\n");
    printf("pos: ");
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf(" %.3f", neckData->pos[i].pos);
    }
    printf("\n");
    printf("torque:");
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf(" %.3f", neckData->torque[i]);
    }
    printf("\n");
    printf("****Neck Data****\n");
}

int main()
{
    int cnt = 0;
    while(1) {
        if(cnt < 32) {
            DisableEyeCtrl();
            DisableNeckCtrl();
            cnt++;
        } else {
            usleep(20000000);
            PrintEyeCal();
            PrintEyeData();
            PrintNeckData();
            break;
        }
    };  
}