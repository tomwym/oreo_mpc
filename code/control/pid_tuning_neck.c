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
#include <math.h>

#include "control/pid.h"

#ifndef M_PI
    #define M_PI                        ((double)3.14159265358979323846)
#endif
void PrintNeckData();

void neck_ctrl()
{
    pid_loop_t loop = {
        .param = {.kp = 0.00311, .ki = 0, .kd = 0, 
        .cmd_lim = {.low = -0.015, .high = 0.015}, 
        .pos_lim = {.low = -1*M_PI/2, .high = M_PI/2}},
        .errSum = 0, .prevErr = 0, .prevTime = 0
    };

    uint8_t axis = NECK_YAW_AXIS;
    
    neckData_t* neckData = GetNeckData();
    pos_t* feedback = &(neckData->pos[axis-1]);
    InitNeckTorqueCtrl(NECK_YAW_AXIS);
    double time = 0, pos = 0, torque = 0;

    // Make sure recving feedback from motors
    do {
        pthread_mutex_lock(&feedback->mutex);
        time = feedback->time;
        pthread_mutex_unlock(&feedback->mutex);
    } while(time == 0);

    double target = 0.175; // 10 deg
    StartMotion(&loop, time);
    printf("\n****PID Parameters****\n");
    printf("Kp = %f\n", loop.param.kp);
    printf("Ki = %f\n", loop.param.ki);
    printf("Kd = %f\n", loop.param.kd);
    printf("Torque Limit High=%f\n", loop.param.cmd_lim.high);
    printf("Torque Limit Low=%f\n", loop.param.cmd_lim.low);
    printf("Target = %f\n", target);
    printf("****PID Parameters****\n");

    while(1) {
        // Poll waiting for new data
        do {
            pthread_mutex_lock(&feedback->mutex);
            time = feedback->time;
            pos = feedback->pos;
            pthread_mutex_unlock(&feedback->mutex);
        } while(time-loop.prevTime < 0.001);

        torque = UpdateLoop(&loop, target, pos, time);
        SetNeckTorque(axis, torque);

        if(time > 10) {
            break;
        }
    }
    PrintNeckData();
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
    printf("ready:");
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf(" %d", neckData->ready[i]);
    }
    printf("\n");
    printf("****Neck Data****\n");
}

int main()
{
    motor_id_t dest = {.type = ID_TYPE_BROADCAST, .id = 0};
    ResetFaults(DEV_NECK, &dest);
    neck_ctrl();
    SendGiveMeData2(DEV_NECK, &dest, REG_CER, true);
    SendGiveMeData2(DEV_NECK, &dest, REG_CSR, true);
    SendGiveMeData2(DEV_NECK, &dest, REG_CBR, true);
    usleep(2*1000*1000);
}