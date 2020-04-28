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
    printf("offset:");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeData->offset[i]);
    }
    printf("\n");
    printf("force: ");
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf(" %.3f", eyeData->force[i]);
    }
    printf("\n");
    printf("****Eye Data****\n");
}

void eye_ctrl()
{
    pid_loop_t pitch_loop = {
        .param = {.kp = 0.0001, .ki = 0.00001, .kd = 0, 
        .cmd_lim = {.low = -0.015, .high = 0.015}, 
        .pos_lim = {.low = 0, .high = 10}},
        .errSum = 0, .prevErr = 0, .prevTime = 0
    };

    pid_loop_t yaw_loop = {
        .param = {.kp = 0.0001, .ki = 0.00001, .kd = 0, 
        .cmd_lim = {.low = -0.015, .high = 0.015}, 
        .pos_lim = {.low = 0, .high = 10}},
        .errSum = 0, .prevErr = 0, .prevTime = 0
    };

    uint8_t left_axis = LEFT_EYE_LEFT_AXIS;
    uint8_t right_axis = LEFT_EYE_RIGHT_AXIS;

    eyeData_t* eyeData = GetEyeData();
    pos_t* pitch_feedback = &(eyeData->pos[LEFT_EYE_PITCH_IDX]);
    pos_t* yaw_feedback = &(eyeData->pos[LEFT_EYE_YAW_IDX]);
    InitEyeForceCtrl(left_axis);
    InitEyeForceCtrl(right_axis);
    double yaw_time = 0, yaw_pos = 0, yaw_target = 0;
    double pitch_time = 0, pitch_pos = 0, pitch_target = 0;
    double force_avg = 0, force_diff = 0, time = 0;

    // Make sure recving feedback from motors
    do {
        pthread_mutex_lock(&pitch_feedback->mutex);
        pitch_time = pitch_feedback->time;
        pthread_mutex_unlock(&pitch_feedback->mutex);

        pthread_mutex_lock(&yaw_feedback->mutex);
        yaw_time = yaw_feedback->time;
        pthread_mutex_unlock(&yaw_feedback->mutex);
    } while(pitch_time == 0 && yaw_time == 0);

    yaw_target = 0;
    pitch_target = 0;
    StartMotion(&pitch_loop, pitch_time);
    StartMotion(&yaw_loop, yaw_time);

    printf("\n****Yaw PID Parameters****\n");
    printf("Kp = %f\n", yaw_loop.param.kp);
    printf("Ki = %f\n", yaw_loop.param.ki);
    printf("Kd = %f\n", yaw_loop.param.kd);
    printf("Torque Limit High=%f\n", yaw_loop.param.cmd_lim.high);
    printf("Torque Limit Low=%f\n", yaw_loop.param.cmd_lim.low);
    printf("Target = %f\n", yaw_target);
    printf("****Yaw PID Parameters****\n");

    printf("\n****Pitch PID Parameters****\n");
    printf("Kp = %f\n", pitch_loop.param.kp);
    printf("Ki = %f\n", pitch_loop.param.ki);
    printf("Kd = %f\n", pitch_loop.param.kd);
    printf("Torque Limit High=%f\n", pitch_loop.param.cmd_lim.high);
    printf("Torque Limit Low=%f\n", pitch_loop.param.cmd_lim.low);
    printf("Target = %f\n", pitch_target);
    printf("****Pitch PID Parameters****\n");

    while(1) {
        // Poll waiting for new data
        do {
            pthread_mutex_lock(&pitch_feedback->mutex);
            pitch_time = pitch_feedback->time;
            pitch_pos = pitch_feedback->pos;
            pthread_mutex_unlock(&pitch_feedback->mutex);

            pthread_mutex_lock(&yaw_feedback->mutex);
            yaw_time = yaw_feedback->time;
            yaw_pos = yaw_feedback->pos;
            pthread_mutex_unlock(&yaw_feedback->mutex);
        } while(yaw_time == yaw_loop.prevTime && pitch_time == pitch_loop.prevTime);

        time = (yaw_time > pitch_time) ? yaw_time : pitch_time;
        force_avg = UpdateLoop(&pitch_loop, pitch_target, pitch_pos, time);
        force_diff = UpdateLoop(&yaw_loop, yaw_target, yaw_pos, time);
        // f(right) - f(left) = force_diff
        // -(f(right) + f(left))/2 = force_avg
        SetEyeForce(left_axis, force_avg-0.5*force_diff);
        SetEyeForce(right_axis, force_avg+0.5*force_diff);

        if(time > 10) {
            break;
        }
    }
    PrintNeckData();
}

int main()
{
    eye_ctrl();
}