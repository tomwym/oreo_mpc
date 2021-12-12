#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#include "control/pid.h"
#include "libmacaque_RS232/macaque_linux.h"

/*static double get_timestamp()
{
    struct timespec curr;
    clock_gettime(CLOCK_MONOTONIC, &curr);
    return (double)curr.tv_sec + (double)curr.tv_nsec/(1000*1000*1000);
}*/

void StartMotion(pid_loop_t* loop, double initTime)
{
    loop->prevErr = 0;
    loop->integ = 0;
    loop->prevTime = initTime;
}

double UpdateLoop(pid_loop_t* loop, double targetPos, double currPos, double time)
{
    // Time since last loop
    double elapsedTime = time - loop->prevTime;
    loop->prevTime = time;
    // Calc current loop parameters
    double error = targetPos - currPos;
    loop->integ += error*elapsedTime;
    // Integral error limits to solve integrator windup
    if(loop->integ > loop->param.cmd_lim.high) {
        loop->integ = loop->param.cmd_lim.high;
    } else if(loop->integ < loop->param.cmd_lim.low) {
        loop->integ = loop->param.cmd_lim.low;
    }
    double deriv = (error - loop->prevErr)/elapsedTime;
    double cmd = (double)loop->param.kp*error + (double)loop->param.ki*loop->integ + (double)loop->param.kd*deriv;
    loop->prevErr = error;

    // Limit currents drawn by motors
    if(cmd > loop->param.cmd_lim.high) {
        printf("Pos torque lim reached cmd=%f\n", cmd);
        cmd = loop->param.cmd_lim.high;
    } else if(cmd < loop->param.cmd_lim.low) {
        printf("Neg torque lim reached cmd=%f\n", cmd);
        cmd = loop->param.cmd_lim.low;
    }

    // Prevent robots from slamming at limits
    if(currPos > loop->param.pos_lim.high) {
        printf("Pos posn lim reached pos=%.3f\n", currPos);
        cmd = 0;
    } else if(currPos < loop->param.pos_lim.low) {
        printf("Neg posn limit reached pos=%.3f\n", currPos);
        cmd = 0;
    }
    
    return cmd;
}