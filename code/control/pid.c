#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#include "pid.h"
#include "libmacaque_RS232/macaque_linux.h"

/*static double get_timestamp()
{
    struct timespec curr;
    clock_gettime(CLOCK_MONOTONIC, &curr);
    return (double)curr.tv_sec + (double)curr.tv_nsec/(1000*1000*1000);
}*/

void StartMotion(pid_loop_t* loop, double initTime)
{
    loop->target = loop->target;
    loop->errSum = 0;
    loop->prevErr = 0;
    loop->prevTime = initTime;
}

double UpdateLoop(pid_loop_t* loop, double pos, double time)
{
    // Time since last loop
    double elapsedTime = time - loop->prevTime;
    loop->prevTime = time;
    // Calc current loop parameters
    double error = loop->target - pos;
    loop->errSum += error;
    // Integral error limits to solve integrator windup
    if(loop->errSum > loop->param.integ_lim.high) {
        loop->errSum = loop->param.integ_lim.high;
    } else if(loop->errSum < loop->param.integ_lim.low) {
        loop->errSum = loop->param.integ_lim.low;
    }
    double deriv = (error - loop->prevErr)/elapsedTime;
    double cmd = (double)loop->param.kp*error + (double)loop->param.ki*loop->errSum + (double)loop->param.kd*deriv;
    
    // Limit currents drawn by motors
    if(cmd > loop->param.cmd_lim.high) {
        printf("Pos torque lim reached\n");
        cmd = loop->param.cmd_lim.high;
    } else if(cmd < loop->param.cmd_lim.low) {
        printf("Neg torque lim reached\n");
        cmd = loop->param.cmd_lim.low;
    }

    // Prevent robots from slamming at limits
    if(pos > loop->param.pos_lim.high && cmd > 0) {
        printf("Pos posn lim reached pos=%.3f\n", pos);
        cmd = 0;
    } else if(pos > loop->param.pos_lim.low && cmd < 0) {
        printf("Neg posn limit reached pos=%.3f\n", pos);
        cmd = 0;
    }
    
    return cmd;
}