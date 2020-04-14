#ifndef PID_H
#define PID_H
#include <stdint.h>
#include <stdbool.h>
#include "libmacaque_RS232/macaque_linux.h"

typedef struct _lim_t {
    double high;
    double low;
} lim_t;

typedef void(*setTorqueFn)(uint8_t, double);

typedef struct _pid_param_t {
    uint8_t kp;
    uint8_t ki;
    uint8_t kd;
    lim_t integ_lim;
    lim_t cmd_lim;
    lim_t pos_lim;
} pid_param_t;

typedef struct _pid_loop_t {
    pid_param_t param;
    double errSum;
    double prevErr;
    double target;
    double prevTime;
} pid_loop_t;

void StartMotion(pid_loop_t* loop, double initTime);
double UpdateLoop(pid_loop_t* loop, double pos, double time);

#endif