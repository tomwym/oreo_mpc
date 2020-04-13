#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <stddef.h>
#include <pthread.h>
#include <errno.h>

#include "pid.h"
#include "libmacaque_RS232/macaque_linux.h"
#include "TML_RS232_lib/include/TML_RS232_lib.h"
#include "sock_interface/sock.h"

static eyeCalData_t* eyeCalData = NULL;
static eyeData_t* eyeData = NULL;
static neckData_t* neckData = NULL;
static double eyeCalPos[NUM_EYE_AXIS] = {10, 10, 10, 10};
static double eyeOffset[NUM_EYE_AXIS] = {0};

#define EYE_POS_LIM                 {.low = 0, .high = 10}
#define NECK_POS_LIM                {.low = 0, .high = 10}
#define NECK_YAW_POS_LIM            {.low = 0, .high = 10}
#define INTEG_LIM                   {.low = 0, .high = 10}
#define EYE_TORQUE_LIM              {.low = 0, .high = 10}
#define NECK_TORQUE_LIM             {.low = 0, .high = 10}
#define NECK_YAW_TORQUE_LIM         {.low = 0, .high = 10}
#define CONTROL_THREAD_PRIO         (31)

static pid_loop_t pid_neck_yaw = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = NECK_YAW_TORQUE_LIM, .pos_lim = NECK_YAW_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_neck_pitch = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = NECK_TORQUE_LIM, .pos_lim = NECK_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_neck_roll = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = NECK_TORQUE_LIM, .pos_lim = NECK_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_eye_yaw_left = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_eye_yaw_right = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_eye_pitch_left = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_eye_pitch_right = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static void print_cmd_menu()
{
    printf("\n****CMD MENU****\n");
    printf("[d] set desired posn\n");
    printf("[s] start torque ctrl\n");
    printf("[t] display target posns\n");
    printf("[q] quit\n");
    printf("[h] help\n");
    printf("****CMD MENU****\n");
}

typedef enum {
    MENU_NECK_YAW = 1,
    MENU_NECK_PITCH,
    MENU_NECK_ROLL,
    MENU_EYE_YAW_LEFT,
    MENU_EYE_YAW_RIGHT,
    MENU_EYE_PITCH_LEFT,
    MENU_EYE_PITCH_RIGHT,
    NUM_MOTORS = MENU_EYE_PITCH_RIGHT,
} motor_menu_id_t;

typedef struct {
    pid_loop_t* pid_loop;
    pos_t* feedback;
    double pos_offset;
    setTorqueFn callback;
    uint8_t axis_id;
    char* name;
    pthread_t thread;
} motor_ctrl_t;

static motor_ctrl_t neck_ctrl[NUM_NECK_AXIS] = {
    {.pid_loop = &pid_neck_yaw, .feedback = NULL, .pos_offset = 0,
        .callback = &SetNeckTorque, .axis_id = NECK_YAW_AXIS, .name = "Neck Yaw"},
    {.pid_loop = &pid_neck_pitch, .feedback = NULL, .pos_offset = 0, 
        .callback = &SetNeckTorque, .axis_id = NECK_PITCH_AXIS, .name = "Neck Pitch"},
    {.pid_loop = &pid_neck_roll, .feedback = NULL, .pos_offset = 0, 
        .callback = &SetNeckTorque, .axis_id = NECK_ROLL_AXIS, .name = "Neck Roll"},
};

static motor_ctrl_t eye_ctrl[NUM_EYE_AXIS] = {
    {.pid_loop = &pid_eye_yaw_left, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = EYE_YAW_LEFT_AXIS, .name = "Eye Yaw Left"},
    {.pid_loop = &pid_eye_yaw_right, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = EYE_YAW_RIGHT_AXIS, .name = "Eye Yaw Right"},
    {.pid_loop = &pid_eye_pitch_left, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = EYE_PITCH_LEFT_AXIS, .name = "Eye Pitch Left"},
    {.pid_loop = &pid_eye_pitch_right, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = EYE_PITCH_RIGHT_AXIS, .name = "Eye Pitch Right"},
};

static pthread_t loop_handle[NUM_MOTORS]; 

void* ControlLoop(void* input)
{
    int err;
    double time, pos, torque;
    motor_ctrl_t* ctrl = (motor_ctrl_t*)input;
    struct sched_param param = {CONTROL_THREAD_PRIO};
    if(sched_setscheduler(0, SCHED_RR, &param) != 0) {
        err = errno;
        printf("Failed to set TX Thread priority with error %d\n", err);
        printf("Exiting %s pid control loop\n", ctrl->name);
        pthread_exit(NULL);
    }
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    // Make sure recving feedback from motors
    do {
        pthread_mutex_lock(&ctrl->feedback->mutex);
        time = ctrl->feedback->time;
        pthread_mutex_unlock(&ctrl->feedback->mutex);
    } while(time != 0);
    StartMotion(ctrl->pid_loop, time);
    
    while(1) {
        // Poll waiting for new data
        do {
            pthread_mutex_lock(&ctrl->feedback->mutex);
            time = ctrl->feedback->time;
            pos = ctrl->feedback->pos - ctrl->pos_offset;
            pthread_mutex_unlock(&ctrl->feedback->time);
        } while(time == ctrl->pid_loop->prevTime);

        torque = UpdateLoop(ctrl->pid_loop, pos, time);
        ctrl->callback(ctrl->axis_id, torque);
    }

    printf("Exiting %s pid control loop\n", ctrl->name);
    pthread_exit(NULL);
}

static void print_motor_menu()
{
    printf("\n****MOTOR MENU****\n");
    printf("[1] neck_yaw\n");
    printf("[2] neck_pitch\n");
    printf("[3] neck_roll\n");
    printf("[4] eye_yaw_left\n");
    printf("[5] eye_yaw_right\n");
    printf("[6] eye_pitch_left\n");
    printf("[7] eye_pitch_right\n");
    printf("****MOTOR MENU****\n");
}

int main()
{
    int err;
    eyeCalData = GetEyeCalData();
    eyeData = GetEyeData();
    neckData = GetNeckData();

    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        neck_ctrl[i].feedback = &neckData->pos[i];
    }
    for(int j = 0; j < NUM_EYE_AXIS; j++) {
        eye_ctrl[j].feedback = &eyeData->pos[j];
    }

    // Wait for neck to run start up
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf("Neck axis %d startup routine....", i+1);
        while(neckData->ready[i] == 0) {}
        printf("done\n");

        neck_ctrl[i].feedback = &neckData->pos[i];
    }

    // Eye calibration routine
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf("Eye axis %d calibration routine....");
        StartEyeCal(i, eyeCalPos[i]);

        // May need to call calibration module here
        
        // Poll waiting for completion
        while(eyeCalData->complete[i] == CAL_COMPLETE) {}
        printf("done\n");
        
        // Get the position offset 
        eye_ctrl[i].pos_offset = eyeCalData->offset[i];
    }

    while(1) {
        int c;
        double data;
        while ((c = getchar()) == '\n' || c == EOF) { }
        switch(c) {
            case 'd':
                // set desired posn
                print_motor_menu();
                while ((c = getchar()) == '\n' || c == EOF) { }
                c = c - '0';                
                printf("Enter target:");
                if(scanf("%lf", &data) == 1) {
                    switch(c) {
                        case MENU_NECK_YAW:
                        case MENU_NECK_PITCH:
                        case MENU_NECK_ROLL:
                            neck_ctrl[c-MENU_NECK_YAW].pid_loop->target = data;
                            printf("Neck axis %d target changed to %f", c, data);
                            break;

                        case MENU_EYE_YAW_LEFT:
                        case MENU_EYE_YAW_RIGHT:
                        case MENU_EYE_PITCH_LEFT:
                        case MENU_EYE_PITCH_RIGHT:
                            eye_ctrl[c-MENU_EYE_YAW_LEFT].pid_loop->target = data;
                            printf("Eye axis %d target changed to %f", c-MENU_EYE_YAW_LEFT+1, data);
                            break;
                        default:
                            printf("Invalid motor selection %d\n", c);
                            break;
                    }
                } else {
                    printf("Invalid target posn\n");
                }
                break;

            case 't':
                // display target posns
                printf("\nTARGET POSNS\n");
                for(int i = 0; i < NUM_NECK_AXIS; i++) {
                    printf("%s %lf\n", neck_ctrl[i].name, neck_ctrl[i].pid_loop->target);
                }
                break;

            case 's':
                // Set motion modes
                InitNeckTorqueCtrl();
                InitEyeForceCtrl();

                // Start pid threads
                for(int i = 0; i < NUM_NECK_AXIS; i++) {
                    if(pthread_create(&neck_ctrl[i].thread, NULL, ControlLoop, &neck_ctrl[i]) != 0) {
                        err = errno;
                        printf("Could not create thread for %s with errno=%d\n", neck_ctrl[i].name, err);
                    }
                }
                for(int i = 0; i < NUM_EYE_AXIS; i++) {
                    if(pthread_create(&eye_ctrl[i].thread, NULL, ControlLoop, &eye_ctrl[i]) != 0) {
                        err = errno;
                        printf("Could not create thread for %s with errno=%d\n", eye_ctrl[i].name, err);
                    }
                }

                while(1) {
                    while ((c = getchar()) == '\n' || c == EOF) {}
                    if(c == 's') {
                        break;
                    } else {
                        printf("press 's' to stop pid ctrl\n");
                    }
                }
                
                for(int i = 0; i < NUM_NECK_AXIS; i++) {
                    pthread_cancel(neck_ctrl[i].thread);
                    if(pthread_join(neck_ctrl[i].thread, NULL) != 0) {
                        err = errno;
                        printf("Could not join thread with %s with errno=%d\n", neck_ctrl[i].name, err);
                    }
                }
                for(int i = 0; i < NUM_EYE_AXIS; i++) {
                    pthread_cancel(eye_ctrl[i].thread);
                    if(pthread_join(eye_ctrl[i].thread, NULL) != 0) {
                        err = errno;
                        printf("Could not join thread with %s with errno=%d\n", eye_ctrl[i].name, err);
                    }
                }
                DisableEyeCtrl();
                DisableNeckCtrl();
                break;

            case 'q':
                // quit
                goto done;

            default:
                print_cmd_menu();
                break;
        }
        while ((c = getchar()) != '\n' && c != EOF) { };
    }

    done:
    printf("Disabling axes....");
    usleep(10000000);
    DisableEyeCtrl();
    DisableNeckCtrl();
    printf("done\n");
}