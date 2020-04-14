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

#define DEBUG_MENU

static eyeCalData_t* eyeCalData = NULL;
static eyeData_t* eyeData = NULL;
static neckData_t* neckData = NULL;
#ifndef DEBUG_MENU
static double eyeCalPos[NUM_EYE_AXIS] = {10, 10, 10, 10};
#endif

#define EYE_POS_LIM                 {.low = 0, .high = 10}
#define NECK_POS_LIM                {.low = 0, .high = 10}
#define NECK_YAW_POS_LIM            {.low = 0, .high = 10}
#define INTEG_LIM                   {.low = 0, .high = 10}
#define EYE_TORQUE_LIM              {.low = 0, .high = 10}
#define NECK_TORQUE_LIM             {.low = 0, .high = 10}
#define NECK_YAW_TORQUE_LIM         {.low = 0, .high = 10}
#define CONTROL_THREAD_PRIO         (31)
#define BUFSIZE                     (80)

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

static pid_loop_t pid_left_eye_left = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_left_eye_right = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_right_eye_left = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

static pid_loop_t pid_right_eye_right = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .integ_lim = INTEG_LIM, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .target = 0, .prevTime = 0
};

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
        .callback = &SetNeckTorque, .axis_id = NECK_YAW_AXIS, .name = "Neck Yaw Axis"},
    {.pid_loop = &pid_neck_pitch, .feedback = NULL, .pos_offset = 0, 
        .callback = &SetNeckTorque, .axis_id = NECK_PITCH_AXIS, .name = "Neck Pitch Axis"},
    {.pid_loop = &pid_neck_roll, .feedback = NULL, .pos_offset = 0, 
        .callback = &SetNeckTorque, .axis_id = NECK_ROLL_AXIS, .name = "Neck Roll Axis"},
};

static motor_ctrl_t eye_ctrl[NUM_EYE_AXIS] = {
    {.pid_loop = &pid_left_eye_left, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = LEFT_EYE_LEFT_AXIS, .name = "Left Eye Left Axis"},
    {.pid_loop = &pid_left_eye_right, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = LEFT_EYE_RIGHT_AXIS, .name = "Left Eye Right Axis"},
    {.pid_loop = &pid_right_eye_left, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = RIGHT_EYE_LEFT_AXIS, .name = "Right Eye Left Axis"},
    {.pid_loop = &pid_right_eye_right, .feedback = NULL, .pos_offset = 0,
        .callback = &SetEyeForce, .axis_id = RIGHT_EYE_RIGHT_AXIS, .name = "Right Eye Right Axis"},
};

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
            pthread_mutex_unlock(&ctrl->feedback->mutex);
        } while(time == ctrl->pid_loop->prevTime);

        torque = UpdateLoop(ctrl->pid_loop, pos, time);
        ctrl->callback(ctrl->axis_id, torque);
    }

    printf("Exiting %s pid control loop\n", ctrl->name);
    pthread_exit(NULL);
}

static void print_cmd_menu()
{
    printf("\n****CMD MENU****\n");
    printf("[e] set eye target posn\n");
    printf("[n] set neck target posn\n");
    printf("[f] set target posn from file\n");
    printf("[t] display target posns\n");
    printf("[s] start ctrl loops\n");
    printf("[q] quit\n");
    printf("[h] help\n");
    printf("****CMD MENU****\n");
}

static void print_eye_menu()
{
    printf("\n****EYE MOTOR MENU****\n");
    /*printf("[1] left_eye_left_axis\n");
    printf("[2] eye_yaw_right_axis\n");
    printf("[3] eye_pitch_left_axis\n");
    printf("[4] eye_pitch_right_axis\n");*/
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf("[%d] %s\n", eye_ctrl[i].axis_id, eye_ctrl[i].name);
    }
    printf("[q] back to main\n");
    printf("****EYE MOTOR MENU****\n");
}

static void print_neck_menu()
{
    printf("\n****NECK MOTOR MENU****\n");
    /*printf("[1] neck_yaw\n");
    printf("[2] neck_pitch\n");
    printf("[3] neck_roll\n");*/
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf("[%d] %s\n", neck_ctrl[i].axis_id, neck_ctrl[i].name);
    }
    printf("[q] back to main\n");
    printf("****NECK MOTOR MENU****\n");
}

static void read_file()
{
    int err = 0;
    char fileName[BUFSIZE], buff[BUFSIZE]; 
    FILE* file = NULL;
    double dbuff[NUM_EYE_AXIS] = {0};
    printf("Enter file name (must be .csv):");
    if(scanf("%s", fileName) != 1) {
        printf("Invalid file name input\n");
        return;
    }
    // .csv only
    if(strstr(fileName, ".csv") == NULL) {
        printf("Invalid file type\n");
        return;
    }
    if((file = fopen(fileName, "r")) == NULL) {
        err = errno;
        printf("Failed to open file %s with errno=%d\n", fileName, err);
        return;
    }
    // One line for neck & eye each
    // Expecting following format:
    // val,val,val\n
    // val,val,val,val\n
    for(int i = 0; i < 2; i++) {
        // Read a line
        if(fgets(buff, sizeof(buff), file) == NULL) {
            err = ferror(file);
            printf("Read error occurred err=%d\n", err);
            return;
        }
        if(strlen(buff)+1 == sizeof(buff)) {
            printf("Buffer overflow while reading line %d\n", i+1);
            return;
        }
        int entries = sscanf(buff, "%lf,%lf,%lf,%lf\n", &dbuff[0], &dbuff[1], &dbuff[2], &dbuff[3]);
        if(entries == NUM_NECK_AXIS) {
            for(int j = 0; j < NUM_NECK_AXIS; j++) {
                neck_ctrl[j].pid_loop->target = dbuff[j];
            }
        } else if(entries == NUM_EYE_AXIS) {
            for(int j = 0; j < NUM_EYE_AXIS; j++) {
                eye_ctrl[j].pid_loop->target = dbuff[j];
            }
        } else {
            printf("Unable to parse line %d", i+1);
        }
    }
}

static void display_target_posns()
{
    printf("\n****TARGET POSNS****\n");
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf("%s %.3f\n", neck_ctrl[i].name, neck_ctrl[i].pid_loop->target);
    }
    for(int j = 0; j < NUM_EYE_AXIS; j++) {
        printf("%s %.3f\n", eye_ctrl[j].name, eye_ctrl[j].pid_loop->target);
    }
    printf("****TARGET POSNS****\n");
}

int main()
{
#ifndef DEBUG_MENU
    int err;
#endif
    eyeCalData = GetEyeCalData();
    eyeData = GetEyeData();
    neckData = GetNeckData();

    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        neck_ctrl[i].feedback = &neckData->pos[i];
    }
    for(int j = 0; j < NUM_EYE_AXIS; j++) {
        eye_ctrl[j].feedback = &eyeData->pos[j];
    }

#ifndef DEBUG_MENU
    // Wait for neck to run start up
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf("Neck axis %d startup routine....", i+1);
        while(neckData->ready[i] == 0) {}
        printf("done\n");

        neck_ctrl[i].feedback = &neckData->pos[i];
    }

    // Eye calibration routine
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        printf("Eye axis %d calibration routine....", i+1);
        StartEyeCal(i, eyeCalPos[i]);

        // May need to call calibration module here
        
        // Poll waiting for completion
        while(eyeCalData->complete[i] == CAL_COMPLETE) {}
        printf("done\n");
        
        // Get the position offset 
        eye_ctrl[i].pos_offset = eyeCalData->offset[i];
    }
    #endif

    int c;
    double data;
    print_cmd_menu();
    while(1) {
        while ((c = getchar()) == '\n' || c == EOF) { }
        switch(c) {
            case 'e':
                // set eye target posn
                print_eye_menu();
                while ((c = getchar()) == '\n' || c == EOF) { }
                if((char)c == 'q') {
                    // back to main menu
                    break;
                }
                c = c - '0';
                if(c > 0 && c <= NUM_EYE_AXIS) {
                    printf("Enter target:");
                    if(scanf("%lf", &data) == 1) {
                        eye_ctrl[c-1].pid_loop->target = data;
                    } else {
                        printf("Invalid target posn\n");
                    }
                } else {
                    printf("Invalid eye axis id\n");
                }
                display_target_posns();
                break;

            case 'n':
                // set neck target posn
                print_neck_menu();
                while ((c = getchar()) == '\n' || c == EOF) { }
                if((char)c == 'q') {
                    // back to main menu
                    break;
                }
                c = c - '0';
                if(c > 0 && c <= NUM_NECK_AXIS) {
                    printf("Enter target:");
                    if(scanf("%lf", &data) == 1) {
                        neck_ctrl[c-1].pid_loop->target = data;
                    } else {
                        printf("Invalid target posn\n");
                    }
                } else {
                    printf("Invalid eye axis id\n");
                }
                display_target_posns();
                break;

            case 'f':
                // Load target positions from file
                read_file();
                display_target_posns();
                break;

            case 't':
                // display target posns
                display_target_posns();
                break;

            case 's':
#ifndef DEBUG_MENU
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
#endif                
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