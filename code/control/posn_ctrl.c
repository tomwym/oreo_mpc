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
#define EYE_TORQUE_LIM              {.low = 0, .high = 10}
#define NECK_TORQUE_LIM             {.low = 0, .high = 10}
#define NECK_YAW_TORQUE_LIM         {.low = 0, .high = 10}
#define HOME_EYE_SPD                (1.5) // rad/s
#define HOME_EYE_ACCEL              (1.5) // rad/s^2
#define HOME_NECK_SPD               (1.5) // rad/s
#define HOME_NECK_ACCEL             (1.5) // rad/s
#define CONTROL_THREAD_PRIO         (31)
#define BUFSIZE                     (100)
#define NUM_COLUMNS                 (2)

static pid_loop_t pid_neck_yaw = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = NECK_YAW_TORQUE_LIM, .pos_lim = NECK_YAW_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

static pid_loop_t pid_neck_pitch = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = NECK_TORQUE_LIM, .pos_lim = NECK_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

static pid_loop_t pid_neck_roll = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = NECK_TORQUE_LIM, .pos_lim = NECK_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

static pid_loop_t pid_left_eye_pitch = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

static pid_loop_t pid_left_eye_yaw = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

static pid_loop_t pid_right_eye_pitch = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

static pid_loop_t pid_right_eye_yaw = {
    .param = {.kp = 0, .ki = 0, .kd = 0, .cmd_lim = EYE_TORQUE_LIM, .pos_lim = EYE_POS_LIM},
    .errSum = 0, .prevErr = 0, .integ = 0, .prevTime = 0
};

typedef struct {
    double* posn;
    double* time;
    uint32_t numPosn;
} target_t;

typedef struct {
    pid_loop_t* pid_loop;
    pos_t* feedback;
    setTorqueFn callback;
    uint8_t axis_id;
    char* name;
    pthread_t thread;
    target_t target;
} neck_ctrl_t;

typedef struct {
    pid_loop_t* pitch_loop;
    pid_loop_t* yaw_loop;
    pos_t* pitch_feedback;
    pos_t* yaw_feedback;
    setTorqueFn callback;
    uint8_t left_axis;
    uint8_t right_axis;
    char* name;
    pthread_t thread;
    target_t pitch_target;
    target_t yaw_target;
} eye_ctrl_t;

static neck_ctrl_t neck_ctrl[NUM_NECK_AXIS] = {
    {.pid_loop = &pid_neck_yaw, .feedback = NULL, .target = {.posn = NULL, .time = NULL, .numPosn = 0},
        .feedback = NULL, .callback = &SetNeckTorque, .axis_id = NECK_YAW_AXIS, .name = "Neck Yaw Axis"},
    {.pid_loop = &pid_neck_pitch, .feedback = NULL, .target = {.posn = NULL, .time = NULL, .numPosn = 0}, 
        .callback = &SetNeckTorque, .axis_id = NECK_PITCH_AXIS, .name = "Neck Pitch Axis"},
    {.pid_loop = &pid_neck_roll, .feedback = NULL, .target = {.posn = NULL, .time = NULL, .numPosn = 0},
        .callback = &SetNeckTorque, .axis_id = NECK_ROLL_AXIS, .name = "Neck Roll Axis"},
};

#define NUM_EYES    2
typedef enum {
    LEFT_EYE_CTRL_IDX,
    RIGHT_EYE_CTRL_IDX,
} eyeCtrlIdx_t;
static eye_ctrl_t eye_ctrl[NUM_EYES] = {
    {.pitch_loop = &pid_left_eye_pitch, .yaw_loop = &pid_right_eye_yaw, .pitch_feedback = NULL, .yaw_feedback = NULL,
        .callback = &SetEyeForce, .left_axis = LEFT_EYE_LEFT_AXIS, .right_axis = RIGHT_EYE_RIGHT_AXIS, .name = "Left Eye",
            .pitch_target = {.posn = NULL, .time = NULL, .numPosn = 0}, .yaw_target = {.posn = NULL, .time = NULL, .numPosn = 0}},
    {.pitch_loop = &pid_right_eye_pitch, .yaw_loop = &pid_right_eye_yaw, .pitch_feedback = NULL, .yaw_feedback = NULL,
        .callback = &SetEyeForce, .left_axis = RIGHT_EYE_LEFT_AXIS, .right_axis = RIGHT_EYE_RIGHT_AXIS, .name = "Right Eye",
            .pitch_target = {.posn = NULL, .time = NULL, .numPosn = 0}, .yaw_target = {.posn = NULL, .time = NULL, .numPosn = 0}}
};

void* EyeControlLoop(void* input)
{
    int err, pitch_idx = 0, yaw_idx;
    double pitch_startTime = 0, pitch_time = 0, pitch_pos = 0, pitch_target = 0;
    double yaw_startTime = 0, yaw_time = 0, yaw_pos = 0, yaw_target = 0;
    double force_avg = 0, force_diff = 0, time = 0;
    eye_ctrl_t* ctrl = (eye_ctrl_t*)input;
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
        pthread_mutex_lock(&ctrl->pitch_feedback->mutex);
        pitch_time = ctrl->pitch_feedback->time;
        pthread_mutex_unlock(&ctrl->pitch_feedback->mutex);

        pthread_mutex_lock(&ctrl->yaw_feedback->mutex);
        yaw_time = ctrl->yaw_feedback->time;
        pthread_mutex_unlock(&ctrl->yaw_feedback->mutex);
    } while(pitch_time == 0 || yaw_time == 0);
    StartMotion(ctrl->yaw_loop, yaw_time);
    StartMotion(ctrl->pitch_loop, pitch_time);
    yaw_startTime = yaw_time;
    pitch_startTime = pitch_time;

    while(1) {
        // Poll waiting for new data
        do {
            pthread_mutex_lock(&ctrl->pitch_feedback->mutex);
            pitch_time = ctrl->pitch_feedback->time;
            pitch_pos = ctrl->pitch_feedback->pos;
            pthread_mutex_unlock(&ctrl->pitch_feedback->mutex);

            pthread_mutex_lock(&ctrl->yaw_feedback->mutex);
            yaw_time = ctrl->yaw_feedback->time;
            yaw_pos = ctrl->yaw_feedback->pos;
            pthread_mutex_unlock(&ctrl->yaw_feedback->mutex);
        } while(yaw_time == ctrl->yaw_loop->prevTime && pitch_time == ctrl->pitch_loop->prevTime);

        time = (yaw_time > pitch_time) ? yaw_time : pitch_time;
        if(time - pitch_startTime > ctrl->pitch_target.time[pitch_idx]) {
            pitch_idx++;
            if(pitch_idx >= ctrl->pitch_target.numPosn) {
                break;
            }
            pitch_target = ctrl->pitch_target.posn[pitch_idx];
        }
        if(time - yaw_startTime > ctrl->yaw_target.posn[yaw_idx]) {
            yaw_idx++;
            if(yaw_idx >= ctrl->yaw_target.numPosn) {
                break;
            }
            yaw_target = ctrl->yaw_target.posn[yaw_idx];
        }
        force_avg = UpdateLoop(&ctrl->pitch_loop, pitch_target, pitch_pos, time);
        force_diff = UpdateLoop(&ctrl->yaw_loop, yaw_target, yaw_pos, time);

        // f(right) - f(left) = force_diff
        // -(f(right) + f(left))/2 = force_avg
        SetEyeForce(ctrl->left_axis, force_avg-0.5*force_diff);
        SetEyeForce(ctrl->right_axis, force_avg+0.5*force_diff);
    }

}

void* ControlLoop(void* input)
{
    int err, idx = 0;
    double startTime, time, pos, torque, target;
    neck_ctrl_t* ctrl = (neck_ctrl_t*)input;
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
    } while(time == 0);
    StartMotion(ctrl->pid_loop, time);
    startTime = time;
    
    while(1) {
        // Poll waiting for new data
        do {
            pthread_mutex_lock(&ctrl->feedback->mutex);
            time = ctrl->feedback->time;
            pos = ctrl->feedback->pos;
            pthread_mutex_unlock(&ctrl->feedback->mutex);
        } while(time == ctrl->pid_loop->prevTime);

        if((time - startTime) >= ctrl->target.time[idx]) {
            idx++;
            target = ctrl->target.posn[idx];
        }

        torque = UpdateLoop(ctrl->pid_loop, target, pos, time);
        ctrl->callback(ctrl->axis_id, torque);
    }

    printf("Exiting %s pid control loop\n", ctrl->name);
    pthread_exit(NULL);
}

static void print_cmd_menu()
{
    printf("\n****CMD MENU****\n");
    printf("[e] load eye target file\n");
    printf("[n] load neck target file\n");
    printf("[s] start posn ctrl\n");
    printf("[h] go to home posn\n");
    printf("[q] quit\n");
    printf("[h] help\n");
    printf("****CMD MENU****\n");
}

typedef enum {
    MENU_LEFT_EYE_YAW = 1,
    MENU_LEFT_EYE_PITCH,
    MENU_RIGHT_EYE_YAW,
    MENU_RIGHT_EYE_PITCH,
} eye_menu_t;

static void print_eye_menu()
{
    printf("\n****EYE MENU****\n");
    printf("[1] Left Eye Yaw\n");
    printf("[2] Left Eye Pitch\n");
    printf("[3] Right Eye Yaw\n");
    printf("[4] Right Eye Pitch\n");
    printf("[q] back to main\n");
    printf("****EYE MENU****\n");
}

static void print_neck_menu()
{
    printf("\n****NECK MOTOR MENU****\n");
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf("[%d] %s\n", neck_ctrl[i].axis_id, neck_ctrl[i].name);
    }
    printf("[q] back to main\n");
    printf("****NECK MOTOR MENU****\n");
}

static int read_file(target_t* target)
{
    int err = 0;
    char fileName[BUFSIZE], buff[BUFSIZE]; 
    FILE* file = NULL;
    double dbuff[NUM_COLUMNS] = {0};
    printf("Enter file name (must be .csv):");
    if(scanf("%s", fileName) != 1) {
        printf("Invalid file name input\n");
        return -1;
    }
    // .csv only
    if(strstr(fileName, ".csv") == NULL) {
        printf("Invalid file type\n");
        return -1;
    }
    if((file = fopen(fileName, "r")) == NULL) {
        err = errno;
        printf("Failed to open file %s with errno=%d\n", fileName, err);
        return -1;
    }
    // One line for neck & eye each
    // Expecting following format:
    // time,posn
    target->numPosn = 0;
    target->posn = (double*)malloc(sizeof(*(target->posn)) * BUFSIZE);
    double *tempPosn = NULL;
    target->time = (double*)malloc(sizeof(*(target->time)) * BUFSIZE);
    double *tempTime = NULL;
    while(fgets(buff,sizeof(buff), file) != NULL) {
        // Extend memory if more positions
        if(target->numPosn % BUFSIZE == 0 && target->numPosn != 0) {
            tempPosn = (double*)realloc(target->posn, sizeof(*(target->posn)) * (target->numPosn+BUFSIZE));
            tempTime = (double*)realloc(target->time, sizeof(*(target->time)) * (target->numPosn+BUFSIZE));
            if(tempPosn == NULL || tempTime == NULL) {
                printf("realloc failure\n");
                free(target->posn);
                target->posn = NULL;
                free(target->time);
                target->time = NULL;
                target->numPosn = 0;
                fclose(file);
                return -1;
            }
            target->posn = tempPosn;
            target->time = tempTime;
        }
        if(strlen(buff)+1 == sizeof(buff)) {
            printf("Buffer overflow while reading line %d\n", target->numPosn+1);
            continue;
        }
        int entries = sscanf(buff, "%lf,%lf\n", &target->time[target->numPosn], &target->posn[target->numPosn]);
        if(entries != NUM_COLUMNS) {
            printf("Unable to parse target posn %d", target->numPosn+1);
            continue;
        }
        target->numPosn++;
    }

    fclose(file);
    return 0;
}

static void read_eye_target()
{
    int c = 0;
    double data = 0;
    target_t* target = NULL;

    print_eye_menu();
    while ((c = getchar()) == '\n' || c == EOF) { }
    if((char)c == 'q') {
        // back to main menu
        return;
    }
    c = c - '0';
    switch(c) {
        case MENU_LEFT_EYE_YAW:
            target = &eye_ctrl[LEFT_EYE_CTRL_IDX].yaw_target;
            break;
        case MENU_LEFT_EYE_PITCH:
            target = &eye_ctrl[LEFT_EYE_CTRL_IDX].pitch_target;
            break;
        case MENU_RIGHT_EYE_YAW:
            target = &eye_ctrl[RIGHT_EYE_CTRL_IDX].yaw_target;
            break;
        case MENU_RIGHT_EYE_PITCH:
            target = &eye_ctrl[RIGHT_EYE_CTRL_IDX].pitch_target;
            break;
        default:
            printf("Invalid eye axis id\n");
            break;
    }

    if(read_file(target) < 0) {
        printf("Failed to read target positions\n");
        return;
    }
}

static void read_neck_target()
{
    int c = 0;
    double data = 0;
    // set neck target posn
    print_neck_menu();
    while ((c = getchar()) == '\n' || c == EOF) { }
    if((char)c == 'q') {
        // back to main menu
        return;
    }
    c = c - '0';
    if(c > 0 && c <= NUM_NECK_AXIS) {
        if(read_file(&neck_ctrl[c-1].target) < 0) {
            printf("Failed to parse %s target file\n", neck_ctrl[c-1].name);
            return;
        }
    } else {
        printf("Invalid neck axis id\n");
    }

}

static void start_ctrl_loops()
{
    int err = 0, c = 0;
    bool ready = true;
    
    // Make sure target positions loaded
    for(int i = 0; i < NUM_EYES; i++) {
        if(eye_ctrl[i].pitch_target.posn == NULL || eye_ctrl[i].pitch_target.time == NULL 
            || eye_ctrl[i].pitch_target.numPosn == 0) {
            printf("%s pitch targets not loaded\n", eye_ctrl[i].name);
            ready = false;
        }
        if(eye_ctrl[i].yaw_target.posn == NULL || eye_ctrl[i].yaw_target.time == NULL 
            || eye_ctrl[i].yaw_target.numPosn == 0) {
            printf("%s yaw targets not loaded\n", eye_ctrl[i].name);
            ready = false;
        }
    }
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        if(neck_ctrl[i].target.posn == NULL || neck_ctrl[i].target.time == NULL 
            || neck_ctrl[i].target.numPosn == 0) {
            printf("%s targets not loaded\n", neck_ctrl[i].name);
            ready = false;
        }
    }

    if(!ready) {
        printf("Could not start ctrl loops\n");
        return;
    }

    // Set motion modes
    InitNeckTorqueCtrl(0);
    InitEyeForceCtrl(0);

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
    
    // Wait for trajectories to finish
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        if(pthread_join(neck_ctrl[i].thread, NULL) != 0) {
            err = errno;
            printf("Could not join thread with %s with errno=%d\n", neck_ctrl[i].name, err);
        }
    }
    
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        if(pthread_join(eye_ctrl[i].thread, NULL) != 0) {
            err = errno;
            printf("Could not join thread with %s with errno=%d\n", eye_ctrl[i].name, err);
        }
    }
    DisableEyeCtrl();
    DisableNeckCtrl();
}

static void go_home()
{
    InitEyePosnCtrl();
    SetEyePosn(0, 0);
    SetEyeSpeed(0, HOME_EYE_SPD);
    SetEyeAccel(0, HOME_EYE_ACCEL);
    UpdateEye(0);
    DisableEyeCtrl();
    
    InitNeckPosnCtrl();
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        SetNeckPosn(i+1, 0);
        SetNeckSpeed(i+1, HOME_NECK_SPD);
        SetNeckAccel(i+1, HOME_NECK_ACCEL);
    }
    UpdateNeck(0);
    DisableNeckCtrl();

}

int main()
{
    eyeCalData = GetEyeCalData();
    eyeData = GetEyeData();
    neckData = GetNeckData();

    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        neck_ctrl[i].feedback = &neckData->pos[i];
    }
    eye_ctrl[LEFT_EYE_CTRL_IDX].pitch_feedback = &eyeData->pos[LEFT_EYE_PITCH_IDX];
    eye_ctrl[LEFT_EYE_CTRL_IDX].yaw_feedback = &eyeData->pos[LEFT_EYE_YAW_IDX];
    eye_ctrl[RIGHT_EYE_CTRL_IDX].pitch_feedback = &eyeData->pos[RIGHT_EYE_PITCH_IDX];
    eye_ctrl[RIGHT_EYE_CTRL_IDX].yaw_feedback = &eyeData->pos[RIGHT_EYE_YAW_IDX];

#ifndef DEBUG_MENU
    // Wait for neck to run start up
    for(int i = 0; i < NUM_NECK_AXIS; i++) {
        printf("Neck axis %d startup routine....", i+1);
        while(neckData->ready[i] == 0) {}
        printf("done\n");

        neck_ctrl[i].feedback = &neckData->pos[i];
    }

    // Wait for eye to run start up
    for (int i = 0; i < NUM_EYE_AXIS; i++) {

        
        eye_ctrl[i].pos_offset = eyeData->offset[i];
    }
#endif

    go_home();

    int c;
    double data;
    print_cmd_menu();
    while(1) {
        while ((c = getchar()) == '\n' || c == EOF) { }
        switch(c) {
            case 'e':
                // Load eye target file
                read_eye_target();               
                break;

            case 'n':
                // Load neck target file
                read_neck_target();
                break;

            case 's':
#ifndef DEBUG_MENU
                start_ctrl_loops();
#else 
                printf("Disabled\n");
#endif
                break;
            
            case 'h':
#ifndef DEBUG_MENU
                go_home();
#else
                printf("Disabled\n");
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