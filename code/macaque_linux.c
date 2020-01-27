/*
    TODO:
    - Replace some of the defines with enums
    - Think about having a socket to rcv error frames
    - sort out can interface name
    - Perhaps will need to re-implement connectDev() and disconnect Dev()
    - Replace SwitchToThread with Linux-equivalent
    - Figure out what to do for debug window

*/

// _DEFAULT_SOURCE defined for ifreq struct
#define _DEFAULT_SOURCE
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include "macaque_linux.h"
#include "./cansock_interface/cansock.h"
#include "TML_CAN_lib/include/TML_CAN_lib.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>

#include <math.h>
#include <sched.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/resource.h>

#define COUNTS_PER_POLE	((double)2048)
#define METERS_PER_POLE ((double)0.018)
#define IU_TO_M (METERS_PER_POLE/COUNTS_PER_POLE)
#define M_TO_IU (COUNTS_PER_POLE/METERS_PER_POLE)
#define COUNTS_PER_REV 	((double)5000)
#define COUNTS_PER_REV_YAW 	((double)16*86)
#define IU_TO_RAD (M_PI/(2*COUNTS_PER_REV))
#define RAD_TO_IU (1/IU_TO_RAD)
#define IU_TO_RAD_YAW (M_PI/(2*COUNTS_PER_REV_YAW))
#define RAD_TO_IU_YAW (1/IU_TO_RAD_YAW)

#define SAMPLE_PERIOD_EYE_S	0.0005
#define SAMPLE_PERIOD_NECK_S	0.001
	
#define MPSS_TO_ACCEL_IU		(SAMPLE_PERIOD_EYE_S * SAMPLE_PERIOD_EYE_S * M_TO_IU)
#define ACCEL_IU_TO_MPSS		(1.0 / MPSS_TO_ACCEL_IU)
#define MPS_TO_SPEED_IU		        (SAMPLE_PERIOD_EYE_S * M_TO_IU)
#define SPEED_IU_TO_MPS		        (1.0 / MPS_TO_SPEED_IU)

#define RADPSS_TO_ACCEL_IU		(SAMPLE_PERIOD_NECK_S * SAMPLE_PERIOD_NECK_S * RAD_TO_IU)
#define RADPSS_TO_ACCEL_IU_YAW	(SAMPLE_PERIOD_NECK_S * SAMPLE_PERIOD_NECK_S * RAD_TO_IU_YAW)
#define ACCEL_IU_TO_RADPSS		(1.0 / RADPSS_TO_ACCEL_IU)
#define RADPS_TO_SPEED_IU		(SAMPLE_PERIOD_NECK_S * RAD_TO_IU)
#define RADPS_TO_SPEED_IU_YAW    (SAMPLE_PERIOD_NECK_S * RAD_TO_IU_YAW)
#define SPEED_IU_TO_RADPS		(1.0 / RADPS_TO_SPEED_IU)

#define NANO_TO_SECS                (1000*1000*1000)
#define CURRENT_TID                 (0)
#define RX_THREAD_PRIORITY          (1)
#define TX_THREAD_PRIORITY          (10)
#define TX_THREAD_SLEEP             (100*1000) // us 

#define NECK_DEV_ID                 (0x00u)     // Is neck motor if MSB of source id in CAN_id is 0
#define NECK_CAN_IFNAME             "can0"
#define EYE_DEV_ID                  (0x80u)     // Is eye motor if MSB of source id in CAN_id is 1
#define EYE_CAN_IFNAME              "can0"
#define CONTROL_CAN_ID              (0x120u)
#define IFNAME_LEN                  (4)
#define DEV_ID_MASK                 (0x80u)
#define HOST_ID_MASK                (0xFFu)
#define AXIS_MASK                   (0xFu)
#define MIN_AXIS_IDX                (1)

#define CURRENT_PID                 0
#define PROCESS_PRIORITY            -20  // -20 (highest) to 19 (lowest)
#define NULL_THREAD                 (pthread_t)(0)

double convert_iu_to_m(int32_t iu)
{
    return IU_TO_M * iu;
}

int32_t convert_m_to_iu(double m)
{
    return (int32_t)(M_TO_IU * m);
}

double convert_iu_to_rad(int32_t iu)
{
    return IU_TO_RAD * iu;
}

double convert_iu_to_rad_yaw(int32_t iu)
{
    return IU_TO_RAD_YAW * iu;
}

typedef struct rawData
{
    uint8_t id;
    double data;
    double time;
} rawData_t;

typedef struct rawDataLog
{
    uint32_t		index;
    rawData_t           entry[LOG_BUFLEN];
} rawDataLog_t;


// Define the underlying communication protocol
#define USE_CAN
#ifdef USE_CAN
typedef CAN_MSG msg_t;
#else
typedef struct msg
{
    uint8_t msg[BUFLEN];
    int  size;
} msg_t;
#endif


static eyeData_t eyeData;
static eyeCalData_t eyeCalData;
static rawDataLog_t eyeLogData;

static neckData_t neckData;
static rawDataLog_t neckLogData;

typedef void(*rxCallbackFxn)(uint16_t, uint16_t, int32_t);
void eyeRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data);
void neckRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data);
static double get_timestamp();

typedef struct devHandle
{
    int             fd;
    uint8_t         devId;
    char            ifName[IFNAME_LEN];
    
    int         	runFlag;
    pthread_t       txThreadHandle;
    pthread_t       rxThreadHandle;
    CAN_MSG        	rx_buf;

    uint32_t        ack_pend;

    msg_t		    cmd_buf[MAX_CMD_PEND];
    uint32_t		cmd_consume_idx;
    uint32_t		cmd_produce_idx;
    uint8_t		    host_id;
    uint8_t         group_Id;
    uint8_t         num_axis;

    rxCallbackFxn	callback;

} devHandle_t;

devHandle_t eye  = {.fd = -1, .devId = EYE_DEV_ID, .ifName = EYE_CAN_IFNAME,.txThreadHandle = NULL_THREAD,
                    .rxThreadHandle = NULL_THREAD, .ack_pend = 0, .cmd_consume_idx = 0, 
                    .cmd_produce_idx = 0, .host_id = HOST_ID, .group_Id = EYE_GROUP_ID, .num_axis = NUM_EYE_AXIS, .callback = &eyeRxCallback};

devHandle_t neck = {.fd=-1, .devId = NECK_DEV_ID, .ifName = NECK_CAN_IFNAME, .txThreadHandle = NULL_THREAD,
                    .rxThreadHandle = NULL_THREAD, .ack_pend = 0, .cmd_consume_idx = 0, 
                    .cmd_produce_idx = 0, .host_id = HOST_ID, .group_Id = NECK_GROUP_ID, .num_axis = NUM_NECK_AXIS, .callback = &neckRxCallback};

// Debugging print function of CAN messages
static void DebugPrintCAN(CAN_MSG* frame, bool recv)
{
    if(recv) {
        printf("\n\nCAN MSG RECEIVED\n");
    } else {
        printf("\n\nCAN MSG SENT\n");
    }
    printf("id: %x\n", frame->identifier);
    printf("length: %x\n", frame->length);
    for(int i = 0; i < frame->length; i++) {
        printf("data[%d]: %x\n", i, frame->CAN_data[i]);
    }

    return;
}

static void add_log_data(rawDataLog_t* log, double time, double data, uint8_t id)
{
    log->entry[log->index].data = data;
    log->entry[log->index].time = time;
    log->entry[log->index].id   = id;
    log->index = (++log->index < LOG_BUFLEN)?log->index:0;	 
}

void neckRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data)
{
    neckData.time = get_timestamp();
    double converted_data;
    uint8_t log_data_id;

    switch (reg_addr)
    {
        case REG_APOS & REG_MASK:
            converted_data = convert_iu_to_rad(data);
            break;
        default:
            break;
    }

    switch (axis_id)
    {
        case NECK_YAW_AXIS:
            log_data_id = NECK_YAW; 
            neckData.yaw = convert_iu_to_rad_yaw(data);
            break;
        case NECK_PITCH_AXIS:
            log_data_id = NECK_PITCH;
	    neckData.pitch = converted_data;
            break;
        case NECK_ROLL_AXIS:
            log_data_id = NECK_ROLL;
	    neckData.roll = converted_data;
            break;
        default:
            break;
    }

    add_log_data(&neckLogData, neckData.time, converted_data, log_data_id);
}

void eyeRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data)
{

    double converted_data;
    uint8_t log_data_id;
    double time = get_timestamp();
    switch (reg_addr)
    {
        case REG_APOS & REG_MASK:
            log_data_id = 1;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.pos[axis_id-1] = converted_data;
            break;

        case REG_POSERR & REG_MASK:
            log_data_id = 2;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.err[axis_id-1] = converted_data;
            break;

        case REG_TPOS & REG_MASK:
            log_data_id = 3;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.tpos[axis_id-1] = converted_data;
            break;

        case REG_APOS2 & REG_MASK:
            log_data_id = 0;
            eyeData.time = time;
            converted_data = convert_iu_to_rad(data);

            switch (axis_id)
            {
                case EYE_YAW_LEFT_AXIS:
                    eyeData.yaw[LEFT] = converted_data;
                    break;
                case EYE_YAW_RIGHT_AXIS:
                    eyeData.yaw[RIGHT] = converted_data;
                    break;
                case EYE_PITCH_LEFT_AXIS:
                    eyeData.pitch[LEFT] = converted_data;
                    break;
                case EYE_PITCH_RIGHT_AXIS:
                    eyeData.pitch[RIGHT] = converted_data;
                    break;
                default:
                    break;
            }
            
            break;

        case CAL_RUN_VAR & REG_MASK:
            eyeCalData.complete[axis_id-1] = data;
            break;

	    case CAL_APOS2_OFF_VAR & REG_MASK:
            eyeData.time = time;
            converted_data = convert_iu_to_rad(data);

            switch (axis_id)
            {
                case EYE_YAW_LEFT_AXIS:
                    eyeData.yaw_offset[LEFT] = converted_data;
                    break;
                case EYE_YAW_RIGHT_AXIS:
                    eyeData.yaw_offset[RIGHT] = converted_data;
                    break;
                case EYE_PITCH_LEFT_AXIS:
                    eyeData.pitch_offset[LEFT] = converted_data;
                    break;
                case EYE_PITCH_RIGHT_AXIS:
                    eyeData.pitch_offset[RIGHT] = converted_data;
                    break;
                default:
                    break;
            }
	    break;

        default:
            break;
    }

    add_log_data(&eyeLogData, time, converted_data, (log_data_id*NUM_EYE_AXIS)+axis_id-1);
}

static void parse_response_msg(devHandle_t* dev)
{   
    CAN_MSG *frame = &dev->rx_buf;
    uint32_t result = 0;
    uint8_t axis;
    uint16_t reg_addr;
    
#ifdef DEBUG_CAN
    DebugPrintCAN(frame, true);
#endif

    if(ParseTMLCAN(frame, &result, &axis, &reg_addr) < 0) {
        printf("Failed to parse TML CAN message. Ignored\n");
        return;
    }

    if(dev->callback != NULL)
        dev->callback(axis, reg_addr, result);
}

static uint64_t timebase;

static double get_timestamp()
{
    struct timespec currTime = {0,0};
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    uint64_t t = currTime.tv_sec*NANO_TO_SECS + currTime.tv_nsec;
    t = t - timebase;
    
    return (double)t/(double)NANO_TO_SECS;
}

void* ThreadRXFunc(void* input)
{
    devHandle_t* dev = (devHandle_t*)input;
    struct sched_param param = {RX_THREAD_PRIORITY};
    int err = 0;

    if(sched_setscheduler(CURRENT_TID, SCHED_RR, &param)) {
        err = errno;
        printf("Failed to set RX Thread priority with error %d\n", err);
    }

    while(dev->runFlag) {
        memset(&(dev->rx_buf),0, BUFLEN);
        if(ReceiveMessage(&(dev->rx_buf), dev->fd) < 0) {
            printf("Failed to properly rx CAN message\n");
            break;
        } else {
            // Parse the msg for relevant data
            parse_response_msg(dev);
        }
    }

    return NULL;
}

void* ThreadTXFunc(void* input) 
{
    devHandle_t* dev = (devHandle_t*)input;
    unsigned int index;
    struct sched_param param = {TX_THREAD_PRIORITY};
    int err = 0;

    if(sched_setscheduler(CURRENT_TID, SCHED_RR, &param)) {
        err = errno;
        printf("Failed to set tx thread priority with error %d\n", err);
    }

    // Loop while dev is running or cmds in buffer
    while(dev->runFlag || (dev->cmd_produce_idx - dev->cmd_consume_idx) != 0) {  
        if((dev->cmd_produce_idx - dev->cmd_consume_idx) != 0) {  
            // check if cmds are still remaining in buf or not too many acks still pending
            index = dev->cmd_consume_idx % MAX_CMD_PEND;
#ifdef DEBUG_CAN
            DebugPrintCAN(&(dev->cmd_buf[index]), false);
#endif
            if (SendMessage(&dev->cmd_buf[index], dev->fd) < 0) {
                printf("Failed to send message from send buf\n");
                break;
            }

            dev->cmd_consume_idx++;
            dev->ack_pend++;
        }
        else {
            //yield, waiting for more messages to consume
            //SwitchToThread();
            //usleep(TX_THREAD_SLEEP); // We will try sleeping it first
            sched_yield();      // a little different than SwitchToThread
        }
    }

    return NULL;
}

// Initialize the device
static int8_t ConnectDev(devHandle_t* dev)
{
    CAN_MSG msg;

    // Remove any existing frames from recv buffer
    FlushCanSock(dev->fd);

    // Perhaps we can check a register for all axes to see if good to go
    // Or can ping all the drives to make sure all are up
    FormatPing(&msg, dev->group_Id);
#ifdef DEBUG_CAN
    DebugPrintCAN(&msg, false);
#endif
    if(SendMessage(&msg, dev->fd) < 0) {
        printf("Failed to broadcast ping message\n");
        return -1;
    }
    
    // Receive pong responses from all axes within the group
    bool* recvMsg = (bool*)malloc(dev->num_axis);
    for(int i = 0; i < dev->num_axis; i++) {
        uint8_t axis = 0;
        char version[VERSION_SIZE];
        memset(&msg, 0, sizeof(CAN_MSG));
        if(ReceiveMessage(&msg, dev->fd) < 0) {
            printf("Failed to properly rcv pong response\n");
            return -1;
        }
#ifdef DEBUG_CAN
        DebugPrintCAN(&msg, true);
#endif
        if(ParsePong(&msg, &axis, version)) {
            printf("Failed to parse pong response\n");
            return -1;
        }

        printf("Received pong: id=%d firmware=%s", axis, version);
        
        // Take 4LSB to ensure expected axis id's
        // Should be between 1 and 4
        uint8_t axisIdx = axis & AXIS_MASK;
        if((!recvMsg[axisIdx-1])) {
            printf("Received multiple pong replies from same axis id\n");
            return -1;
        } else if((axisIdx < MIN_AXIS_IDX) || (axisIdx > dev->num_axis) || !(axis & dev->devId)) {
            printf("Received unexpected axis id response\n");
            return -1;
        } else {
            recvMsg[axisIdx-1] = true;
        }
    }

    return 0;
}

// Might not be needed!
static uint16_t DisconnectDev(devHandle_t* dev)
{
    return 0;
}

static void ShutdownDev(devHandle_t* dev)
{
    uint16_t runState = dev->runFlag;
    int err = 0;

    // Stop the processing threads
    dev->runFlag = 0;

    if(dev->rxThreadHandle != NULL_THREAD) {
        pthread_cancel(dev->rxThreadHandle);
        if(pthread_join(dev->rxThreadHandle, NULL) != 0) {
            err = errno;
            printf("pthread_join for rx failed with errno=%d\n", err);
        }
        dev->rxThreadHandle = NULL_THREAD;
    }

    if(dev->txThreadHandle != NULL_THREAD) {
        pthread_cancel(dev->txThreadHandle);
        if(pthread_join(dev->txThreadHandle, NULL) != 0) {
            err = errno;
            printf("pthread_join for tx failed with errno=%d\n", err);
        }
        dev->txThreadHandle = NULL_THREAD;
    }

    if(dev->fd > -1) {
        if(runState) {
            printf("Processing threads complete, disconnecting\n");
            DisconnectDev(dev);
        }
        printf("Closing Socket\n");
        CleanCanSock(dev->fd);
        dev->fd = -1;
    }    

    dev->cmd_consume_idx = 0;
    dev->cmd_produce_idx = 0;
}

static void StartDev(devHandle_t* dev)
{
    // In case we are currently running call shutdown first;
    ShutdownDev(dev);
    
    //Create a socket
    dev->fd = InitCanSock(dev->ifName, RX_TIMEOUT_MS);
    if(dev->fd < 0) {
        printf("Failed to init CAN on %s interface\n", dev->ifName);
        return;
    }

    // Add suitable filters - make sure destined to host, ensure sent by eye or neck only
    AddFilter(dev->fd, FORMAT_MASK_DEST(dev->host_id), FILTER_ID_DEST(HOST_ID_MASK));
    AddFilter(dev->fd, FORMAT_MASK_SOURCE(dev->devId), FILTER_ID_SOURCE(DEV_ID_MASK));    

    printf("CAN configured.\n");
    uint16_t conn_count = 0;
    while (conn_count < 5 && ConnectDev(dev) != 0) {
        conn_count++;
    }
    
    // Setup the connection to the technosoft drive
    if(conn_count < 5) {
        // If we can talk with the drive, start the processing threads
        dev->runFlag=1;
        if(pthread_create(&dev->rxThreadHandle, NULL, ThreadRXFunc, dev) != 0) {
            printf("Failed to start dev rx thread\n");
            return;
        }
        if(pthread_create(&dev->txThreadHandle, NULL, ThreadTXFunc, dev)) {
            printf("Failed to start dev tx thread\n");
            return;
        }
    }
    else {
        printf("Failed to connect to drive\n");
    }
}

// Return slot in the tx buffer
static msg_t* getMsgSlot(devHandle_t* dev)
{
    double base_time = get_timestamp();
    double cur_time;
    while (dev->cmd_produce_idx - dev->cmd_consume_idx == MAX_CMD_PEND) {
        cur_time = get_timestamp();
 
        if(cur_time - base_time > SEND_TIMEOUT_S || dev->runFlag == 0)
        { 
            printf("send message failed to get slot\n");
            return NULL;
        }

        // buffer is full, yield
        //SwitchToThread();
        sched_yield();
        
    }

    msg_t* msg = &dev->cmd_buf[dev->cmd_produce_idx % MAX_CMD_PEND];
    memset(msg, 0, BUFLEN);
    return msg;
}

// Start the library
void __attribute__ ((constructor)) Start(void)
{
    int err; 
    struct timespec currTime = {0,0};
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    timebase = currTime.tv_sec*NANO_TO_SECS + currTime.tv_nsec;

    // Elevate priority of current process
    if(setpriority(PRIO_PROCESS, CURRENT_PID, PROCESS_PRIORITY)) {
        err = errno;
        printf("setpriority failed with errno=%d\n", err);
    }

#ifdef DEBUG_CAN
    printf("DEBUG_CAN turned on\n");
#endif

    SetHostId(HOST_ID);

    ResetEyePollData();
    StartDev(&eye);

    ResetNeckPollData();
    StartDev(&neck);

    printf("Initialised.\n");
}

void __attribute__ ((destructor)) Cleanup(void)
{
    ShutdownDev(&eye);
    ShutdownDev(&neck);
}

eyeData_t* GetEyeData(void)
{
    return &eyeData;
}

eyeCalData_t* GetEyeCalData(void)
{
    return &eyeCalData;
}

neckData_t* GetNeckData(void)
{
    return &neckData;
}

#define MAX_FIXED_POINT 32767.999969
#define MIN_FIXED_POINT -32767.999969

static uint32_t GetFixedPoint(double value)
{
    if(value >= MAX_FIXED_POINT  || value <= MIN_FIXED_POINT)
    {
        return 0;
    }
    
    int16_t whole = (int16_t)floor(value);
    double fractional = value - whole;
    uint16_t decimal = fractional*0xFFFF;
    uint32_t result = ((uint32_t)decimal << 16) | (whole&0xFFFF);
    return result;
}

static uint32_t GetLong(int32_t value)
{
    uint32_t result = ((uint32_t)value << 16) | ((value>>16)&0xFFFF);
    return result;
}

// New send message function which uses the helper functions provided
static void SendMotorCommand(devHandle_t* dev, msgType_t msgType, uint8_t id, uint8_t group)
{
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;

    switch(msgType) {
        case CMD_STA:
            FormatSTA(msgSlot, id);
            break;
        case CMD_AXISON:
            FormatSetAxisControl(msgSlot, id, 1);
            break;
        case CMD_AXISOFF:
            FormatSetAxisControl(msgSlot, id, 0);
            break;
        case CMD_UPDATEMODE_0:
            FormatTUM(msgSlot, id, 0);
            break;
        case CMD_UPDATEMODE_1:
            FormatTUM(msgSlot, id, 1);
            break;
        case CMD_MOTIONMODE_PP:
            FormatSetModePP(msgSlot, id);
            break;
        case CMD_MOTIONMODE_PP1:
            FormatSetModePP1(msgSlot, id);
            break;
        case CMD_MOTIONMODE_PP3:
            FormatSetModePP3(msgSlot, id);
            break;
        case CMD_SET_POSABS:
            FormatSetPosRef(msgSlot, id, 0);
            break;
        case CMD_SET_POSREL:
            FormatSetPosRef(msgSlot, id, 1);
            break;
        default:
            printf("Request unknown message type for build\n");
            return;
    }

    dev->cmd_produce_idx++;
}

// Function to get data from motor memory
static void GetDataMem(devHandle_t* dev, msgType_t msgType, uint8_t id, uint16_t regAddr)
{
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;
    
    switch(msgType) {
        case CMD_GETDATA_16:
            FormatGiveMeData(msgSlot, id, regAddr, 1);
            break;
        case CMD_GETDATA_32:
            FormatGiveMeData(msgSlot, id, regAddr, 0);
            break;
        case CMD_GETDATA2_16:
            FormatGiveMeData2(msgSlot, id, regAddr, 1);
            break;
        case CMD_GETDATA2_32:
            FormatGiveMeData2(msgSlot, id, regAddr, 0);
            break;
        default:
            return;
    }

    dev->cmd_produce_idx++;
}

// Function to set data
static void SetDataMem(devHandle_t* dev, msgType_t msgType, uint8_t id, uint16_t regAddr, uint32_t data)
{
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;
    
    switch(msgType) {
        case CMD_SETDATA_16:
            FormatSetVal16(msgSlot, id, regAddr, data);
            break;
        case CMD_SETDATA_32:
            FormatSetVal16(msgSlot, id, regAddr, data);
            break;
        default:
            return;
    }

    dev->cmd_produce_idx++;
}

// Function wrapper to goto instruction in TML program 
static void GoToInstr(devHandle_t* dev, uint8_t id, uint8_t isGroupId, uint16_t instrAddr)
{
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;
    
    FormatGoTo(msgSlot, id, instrAddr, isGroupId);
}

// Wrapper functions for communication with motors
void SendMotorCommandEye(msgType_t msgType, uint8_t id, uint8_t group)
{
    SendMotorCommand(&eye, msgType, id, group);
}

void SendMotorCommandNeck(msgType_t msgType, uint8_t id, uint8_t group)
{
    SendMotorCommand(&neck, msgType, id, group);
}

void GetDataMemEye(msgType_t msgType, uint8_t id, uint16_t regAddr)
{
    GetDataMem(&eye, msgType, id, regAddr);
}

void GetDataMemNeck(msgType_t msgType, uint8_t id, uint16_t regAddr)
{
    GetDataMem(&neck, msgType, id, regAddr);
}

void SetDataMemEye(msgType_t msgType, uint8_t axis_id, uint16_t regAddr, uint32_t data)
{
    SetDataMem(&eye, msgType, axis_id, regAddr, data);
}

void SetDataMemNeck(msgType_t msgType, uint8_t axis_id, uint16_t regAddr, uint32_t data)
{
    SetDataMem(&neck, msgType, axis_id, regAddr, data);
}

// Functions to set operating mode of the drives
// Set eye controller to poll for new positions
void SetEyePollData(void)
{
    GoToInstr(&eye, eye.group_Id, 1, MOTION_LOOP_IP);   
    return;
}

// Set eye controller to busy wait
void ResetEyePollData(void)
{
    GoToInstr(&eye, eye.group_Id, 1, WAIT_LOOP_IP);
    return;
}

// Set neck controller to poll for new positions
void SetNeckPollData(void)
{
    GoToInstr(&eye, eye.group_Id, 1, MOTION_LOOP_NECK_IP);
    return;
}

// Set neck controller to busy wait
void ResetNeckPollData(void)
{
    GoToInstr(&neck, neck.group_Id, 1, WAIT_LOOP_NECK_IP);
    return;
}

// Functions to set pos, speed, accel of controllers
void SetEyePosn(uint8_t axis, double pos_m)
{
    uint32_t data = GetLong(pos_m*M_TO_IU);
    SetDataMemEye(CMD_SETDATA_32, axis, REG_CPOS, data);
}

void SetNeckPosn(uint8_t axis, double pos_rad)
{
    uint32_t data;
    if (axis == NECK_YAW_AXIS)
        data = GetLong(pos_rad*RAD_TO_IU_YAW);
    else
        data = GetLong(pos_rad*RAD_TO_IU);
    
    SetDataMemNeck(CMD_SETDATA_32, axis, REG_CPOS, data);
}

void SetNeckSpeed(uint8_t axis, double speed_rps)
{
    uint32_t data;
    
    if(axis == NECK_YAW_AXIS)
        data = GetFixedPoint(speed_rps*RADPS_TO_SPEED_IU_YAW);
    else
        data = GetFixedPoint(speed_rps*RADPS_TO_SPEED_IU);
    
    SetDataMemNeck(CMD_SETDATA_32, axis, REG_CSPD, data);
}

void SetEyeSpeed(uint8_t axis, double speed_mps)
{
    uint32_t data = GetFixedPoint(speed_mps*MPS_TO_SPEED_IU);
    SetDataMemEye(CMD_SETDATA_32, axis, REG_CSPD, data);
}

void SetNeckAccel(uint8_t axis, double accel_rpss)
{
    uint32_t data;
    
    if(axis == NECK_YAW_AXIS)
        data = GetFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU_YAW);
    else
        data = GetFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU);
    
    SetDataMemNeck(CMD_SETDATA_32, axis, REG_CACC, data);
}

void SetEyeAccel(uint8_t axis, double accel_mpss)
{
    uint32_t data = GetFixedPoint(accel_mpss*MPSS_TO_ACCEL_IU);
    SetDataMemEye(CMD_SETDATA_32, axis, REG_CACC, data);
}

// Start calibration routine of eyes
void StartEyeCal(uint8_t axis, double pos_m)
{
    if(axis <=0 || axis > NUM_EYE_AXIS) {
        return;
    }

    SetEyePosn(axis, pos_m);
    eyeCalData.complete[axis-1] = CAL_RUNNING;

    if(pos_m > 0) {
        GoToInstr(&eye, axis, 0, FOR_CAL_IP);
    }
    else {
        GoToInstr(&eye, axis, 0, REV_CAL_IP);
    }
}