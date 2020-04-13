// _DEFAULT_SOURCE defined for ifreq struct
#define _DEFAULT_SOURCE
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include "libmacaque_RS232/macaque_linux.h"
#include "sock_interface/sock.h"
#include "TML_RS232_lib/include/TML_RS232_lib.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <time.h>
#include <semaphore.h>

#include <math.h>
#include <sched.h>
#include <pthread.h>

#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <errno.h>
#include <sys/resource.h>

#ifndef M_PI
    #define M_PI                        ((double)3.14159265358979323846)
#endif
#define COUNTS_PER_POLE	            (double)(2048)
#define METERS_PER_POLE             (double)(0.018)
#define PEAK_CURR_EYE               (double)(1.6)   // amps
#define MOTOR_CONST_EYE             (double)(6.43)  // N/A
#define SAMPLE_PERIOD_EYE_S	        (double)(0.001)
#define IU_TO_M                     (double)(METERS_PER_POLE/COUNTS_PER_POLE)
#define M_TO_IU                     (double)(COUNTS_PER_POLE/METERS_PER_POLE)
#define MPS_TO_SPEED_IU		        (double)(SAMPLE_PERIOD_EYE_S * M_TO_IU)
#define SPEED_IU_TO_MPS		        (double)(1.0 / MPS_TO_SPEED_IU)
#define MPSS_TO_ACCEL_IU		    (double)(SAMPLE_PERIOD_EYE_S * SAMPLE_PERIOD_EYE_S * M_TO_IU)
#define ACCEL_IU_TO_MPSS		    (double)(1.0 / MPSS_TO_ACCEL_IU)
#define CURR_IU_TO_AMPS_EYE         (double)((2.0*PEAK_CURR_EYE)/6552.0)
#define AMPS_TO_CURR_IU_EYE         (double)(1.0/CURR_IU_TO_AMPS_EYE)
#define IU_TO_FORCE_EYE(iu)         (double)(iu*MOTOR_CONST_EYE*CURR_IU_TO_AMPS_EYE)
#define FORCE_TO_IU_EYE(f)          (int16_t)(f*(1.0/MOTOR_CONST_EYE)*AMPS_TO_CURR_IU_EYE)

#define COUNTS_PER_REV 	            (double)(5000)
#define COUNTS_PER_REV_YAW 	        (double)(16*86)
#define SAMPLE_PERIOD_NECK_S	    (double)(0.001)
#define IU_TO_RAD_YAW               (double)(M_PI/(2*COUNTS_PER_REV_YAW))
#define RAD_TO_IU_YAW               (double)(1/IU_TO_RAD_YAW)
#define IU_TO_RAD                   (double)(M_PI/(2*COUNTS_PER_REV))
#define RAD_TO_IU                   (double)(1/IU_TO_RAD)
#define RADPSS_TO_ACCEL_IU		    (double)(SAMPLE_PERIOD_NECK_S * SAMPLE_PERIOD_NECK_S * RAD_TO_IU)
#define RADPSS_TO_ACCEL_IU_YAW	    (double)(SAMPLE_PERIOD_NECK_S * SAMPLE_PERIOD_NECK_S * RAD_TO_IU_YAW)
#define ACCEL_IU_TO_RADPSS		    (double)(1.0 / RADPSS_TO_ACCEL_IU)
#define ACCEL_IU_TO_RADPSS_YAW      (double)(1.0 / RADPSS_TO_ACCEL_IU_YAW)
#define RADPS_TO_SPEED_IU		    (double)(SAMPLE_PERIOD_NECK_S * RAD_TO_IU)
#define RADPS_TO_SPEED_IU_YAW       (double)(SAMPLE_PERIOD_NECK_S * RAD_TO_IU_YAW)
#define SPEED_IU_TO_RADPS		    (double)(1.0 / RADPS_TO_SPEED_IU)
#define SPEED_IU_TO_RADPS_YAW       (double)(1.0 / RADPS_TO_SPEED_IU_YAW)

#define PEAK_CURR_NECK              (double)(0.75)  // amps
#define AMPS_TO_CURR_IU_NECK        (double)(65472.0/(2.0*PEAK_CURR_NECK))
#define CURR_IU_TO_AMPS_NECK        (double)(1.0/AMPS_TO_CURR_IU_NECK)
#define MOTOR_CONST_NECK            (double)(1.175) // Nm/A
#define MOTOR_CONST_NECK_YAW        (double)(0.0261) // Nm/A
#define TORQUE_TO_IU_NECK(t)        (int16_t)(t*(1.0/MOTOR_CONST_NECK)*AMPS_TO_CURR_IU_NECK)
#define TORQUE_TO_IU_NECK_YAW(t)    (int16_t)(t*(1.0/MOTOR_CONST_NECK_YAW)*AMPS_TO_CURR_IU_NECK)
#define IU_TO_TORQUE_NECK(iu)       (double)(iu*MOTOR_CONST_NECK*CURR_IU_TO_AMPS_NECK)
#define IU_TO_TORQUE_NECK_YAW(iu)   (double)(iu*MOTOR_CONST_NECK*CURR_IU_TO_AMPS_NECK)


#define SECS_TO_NANO                (uint64_t)(1000*1000*1000)
#define CURRENT_TID                 (0)
#define RX_THREAD_PRIORITY          (10)
#define TX_THREAD_PRIORITY          (31)
#define TX_THREAD_SLEEP             (100*1000) // us 

#define CURRENT_PID                 (0)
#define PROCESS_PRIORITY            (-10)  // -20 (highest) to 19 (lowest)

#define DBG_BUFF_SIZE               (255)
#define SEM_NOTSHARED               (0)
#define SYNC_TRIES                  (8)

#ifdef STARTUP_TEST
#define DEBUG_STARTUP
#endif
#define DEBUG_SHUTDOWN
//#define DEBUG_SYNC
//#define DEBUG_TX
//#define DEBUG_RX
#define ENABLE_LOG

#if defined DEBUG_RX || defined DEBUG_TX 
// convert RS232 to string
static char* get_msg_str(msg_t* frame, char* buff)
{
    snprintf(buff, DBG_BUFF_SIZE, "0x");
    char* ptr = buff+2;
    for(int i = 0; i < frame->length; i++) {
        snprintf(ptr, DBG_BUFF_SIZE, " %02x", frame->RS232_data[i]);
        ptr+=sizeof(frame->RS232_data[i]) + 2;
    }

    return buff;
}
#endif

typedef struct rawData
{
    uint8_t id;
    double data;
    double time;
} rawData_t;

typedef struct rawDataLog
{
    uint32_t		index;
    rawData_t       entry[LOG_BUFLEN];
    char            file[LOG_FILELEN];
} rawDataLog_t;

static eyeData_t eyeData;
static eyeCalData_t eyeCalData;
static rawDataLog_t* eyeLogData = NULL;

static neckData_t neckData;
static rawDataLog_t* neckLogData = NULL;

typedef void(*rxCallbackFxn)(uint16_t, uint16_t, int32_t);
void EyeRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data);
void NeckRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data);
static double get_timestamp();

typedef struct devHandle
{
    int                 fd;
    const char* const  	local_ip;
    const uint16_t     	local_port;
    const char* const  	dest_ip;
    uint16_t            dest_conn_port;
    uint16_t            dest_cmd_port;

    int         	    runFlag;
    pthread_t      	    txThreadHandle;
    bool                txThreadValid;
    pthread_t      	    rxThreadHandle;
    bool                rxThreadValid;
    
    msg_t        	    rx_buf;
    uint32_t         	ack_pend;
    msg_t		        cmd_buf[MAX_CMD_PEND];
    uint32_t		    cmd_consume_idx;
    uint32_t		    cmd_produce_idx;
    sem_t               buf_empty;
    sem_t               buf_full;

    uint8_t		        host_id;

    rxCallbackFxn	    callback;
    pthread_cond_t      sync_cond;
    pthread_mutex_t     sync_mutex;

} devHandle_t;

static devHandle_t eye  = {.fd = -1, .local_ip = LOCAL_IP, .local_port = EYE_LOCAL_PORT, .dest_ip = EYE_IP,
                    .dest_conn_port = CONN_PORT, .dest_cmd_port = CMD_PORT,
                    .ack_pend = 0, .cmd_consume_idx = 0, .cmd_produce_idx = 0,
		            .host_id = HOST_ID, .callback = &EyeRxCallback, 
                    .sync_cond = PTHREAD_COND_INITIALIZER, .sync_mutex = PTHREAD_MUTEX_INITIALIZER};

static devHandle_t neck = {.fd = -1, .local_ip = LOCAL_IP, .local_port = NECK_LOCAL_PORT, .dest_ip = NECK_IP,
                    .dest_conn_port = CONN_PORT, .dest_cmd_port = CMD_PORT,
                    .ack_pend = 0, .cmd_consume_idx = 0, .cmd_produce_idx = 0,
		            .host_id = HOST_ID, .callback = &NeckRxCallback, 
                    .sync_cond = PTHREAD_COND_INITIALIZER, .sync_mutex = PTHREAD_MUTEX_INITIALIZER};

void* ThreadLogFn(void* input)
{
    rawDataLog_t* log = (rawDataLog_t*)input;
    FILE* file = NULL;

    file = fopen(log->file, "a");
    if(file == NULL) {
        printf("Failed to open %s to log data\n", log->file);
        free(log);
        pthread_exit(NULL);
    }

    // Beginning of file
    if(ftell(file) == (long)0) {
        char str[BUFLEN]="";
        snprintf(str, BUFLEN, "Time,Log ID,Data\n");
        if(fwrite(str, BUFLEN, 1, file) != 1) {
            printf("Failed to write header to file %s\n", log->file);
            fclose(file);
            free(log);
            pthread_exit(NULL);
        }
    }
    
    for(int i = 0; i < log->index; i++) {
        char str[BUFLEN]="";
        snprintf(str, DBG_BUFF_SIZE, " %.4f,%d,%.4f\n", log->entry[i].time, log->entry[i].id, log->entry[i].data);
        if(fwrite(str, BUFLEN, 1, file) != 1) {
            printf("Failed to write log entry %d to file %s\n", i, log->file);
            fclose(file);
            free(log);
            pthread_exit(NULL);
        }
    }

    fclose(file);
    free(log);
    pthread_exit(NULL);
}

// Write out log buffers to file
void FlushLogs()
{
        int err;
        pthread_t neckLogHandle, eyeLogHandle;
        if(pthread_create(&neckLogHandle, NULL, ThreadLogFn, neckLogData) != 0) {
            err = errno;
            printf("Failed to start neck thread logging function with errno=%d\n", err);
            neckLogData = NULL;
            return;
        }
        if(pthread_join(neckLogHandle, NULL) != 0) {
            err = errno;
            printf("Failed to join neck logging thread with errno=%d\n", err);
        }
        neckLogData = NULL;

        if(pthread_create(&eyeLogHandle, NULL, ThreadLogFn, eyeLogData) != 0) {
            printf("Failed to start eye thread logging function with errno=%d\n", err);
            eyeLogData = NULL;
            return;
        }
        if(pthread_join(eyeLogHandle, NULL) != 0) {
            err = errno;
            printf("Failed to join eye logging thread with errno=%d\n", err);
            return;
        }
        eyeLogData = NULL;
}

static void add_log_data(rawDataLog_t* log, double time, double data, uint8_t id)
{
    log->entry[log->index].data = data;
    log->entry[log->index].time = time;
    log->entry[log->index].id   = id;
    log->index++;
}

void NeckRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data)
{
    double time = get_timestamp();
    double converted_data;
    uint8_t log_data_type;
    int err;
    
    // Neck axis id 1-3
    if(axis_id > NUM_NECK_AXIS) {
        printf("Neck data recvd from invalid axis id %d", axis_id);
        return;
    }
    neckData.time = time;
    switch (reg_addr) {
        case REG_APOS:
            switch (axis_id) {
                case NECK_YAW_AXIS:
                    converted_data = IU_TO_RAD_YAW*data;
                    break;
                case NECK_PITCH_AXIS:
                case NECK_ROLL_AXIS:
                    converted_data = IU_TO_RAD*data;
                    break;
                default:
                    printf("Unknown neck axis in APOS msg (axis=%d)\n", axis_id);
                    return;
            }
            pthread_mutex_lock(&neckData.pos[AXISID_TO_DATAIDX(axis_id)].mutex);
            neckData.pos[AXISID_TO_DATAIDX(axis_id)].pos = converted_data;
            neckData.pos[AXISID_TO_DATAIDX(axis_id)].time = time;
            pthread_mutex_unlock(&neckData.pos[AXISID_TO_DATAIDX(axis_id)].mutex);
            log_data_type = NECK_POS;
            break;

        case REG_IQ:
            switch(axis_id) {
                case NECK_YAW_AXIS:
                    converted_data = IU_TO_TORQUE_NECK_YAW(data);
                    break;
                case NECK_PITCH_AXIS:
                case NECK_ROLL_AXIS:
                    converted_data = IU_TO_TORQUE_NECK(data);
                    break;
                default:
                    printf("Unknown neck axis in IQ msg (axis=%x)\n", axis_id);
                    return;
            }
            neckData.torque[AXISID_TO_DATAIDX(axis_id)] = converted_data;
            log_data_type = NECK_TORQUE;
            break;

        case VAR_CAL_READY:
            neckData.ready[AXISID_TO_DATAIDX(axis_id)] = data;
            // no logging needed return
            return;
        default:
            printf("Unknown neck data returned (reg=%x)\n", reg_addr);
            return;
    }

    if(neckLogData == NULL) {
        neckLogData = (rawDataLog_t*)malloc(sizeof(rawDataLog_t));
        strcpy(neckLogData->file, NECK_LOG_FILENAME);
        neckLogData->index = 0;
    }
    add_log_data(neckLogData, neckData.time, converted_data, log_data_type*NUM_NECK_AXIS+axis_id-1);
    if(neckLogData->index == LOG_BUFLEN) {
        pthread_t logHandle;
        if(pthread_create(&logHandle, NULL, ThreadLogFn, neckLogData) != 0) {
            err = errno;
            printf("Failed to start neck thread logging function with errno=%d\n", err);
        }
        neckLogData = NULL;
    }

}

void EyeRxCallback(uint16_t axis_id, uint16_t reg_addr, int32_t data)
{
    double converted_data;
    uint8_t log_data_type;
    double time = get_timestamp();
    int err;
    
    // Eye axis ids 1-4
    if(axis_id > NUM_EYE_AXIS) {
        printf("Eye data recvd from invalid axis id %d", axis_id);
        return;
    }

    switch (reg_addr) {
        case REG_APOS:
            log_data_type = EYE_HALL;
            converted_data = IU_TO_M*data;
            eyeCalData.time = time;
            eyeCalData.pos[AXISID_TO_DATAIDX(axis_id)] = converted_data;
            break;

        case REG_POSERR:
            log_data_type = EYE_POSERR;
            converted_data = IU_TO_M*data;
            eyeCalData.time = time;
            eyeCalData.err[AXISID_TO_DATAIDX(axis_id)] = converted_data;
            break;

        case REG_TPOS:
            log_data_type = EYE_TARGET;
            converted_data = IU_TO_M*data;
            eyeCalData.time = time;
            eyeCalData.tpos[AXISID_TO_DATAIDX(axis_id)] = converted_data;
            break;

        case REG_APOS2:
            log_data_type = EYE_ENCODER;
            eyeData.time = time;
            converted_data = IU_TO_RAD*data;
             pthread_mutex_lock(&eyeData.pos[AXISID_TO_DATAIDX(axis_id)].mutex);
            eyeData.pos[AXISID_TO_DATAIDX(axis_id)].pos = converted_data;
            eyeData.pos[AXISID_TO_DATAIDX(axis_id)].time = time;
            pthread_mutex_unlock(&eyeData.pos[AXISID_TO_DATAIDX(axis_id)].mutex);
            break;
        
        case REG_IQ:
            log_data_type = EYE_CURRENT;
            eyeData.time = time;
            converted_data = IU_TO_FORCE_EYE(data);
            eyeData.torque[AXISID_TO_DATAIDX(axis_id)] = converted_data;
            break;

        case VAR_CAL_RUN:
            eyeCalData.complete[AXISID_TO_DATAIDX(axis_id)] = data;
            eyeCalData.time = time;
            // No logging, return instead
            return;

	    case VAR_CAL_APOS2_OFF:
            eyeCalData.time = time;
            converted_data = IU_TO_RAD*data;
            eyeCalData.offset[AXISID_TO_DATAIDX(axis_id)] = converted_data;
            // No logging, return instead
	        return;

        default:
            printf("Unknown data returned (reg=0x%x)\n", reg_addr);
            // No logging, return instead
            return;
    }
    
    if(eyeLogData == NULL) {
        eyeLogData = (rawDataLog_t*)malloc(sizeof(rawDataLog_t));
        strcpy(eyeLogData->file, EYE_LOG_FILENAME);
        eyeLogData->index = 0;
    }
    add_log_data(eyeLogData, time, converted_data, (log_data_type*NUM_EYE_AXIS)+axis_id-1);
    if(eyeLogData->index == LOG_BUFLEN) {
        pthread_t logHandle;
        if(pthread_create(&logHandle, NULL, ThreadLogFn, eyeLogData) != 0) {
            err = errno;
            printf("Failed to start eye thread logging function with errno=%d\n", err);
        }
        eyeLogData = NULL;
    }
}

static void parse_response_msg(devHandle_t* dev)
{   
    msg_t *msg = &dev->rx_buf;
    uint32_t result = 0;
    uint8_t axis;
    uint16_t reg_addr;

    if(msg == NULL) {  
        return;
    }
    
    #ifdef DEBUG_RX
        char frame_str[DBG_BUFF_SIZE];
        printf("recv: %s\n", get_msg_str(msg, frame_str));
    #endif

    if(ParseResponse(msg, &result, &axis, &reg_addr)) {
        return;
    }

    if(dev->callback != NULL) {
        dev->callback(axis, reg_addr, result);
    }
}

static uint64_t timebase;

static double get_timestamp()
{
    struct timespec currTime = {0,0};
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    uint64_t t = (uint64_t)currTime.tv_sec*SECS_TO_NANO + currTime.tv_nsec;
    t = t - timebase;

    return (double)t/SECS_TO_NANO;
}

void* ThreadRxFunc(void* input)
{
    devHandle_t* dev = (devHandle_t*)input;
    int rx_bytes;
    struct sched_param param = {RX_THREAD_PRIORITY};
    int err = 0;

    if(sched_setscheduler(CURRENT_TID, SCHED_RR, &param) != 0) {
        err = errno;
        printf("Failed to set RX Thread priority with error %d\n", err);
        printf("Exiting rx thread for ip=%s\n", dev->dest_ip);
        pthread_exit(NULL);
    }
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    while(dev->runFlag) {
        memset(&dev->rx_buf, 0, sizeof(msg_t));
        rx_bytes = ReceiveMessage(&dev->rx_buf, dev->fd);
        if(rx_bytes < 0) {
            printf("Failed to properly rx message\n");
            break;
        } else if (rx_bytes > 0) {
            // Sort by msg type
            if(rx_bytes == CONN_SYNC_RESP_BYTES && dev->rx_buf.RS232_data[CONN_SYNC_RESP_BYTES-1] == CONN_SYNC_RESP) {
                // Receive sync msg response
                pthread_mutex_lock(&dev->sync_mutex);
                dev->ack_pend = 0;
                pthread_cond_signal(&dev->sync_cond);
#ifdef DEBUG_SYNC
                printf("Sync response recvd\n");
#endif
                pthread_mutex_unlock(&dev->sync_mutex);
            } else if(dev->rx_buf.RS232_data[0] == MSG_ACK) {
                // Check if ack message has been received (empty message)
#ifdef DEBUG_RX
                printf("Msg ack recvd\n");
#endif
                pthread_mutex_lock(&dev->sync_mutex);
                if(dev->ack_pend > 0) {
                    dev->ack_pend--;
                }                
                pthread_mutex_unlock(&dev->sync_mutex);

                // If data requested, resp is appended to ack
                if(dev->rx_buf.length > sizeof(MSG_ACK)) {
                    memmove(&(dev->rx_buf.RS232_data[0]), &(dev->rx_buf.RS232_data[1]), dev->rx_buf.length-1);
                    parse_response_msg(dev);
                }
            } else {
#ifdef DEBUG_RX
                printf("Recv data msg from motors\n");
#endif
                // Parse the msg for relevant data
                parse_response_msg(dev);
            }
        }
    }

    printf("Exiting rx thread for ip=%s\n", dev->dest_ip);
    pthread_exit(NULL);
}

// Assumes sync_mutex has been acquired
static int8_t SyncThreads(devHandle_t* dev)
{
#ifdef DEBUG_SYNC
    printf("Sending sync msg....");
#endif
    msg_t msg;
    msg.length = CONN_SYNC_BYTES;
    memset(msg.RS232_data, CONN_SYNC, msg.length);
    if(SendMessage(&msg, dev->fd) < 0) {
        printf("Failed to send sync msg\n");
        return -1;
    }
#ifdef DEBUG_SYNC
    printf("sent\n");
#endif

    // Wait for rx thread to get the response
#ifdef DEBUG_SYNC
    printf("Waiting for sync response....");
#endif
    pthread_mutex_lock(&dev->sync_mutex);
    uint8_t tries = 0;
    while(dev->ack_pend != 0 && tries < SYNC_TRIES) {
        pthread_cond_wait(&dev->sync_cond, &dev->sync_mutex);
#ifdef DEBUG_SYNC
        printf("SyncThreads signal received (ack_pend=%u)\n", dev->ack_pend);
        tries++;
#endif
    }
    pthread_mutex_unlock(&dev->sync_mutex);
#ifdef DEBUG_SYNC
    printf("recvd\n\n");
#endif

    return 0;
}

void* ThreadTxFunc(void* input)
{
    devHandle_t* dev = (devHandle_t*)input;
    struct sched_param param = {TX_THREAD_PRIORITY};
    int err = 0;
    bool needSync = false;

    if(sched_setscheduler(CURRENT_TID, SCHED_RR, &param) != 0) {
        err = errno;
        printf("Failed to set TX Thread priority with error %d\n", err);
        printf("Exiting tx thread with dest=%s\n", dev->dest_ip);
        pthread_exit(NULL);
    }
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    // Force sync
    dev->ack_pend = MAX_ACK_PEND;
    if(SyncThreads(dev) < 0) {
        printf("Failed to sync threads\n");
        printf("Exiting tx thread with dest=%s\n", dev->dest_ip);
        pthread_exit(NULL);
    }

    // Loop while dev is running or cmds in buffer
    while(dev->runFlag) {
        // Send next msg on buffer
        sem_wait(&dev->buf_full);
#ifdef DEBUG_TX
        char frame_str[DBG_BUFF_SIZE];
        printf("send [%u]: %s\n", dev->cmd_consume_idx, get_msg_str(&(dev->cmd_buf[dev->cmd_consume_idx]), frame_str));
#endif
        if(SendMessage(&(dev->cmd_buf[dev->cmd_consume_idx]), dev->fd) < 0) {
            printf("Failed to send motor command\n");
            break;
        }
        dev->cmd_consume_idx = (dev->cmd_consume_idx + 1) % MAX_CMD_PEND;
        sem_post(&dev->buf_empty);
        sched_yield();

        // Handle if comm has lost sync
        pthread_mutex_lock(&dev->sync_mutex);
        dev->ack_pend++;
        needSync = dev->ack_pend > MAX_ACK_PEND;
#ifdef DEBUG_SYNC
        printf("Resync required\n");
#endif
        pthread_mutex_unlock(&dev->sync_mutex);
        if(needSync) {
            if(SyncThreads(dev) < 0) {
                printf("Failed to resync threads\n");
                break;
            }
        }
    }

    printf("Exiting tx thread with dest=%s\n", dev->dest_ip);
    pthread_exit(NULL);
}

// Initialize the device
static uint16_t ConnectDev(devHandle_t* dev)
{
    msg_t msg;

    const uint8_t* conn_msg[NUM_CONN_MSG] = {CONN_MSG1, CONN_MSG2, CONN_MSG3, CONN_MSG4};
    const uint8_t conn_msg_size[NUM_CONN_MSG] = {sizeof(CONN_MSG1), sizeof(CONN_MSG2), sizeof(CONN_MSG3), sizeof(CONN_MSG4)};
    const uint8_t conn_resp[NUM_CONN_MSG] = {CONN_RESP1, CONN_RESP2, CONN_RESP3, CONN_RESP4};

    for(uint8_t i = 0; i < NUM_CONN_MSG; i++) {
#ifdef DEBUG_STARTUP
        printf("Sending conn msg %d....", i);
#endif
        msg.length = conn_msg_size[i];
        memset(msg.RS232_data, 0, sizeof(msg.RS232_data));
        memcpy(msg.RS232_data, conn_msg[i], msg.length);
        if(SendMessage(&msg, dev->fd) < 0) {
            printf("Could not send conn msg %u\n", i);
            return CONN_ERR;
        }
#ifdef DEBUG_STARTUP
        printf("sent\n");
#endif

#ifdef DEBUG_STARTUP
        printf("Recving conn resp %d....", i);
#endif
        // Initialize receive buffer
        memset(&dev->rx_buf, 0, sizeof(msg_t));
        if (ReceiveMessage(&dev->rx_buf, dev->fd) <= 0) {
            printf("Could not recv conn resp %u\n", i);
            return CONN_ERR;
        }
#ifdef DEBUG_STARTUP
        printf("recvd\n");
#endif

        if(dev->rx_buf.RS232_data[0] != conn_resp[i]) {
            printf("Unexpected response to conn msg %u\n", i);
#ifdef DEBUG_STARTUP
            printf("Recvd:%d Exp:%d\n", dev->rx_buf.RS232_data[0], conn_resp[i]);
#endif
            return CONN_ERR;
        }
    }

    return CONN_OK;
}

static uint16_t DisconnectDev(devHandle_t* dev)
{
    msg_t msg;

#ifdef DEBUG_SHUTDOWN
    printf("Connecting to conn port....");
#endif
    if(ConnectSock(dev->dest_ip, dev->dest_conn_port, dev->fd) < 0) {
        printf("Failed to connect to conn port for remote addr %s\n", dev->dest_ip);
        CleanSock(&dev->fd);
        dev->fd = -1;
        return CONN_ERR;
    }
#ifdef DEBUG_SHUTDOWN
    printf("done\n");
#endif

#ifdef DEBUG_SHUTDOWN
    printf("Sending disconnect msg....");
#endif
    msg.length = sizeof(DISCONN_MSG);
    memset(msg.RS232_data, 0, sizeof(msg.RS232_data));
    memcpy(msg.RS232_data, DISCONN_MSG, msg.length);
    if (SendMessage(&msg, dev->fd)) {
        printf("Could not send disconnect msg\n");
        return CONN_ERR;
    }
#ifdef DEBUG_SHUTDOWN
    printf("done\n");
#endif

#ifdef DEBUG_SHUTDOWN
    printf("Recving disconnect resp....");
#endif
    memset(&dev->rx_buf, 0, sizeof(msg_t));
    if (ReceiveMessage(&dev->rx_buf, dev->fd) <= 0) {
        printf("Could not recv disconnect response\n");
        return CONN_ERR;
    }
#ifdef DEBUG_SHUTDOWN
    printf("done\n");
#endif

    if(dev->rx_buf.RS232_data[0] != DISCONN_RESP) {
        printf("Unexpected response to disconnect msg %x\n", dev->rx_buf.RS232_data[0]);
        return CONN_ERR;
    }

    return CONN_OK;
}

static void ShutdownDev(devHandle_t* dev)
{
    uint16_t runState = dev->runFlag;
    int err = 0;

#ifdef DEBUG_SHUTDOWN
    printf("\n***Shutting down dev***\n");
#endif

    // Stop the processing threads
    dev->runFlag = 0;

    if(dev->rxThreadValid) {
#ifdef DEBUG_SHUTDOWN
        printf("Shutting down rx thread....");
#endif
        pthread_cancel(dev->rxThreadHandle);
        if(pthread_join(dev->rxThreadHandle, NULL) != 0) {
            err = errno;
            printf("pthread_join for rx failed with errno=%d\n", err);
        }
#ifdef DEBUG_SHUTDOWN
        printf("done\n");
#endif
        dev->rxThreadValid = false;
    }

    if(dev->txThreadValid) {
#ifdef DEBUG_SHUTDOWN
        printf("Shutting down tx thread....");
#endif
        pthread_cancel(dev->txThreadHandle);
        if(pthread_join(dev->txThreadHandle, NULL) != 0) {
            err = errno;
            printf("pthread_join for tx failed with errno=%d\n", err);
        }
#ifdef DEBUG_SHUTDOWN
        printf("done\n");
#endif
        dev->txThreadValid = false;
    }

    if(dev->fd > -1) {
        if(runState) {
            printf("Processing threads complete, disconnecting\n");
            DisconnectDev(dev);
        }
        printf("Closing socket\n");
        CleanSock(&dev->fd);
    }    

    dev->ack_pend = 0;
    dev->cmd_consume_idx = 0;
    dev->cmd_produce_idx = 0;
    sem_destroy(&dev->buf_empty);
    sem_destroy(&dev->buf_full);

#ifdef DEBUG_SHUTDOWN
    printf("***Shutdown complete***\n");
#endif
}

static int StartDev(devHandle_t* dev)
{
    int err;
    uint16_t conn_count = 0;

#ifdef DEBUG_STARTUP
    printf("***Dev Settings***\n");
    printf("Local: %s:%d\n", dev->local_ip, dev->local_port);
    printf("Remote: %s:%d\n", dev->dest_ip, dev->dest_conn_port);
#endif 
    
    // In case we are currently running call shutdown first;
    ShutdownDev(dev);

#ifdef DEBUG_STARTUP
    printf("Configuring socket....");
#endif
    // Socket for all comm
    dev->fd = InitSock(dev->local_ip, dev->local_port, dev->dest_ip, dev->dest_conn_port, RX_TIMEOUT_MS);
    if(dev->fd < 0) {
        printf("Failed to init connection to %s\n", dev->dest_ip);
        return -1;
    }
#ifdef DEBUG_STARTUP
    printf("configured\n");
#endif
    while (conn_count < 5 && ConnectDev(dev) != CONN_OK) {
        conn_count++;
#ifdef DEBUG_STARTUP
        printf("Connection attempt %d failed\n\n", conn_count);
#endif
    }

    // Setup the connection to the technosoft drive
    if(conn_count < 5) {
        // Associate with the cmd port now
#ifdef DEBUG_STARTUP
        printf("Connecting to cmd port....");
#endif
        if(ConnectSock(dev->dest_ip, dev->dest_cmd_port, dev->fd) < 0) {
            printf("Failed to connect to remote address %s port %u\n", dev->dest_ip, CMD_PORT);
            return -1;
        }
#ifdef DEBUG_STARTUP
        printf("connected\n");
#endif
        // Initialize buffer synch structures
        sem_init(&dev->buf_empty, SEM_NOTSHARED, MAX_CMD_PEND);
        sem_init(&dev->buf_full, SEM_NOTSHARED, 0);
        // If we can talk with the drive, start the processing threads
        dev->runFlag = 1;
        if(pthread_create(&dev->rxThreadHandle, NULL, ThreadRxFunc, dev) != 0) {
            err = errno;
            printf("Failed to start dev rx thread with errno=%d\n", err);
            return -1;
        }
#ifdef DEBUG_STARTUP
        printf("Rx thread created\n");
#endif
        dev->rxThreadValid = true;
        if(pthread_create(&dev->txThreadHandle, NULL, ThreadTxFunc, dev) != 0) {
            err = errno;
            printf("Failed to start dev tx thread with errno=%d\n", err);
            return -1;
        }
#ifdef DEBUG_STARTUP
        printf("Tx thread created\n\n");
#endif
        dev->txThreadValid = true;
        return 0;
    } else {
        printf("Failed to connect to drive\n");
        return -1;
    }
}

// Add command to the buffer
static void AddCmd(devHandle_t* dev, msg_t* cmd)
{
    sem_wait(&dev->buf_empty);
    memcpy(&(dev->cmd_buf[dev->cmd_produce_idx]), cmd, sizeof(msg_t));
#ifdef DEBUG_TX
    //char frame_str[DBG_BUFF_SIZE];
    //printf("cmd: %s\n", get_msg_str(cmd, frame_str));
#endif
    dev->cmd_produce_idx = (dev->cmd_produce_idx + 1) % MAX_CMD_PEND;

    sem_post(&dev->buf_full);
}

void AddCmdEye(msg_t* cmd)
{
    AddCmd(&eye, cmd);
}

void AddCmdNeck(msg_t* cmd)
{
    AddCmd(&neck, cmd);
}

// Start the library
void __attribute__ ((constructor)) Start(void)
{
    int err; 
    struct timespec currTime = {0,0};
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    timebase = currTime.tv_sec*SECS_TO_NANO + currTime.tv_nsec;

    /*// Elevate priority of current process
    struct rlimit rlim = {0,0};
    if(getrlimit(RLIMIT_NICE, &rlim) < 0) {
        err = errno;
        printf("getrlim failed with errno=%d\n", err);
    }
    printf("Soft lim=%lu Hard Lim=%lu\n", rlim.rlim_cur, rlim.rlim_max);

    if(setpriority(PRIO_PROCESS, CURRENT_PID, PROCESS_PRIORITY) < 0) {
        err = errno;
        printf("setpriority failed with errno=%d\n", err);
    }*/

    InitLib(HOST_ID, BAUDRATE_115200);

    // Initialize eyeData, neckData and eyeCalData mutexes
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
        if(pthread_mutex_init(&eyeData.pos[i].mutex, NULL) != 0) {
            err = errno;
            printf("Failed to init eye axis %d mutex with errno=%d\n", i+1);
        }
    }

    for(int j = 0; j < NUM_NECK_AXIS; j++) {
        if(pthread_mutex_init(&neckData.pos[j].mutex, NULL) != 0) {
            err = errno;
            printf("Failed to init neck axis %d mutex with errno=%d\n", j+1, err);
        }
    }
    
#ifdef DEBUG_STARTUP
    // During tests, eye and neck need different ports
    eye.dest_cmd_port = EYE_CMD_PORT;
    eye.dest_conn_port = EYE_CONN_PORT;
    neck.dest_cmd_port = NECK_CMD_PORT;
    neck.dest_conn_port = NECK_CONN_PORT;
#endif

    if(StartDev(&eye) == 0 && StartDev(&neck) == 0) {
        printf("Initialized\n");
        DisableEyeCtrl();
        DisableNeckCtrl();
    } else {
        ShutdownDev(&eye);
        ShutdownDev(&neck);
        printf("Failed to properly initialize\n");
    }    
}

void __attribute__ ((destructor)) Cleanup(void)
{
    ShutdownDev(&eye);
    ShutdownDev(&neck);
    FlushLogs();

    // Initialize eyeData, neckData and eyeCalData mutexes
    int err;
    for(int i = 0; i < NUM_EYE_AXIS; i++) {
       if(pthread_mutex_destroy(&eyeData.pos[i].mutex) != 0) {
           err = errno;
           printf("Failed to destroy eye axis %d mutex with errno=%d\n", i+1, err);
           return;
        }
    }

    for(int j = 0; j < NUM_NECK_AXIS; j++) {
        if(pthread_mutex_destroy(&neckData.pos[j].mutex) != 0) {
            err = errno;
            printf("Failed to destroy neck axis %d mutex with errno=%d", j+1, err);
        }
    }
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

// Place eye motors in idle mode
void DisableEyeCtrl(void)
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    SendGoTo(DEV_EYE, &allMotors, WAIT_LOOP_EYE_IP);
    SetAxisControl(DEV_EYE, &allMotors, false);
    return;
}

// Place neck motors in idle mode
void DisableNeckCtrl(void)
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    SendGoTo(DEV_NECK, &allMotors, WAIT_LOOP_NECK_IP);
    SetAxisControl(DEV_NECK, &allMotors, false);
    return;
}

#define MAX_FIXED_POINT 32767.999969
#define MIN_FIXED_POINT -32767.999969

static uint32_t GetFixedPoint(double value)
{
    if(value >= MAX_FIXED_POINT  || value <= MIN_FIXED_POINT) {
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

void InitEyePosnCtrl()
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    SendGoTo(DEV_EYE, &allMotors, POSN_LOOP_EYE_IP);
    //SendSTA(DEV_EYE, &allMotors); automatically done with TUM0
    SetModePP(DEV_EYE, &allMotors);
    SetTUM(DEV_EYE, &allMotors, 0);
    SetAxisControl(DEV_EYE, &allMotors, true);
    UpdatePosn(DEV_EYE, &allMotors);
}

void SetEyePos(uint8_t axis, double pos_m)
{
    uint32_t data = GetLong(pos_m*M_TO_IU);
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal32(DEV_EYE, &dest, REG_CPOS, data);
}

void SetEyeSpeed(uint8_t axis, double speed_mps)
{
    uint32_t data = GetFixedPoint(speed_mps*MPS_TO_SPEED_IU);
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal32(DEV_EYE, &dest, REG_CSPD, data);
}

void SetEyeAccel(uint8_t axis, double accel_mpss)
{
    uint32_t data = GetFixedPoint(accel_mpss*MPSS_TO_ACCEL_IU);
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal32(DEV_EYE, &dest, REG_CACC, data);
}

void UpdateEyePos()
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    UpdatePosn(DEV_EYE, &allMotors);
}

void InitNeckPosnCtrl()
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    SendGoTo(DEV_NECK, &allMotors, POSN_LOOP_EYE_IP);
    //SendSTA(DEV_NECK, &allMotors); automatically done with TUM0
    SetModePP(DEV_NECK, &allMotors);
    SetTUM(DEV_NECK, &allMotors, 0);
    SetAxisControl(DEV_NECK, &allMotors, true);
    UpdatePosn(DEV_NECK, &allMotors);
}

void SetNeckPos(uint8_t axis, double pos_rad)
{
    uint32_t data;
    if (axis == NECK_YAW_AXIS) {
        data = GetLong(pos_rad*RAD_TO_IU_YAW);
    } else {
        data = GetLong(pos_rad*RAD_TO_IU);
    }        

    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal32(DEV_NECK, &dest, REG_CPOS, data);
}

void SetNeckSpeed(uint8_t axis, double speed_rps)
{
    uint32_t data;
    if(axis == NECK_YAW_AXIS) {
        data = GetFixedPoint(speed_rps*RADPS_TO_SPEED_IU_YAW);
    } else {
        data = GetFixedPoint(speed_rps*RADPS_TO_SPEED_IU);
    }
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal32(DEV_NECK, &dest, REG_CSPD, data);
}

void SetNeckAccel(uint8_t axis, double accel_rpss)
{
    uint32_t data;
    if(axis == NECK_YAW_AXIS) {
        data = GetFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU_YAW);
    } else {
        data = GetFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU);
    }      
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal32(DEV_NECK, &dest, REG_CACC, data);
}

void UpdateNeckPos()
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    UpdatePosn(DEV_NECK, &allMotors);
}

void StartEyeCal(uint8_t axis, double pos_m)
{
    if(axis <= 0 || axis > NUM_EYE_AXIS) {
        return;
    }

    SetEyePos(axis, pos_m);
    eyeCalData.complete[axis-1] = CAL_RUNNING;
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = 1};

    if(pos_m > 0) {
        SendGoTo(DEV_EYE, &dest, FOR_CAL_EYE_IP);
    }
    else {
        SendGoTo(DEV_EYE, &dest, REV_CAL_EYE_IP);
    }
}

void InitEyeForceCtrl()
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    SendGoTo(DEV_EYE, &allMotors, FORCE_LOOP_EYE_IP);
    // Set to initial forces
    for(uint8_t i = 0; i < NUM_NECK_AXIS; i++) {
        // axis id start at 1
        SetEyeForce(i+1, 0);
    }
    SetExtRefOnline(DEV_EYE, &allMotors);
    SetModeTorqueSlow(DEV_EYE, &allMotors);
    SetAxisControl(DEV_EYE, &allMotors, true);
    
    // Update all 
    UpdatePosn(DEV_EYE, &allMotors);    
}

void SetEyeForce(uint8_t axis, double force_n)
{
    uint16_t data;
    data = FORCE_TO_IU_EYE(force_n);
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal16(DEV_EYE, &dest, REG_EREFT, data);
}

void InitNeckTorqueCtrl()
{
    motor_id_t allMotors = {.type = ID_TYPE_BROADCAST, .id = 0};
    SendGoTo(DEV_NECK, &allMotors, TORQUE_LOOP_NECK_IP);
    // Set initial torques
    for(uint8_t i = 0; i < NUM_EYE_AXIS; i++) {
        // axis id start at 1
        SetNeckTorque(i+1, 0);
    }
    SetExtRefOnline(DEV_NECK, &allMotors);
    SetModeTorqueSlow(DEV_NECK, &allMotors);
    SetAxisControl(DEV_NECK, &allMotors, true);

    // Update everything
    UpdatePosn(DEV_NECK, &allMotors);
}

void SetNeckTorque(uint8_t axis, double torque_nm)
{
    uint16_t data;
    if(axis == NECK_YAW_AXIS) {
        data = TORQUE_TO_IU_NECK_YAW(torque_nm);
    } else {
        data = TORQUE_TO_IU_NECK(torque_nm);
    }
    motor_id_t dest = {.type = ID_TYPE_AXIS, .id = axis};
    SetVal16(DEV_NECK, &dest, REG_EREFT, data);
}