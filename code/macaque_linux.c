/*
    TODO:
    - Replace SwitchToThread with Linux-equivalent
    - Figure out how byte ordering works
    - Figure out what to do for debug window
*/

// _DEFAULT_SOURCE defined for ifreq struct
#define _DEFAULT_SOURCE
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <time.h>
#include "macaque_linux.h"
#include "cansock.h"
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
#include <errno.h>
#include <sys/resource.h>

#define M_PI            ((double)3.14159265358979323846)
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

#define GETBYTES_FROM_WORD(src,dst_p) (*(uint16_t*)(dst_p) = htons((uint16_t)src))
#define GETBYTES_FROM_LONG(src,dst_p) (*(uint32_t*)(dst_p) = htonl((uint32_t)src))
#define GETWORD_FROM_BYTES(src_p) (ntohs(*(uint16_t*)(src_p)))
#define GETLONG_FROM_BYTES(src_p) (ntohs((*(uint32_t*)src_p)&0xFFFF) | (ntohs((*(uint32_t*)src_p)>>16)<<16))

#define NANO_TO_SECS                (1000*1000*1000)
#define CURRENT_TID                 (0)
#define RX_THREAD_PRIORITY          (1)
#define TX_THREAD_PRIORITY          (10)
#define TX_THREAD_SLEEP             (100*1000) // us 

#define NECK_CAN_ID                 (0x102)
#define NECK_CAN_IFNAME             "can0"
#define EYE_CAN_ID                  (0x101)
#define EYE_CAN_IFNAME              "can1"
#define CONTROL_CAN_ID              (0x100)
#define IFNAME_LEN                  4

#define CURRENT_PID                    0
#define PROCESS_PRIORITY                -20  // -20 (highest) to 19 (lowest)

#define SEC_TO_MSEC         (1000)
#define MSEC_TO_USEC        (1000)
#define MSEC_TO_NSEC        (1000*1000)

// TODO this implementation
bool ErrorHandler(uint32_t fdwCtrlType) {


}

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

typedef struct msg
{
    uint8_t msg[BUFLEN];
    int  size;
} msg_t;

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
    int fd;
    uint32_t can_id;
    char            ifName[IFNAME_LEN];
    
    int         	runFlag;
    pthread_t      	txThreadHandle;
    pthread_t      	rxThreadHandle;
    char        	rx_buf[BUFLEN];

    uint32_t         	ack_pend;
    uint8_t		force_sync;

    msg_t		cmd_buf[MAX_CMD_PEND];
    uint32_t		cmd_consume_idx;
    uint32_t		cmd_produce_idx;
    uint8_t		host_id;

    rxCallbackFxn	callback;

} devHandle_t;

devHandle_t eye  = {.can_id = EYE_CAN_ID, .ifName = EYE_CAN_IFNAME,.force_sync=0,
                    .ack_pend = 0, .cmd_consume_idx = 0, .cmd_produce_idx = 0, 
		    .host_id = EYE_HOST_ID, .callback = &eyeRxCallback};

devHandle_t neck = {.can_id = NECK_CAN_ID, .ifName = NECK_CAN_IFNAME, .force_sync=0,
                    .ack_pend = 0, .cmd_consume_idx = 0, .cmd_produce_idx = 0, 
		    .host_id = NECK_HOST_ID, .callback = &neckRxCallback};

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
        case REG_ADDR_APOS & REG_ADDR_MASK:
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
        case REG_ADDR_APOS & REG_ADDR_MASK:
            log_data_id = 1;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.pos[axis_id-1] = converted_data;
            break;

        case REG_ADDR_POSERR & REG_ADDR_MASK:
            log_data_id = 2;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.err[axis_id-1] = converted_data;
            break;

        case REG_ADDR_TPOS & REG_ADDR_MASK:
            log_data_id = 3;
            converted_data = convert_iu_to_m(data);
            eyeCalData.time = time;
            eyeCalData.tpos[axis_id-1] = converted_data;
            break;

        case REG_ADDR_APOS2 & REG_ADDR_MASK:
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

        case CAL_RUN_VAR & REG_ADDR_MASK:
            eyeCalData.complete[axis_id-1] = data;
            break;

	    case CAL_APOS2_OFF_VAR & REG_ADDR_MASK:
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

static uint8_t gen_cksum(uint8_t *msg, uint8_t size)
{
    uint8_t cksum=0;
    uint8_t n=0;
    for(n=0; n < size-1; n++)
    {
        cksum+=msg[n];
    }

    return cksum&0xFF;
}

static uint16_t build_addr(uint8_t axis_id, uint8_t is_host, uint8_t is_group)
{
    uint16_t addr;
    addr = (axis_id&0xFF) << 4;
    addr |= (is_host)?0x0001:0x0000;
    addr |= (is_group)?0x1000:0x0000;
    return addr;
}

static uint8_t get_addr(uint16_t formatted)
{
    uint8_t addr;
    addr = (uint8_t)((formatted&0x0FF0) >> 4);
    return addr;
}

static void parse_response_msg(devHandle_t* dev)
{   
    uint8_t *msg = dev->rx_buf;
    int32_t result = 0;
    uint16_t axis;
    uint16_t reg_addr;

    if(msg == NULL)
    {  
        return;
    }

    int size = msg[MSG_SIZE_OFFSET]+2;
    
    if( msg[size-1] != gen_cksum(msg,size) || size < MSG_TYPEA_BASE)
    {
        return;
    }

    uint16_t dst_addr   = get_addr(GETWORD_FROM_BYTES(&msg[MSG_ADDR_OFFSET]));
    uint16_t optcode    = GETWORD_FROM_BYTES(&msg[MSG_OPT_OFFSET]);

    if (((optcode&MSG_TAKE_OPTCODE_MASK) == (OPT_1WORD_RESP_GRP&MSG_TAKE_OPTCODE_MASK)) 
        && size == MSG_TAKE1WORD_SIZE)
    {
        axis         = optcode&0xFF;      
        reg_addr     = GETWORD_FROM_BYTES(&msg[MSG_TAKE_REG_ADDR_OFFSET]);
        int16_t temp = GETWORD_FROM_BYTES(&msg[MSG_TAKE_DATA_OFFSET]);
        result       = (int32_t)temp;
    }
    else if (((optcode&MSG_TAKE_OPTCODE_MASK) == (OPT_2WORD_RESP_GRP&MSG_TAKE_OPTCODE_MASK))
             && size == MSG_TAKE2WORD_SIZE)
    {
        axis     = optcode&0xFF;
        reg_addr = GETWORD_FROM_BYTES(&msg[MSG_TAKE_REG_ADDR_OFFSET]);	
        result   = GETLONG_FROM_BYTES(&msg[MSG_TAKE_DATA_OFFSET]);
    }	
    else if(optcode == OPT_1WORD_RESP && size == MSG_GIVE1WORD_SIZE)
    {
        axis         = get_addr(GETWORD_FROM_BYTES(&msg[MSG_SEND_ADDR_OFFSET]));
        reg_addr     = GETWORD_FROM_BYTES(&msg[MSG_GIVE_REG_ADDR_OFFSET]);
        int16_t temp = GETWORD_FROM_BYTES(&msg[MSG_GIVE_DATA_OFFSET]);
        result       = (int32_t)temp;
    }
    else if(optcode == OPT_2WORD_RESP && size == MSG_GIVE2WORD_SIZE)
    { 
        axis       = get_addr(GETWORD_FROM_BYTES(&msg[MSG_SEND_ADDR_OFFSET]));
        reg_addr   = GETWORD_FROM_BYTES(&msg[MSG_GIVE_REG_ADDR_OFFSET]);
        result     = GETLONG_FROM_BYTES(&msg[MSG_GIVE_DATA_OFFSET]);
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
    struct sockaddr_can source_addr;
    int source_addr_len = sizeof(source_addr);
    int rx_bytes;
    struct sched_param param = {RX_THREAD_PRIORITY};
    int err = 0;

    if(sched_setscheduler(CURRENT_TID, SCHED_RR, &param)) {
        err = errno;
        printf("Failed to set RX Thread priority with error %d\n", err);
    }

    while(dev->runFlag) {
        memset(dev->rx_buf,0, BUFLEN);
        if(ReceivePayload(dev->rx_buf, dev->fd)) {
            printf("Failed to properly rx CAN message\n");
            break;
        } else {
            if(dev->force_sync) {
                // Check if this is sync return message
                if(rx_bytes == CONN_SYNC_BYTES && dev->rx_buf[CONN_SYNC_BYTES-1] == CONN_SYNC_RESP) {
                    dev->force_sync = 0;
                    dev->ack_pend = 0;
                } else if(dev->rx_buf[0] == MSG_ACK && dev->ack_pend > 0) {
                    // Check if ack message has been received (empty message)
                    dev->ack_pend--;
                } else {
                    // Parse the msg for relevant data
                    parse_response_msg(dev);
                }
            }
        }
    }
}

void* ThreadTXFunc(void* input) 
{
    devHandle_t* dev = (devHandle_t*)input;
    unsigned int index;
    uint8_t sync_cmd[CONN_SYNC_BYTES];
    memset(sync_cmd, CONN_SYNC_BYTE, CONN_SYNC_BYTES);
    struct sched_param param = {TX_THREAD_PRIORITY};
    CAN_MSG msg = {0,0,{0}};
    int err = 0;

    if(sched_setscheduler(CURRENT_TID, SCHED_RR, &param)) {
        err = errno;
        printf("Failed to set RX Thread priority with error %d\n", err);
    }

    // Loop while dev is running or cmds in buffer
    while(dev->runFlag || (dev->cmd_produce_idx - dev->cmd_consume_idx) != 0) {  
        if(dev->force_sync) { 
            if (SendMessage(&msg, dev->fd) < 0) {
                printf("Failed to tx sync CAN message\n");
                break;
            }
            usleep(TX_THREAD_SLEEP);
        } 
        else if(dev->ack_pend < MAX_ACK_PEND && (dev->cmd_produce_idx - dev->cmd_consume_idx) != 0) {  
            // check if cmds are still remaining in buf or not too many pending acks
            index = dev->cmd_consume_idx % MAX_CMD_PEND;

            if (SendMessage(&msg, dev->fd) < 0) {
                printf("sendto() failed with error code : %d\n" , WSAGetLastError());
                break;
            }

            dev->cmd_consume_idx++;
            dev->ack_pend++;
        }
        else {
            if(dev->ack_pend > MAX_ACK_PEND) {
                //our communication link may have gotten confused
                //resync
                printf("lost too many MSG ACKS, resyncing...\n");
                dev->force_sync=1;
            }
            //yield, ethier we have too many messages pending or nothing to consume
            //SwitchToThread();
            usleep(TX_THREAD_SLEEP); // We will try sleeping it first
            sched_yield();      // a little different than SwitchToThread
        }
    }
}

// Force device to enter sync mode
static void sync(devHandle_t* dev)
{
    dev->force_sync=1;
}

// Initialize the device
static uint16_t connectDev(devHandle_t* dev)
{
    CAN_MSG msg;

    msg.identifier = CONTROL_CAN_ID;
    msg.length = sizeof(CONN_MSG);
    memset(msg.CAN_data, 0, sizeof(msg.CAN_data));
    memcpy(msg.CAN_data, CONN_MSG, msg.length);
    if (SendMessage(&msg, dev->fd) < 0) {
        printf("Could not send connectDev msg\n");
        return CONN_ERR;
    }
    
    // Initialize receive buffer
    memset(dev->rx_buf,0, BUFLEN);
    if (ReceivePayload(dev->rx_buf, dev->fd) < 0) {
        printf("Could not receive connectDev response\n");
        return CONN_ERR;
    }

    if(dev->rx_buf[0] != CONN_RESP) {
        printf("Unexpected response to connectDev msg\n");
        return CONN_ERR;
    }

    msg.identifier = CONTROL_CAN_ID;
    msg.length = sizeof(CONN_CONFIG);
    memset(msg.CAN_data, 0, sizeof(msg.CAN_data));
    memcpy(msg.CAN_data, CONN_CONFIG, msg.length);
    if (SendMessage(&msg, dev->fd) < 0) {
        printf("Could not send connect config msg\n");
        return CONN_CONFIG_ERR;
    }

    memset(dev->rx_buf,0, BUFLEN);
    if (ReceivePayload(&msg, dev->fd) < 0) {
        printf("Could not receive connect config response\n");
        return CONN_CONFIG_ERR;
    }

    if(dev->rx_buf[0] != CONN_CONFIG_RESP) {
        printf("Unexpected response to connect config msg\n");
        return CONN_CONFIG_ERR;
    }

    sync(dev);

    return CONN_OK;
}

static uint16_t disconnectDev(devHandle_t* dev)
{
    dev->force_sync = 0;
    CAN_MSG msg;

    memset(&msg, 0, sizeof(msg));
    msg.identifier = CONTROL_CAN_ID;
    msg.length = sizeof(DISCONN_MSG);
    memset(msg.CAN_data, 0, sizeof(msg.CAN_data));
    memcpy(msg.CAN_data, DISCONN_MSG, msg.length);
    if (SendMessage(&msg, dev->fd)) {
        printf("Could not send disconnect msg\n");
        return CONN_ERR;
    }
    
    memset(dev->rx_buf,0, BUFLEN);
    if (ReceivePayload(dev->rx_buf, dev->fd)) {
        printf("Could not receive disconnect response\n");
        return CONN_ERR;
    }

    if(dev->rx_buf[0] != DISCONN_RESP) {
        printf("Unexpected response to disconnect msg\n");
        return CONN_ERR;
    }

    return CONN_OK;
}

static void shutdownDev(devHandle_t* dev)
{
    uint16_t runState = dev->runFlag;
    int err = 0;

    // Stop the processing threads
    dev->runFlag = 0;

    if(dev->rxThreadHandle != NULL) {
        pthread_cancel(dev->rxThreadHandle);
        if(pthread_join(dev->rxThreadHandle, NULL) != 0) {
            err = errno;
            printf("pthread_join for rx failed with errno=%d\n", err);
        }
        dev->rxThreadHandle = NULL;
    }

    if(dev->txThreadHandle != NULL)
    {
        pthread_cancel(dev->txThreadHandle);
        if(pthread_join(dev->txThreadHandle, NULL) != 0) {
            err = errno;
            printf("pthread_join for tx failed with errno=%d\n", err);
        }
        dev->txThreadHandle = NULL;
    }

    if(dev->fd > -1)
    {
        if(runState) {
            printf("Processing threads complete, disconnecting\n");
            disconnectDev(dev);
        }
        printf("Closing Socket\n");
        CleanCanSock(dev->fd);
        dev->fd = -1;
    }    

    dev->force_sync=0;
    dev->ack_pend = 0;
    dev->cmd_consume_idx = 0;
    dev->cmd_produce_idx = 0;
}

static void startDev(devHandle_t* dev)
{
    // In case we are currently running call shutdown first;
    shutdownDev(dev);
    
    //Create a socket
    if(InitCanSock(dev->can_id, dev->ifName, RX_TIMEOUT_MS)) {
        printf("Failed to init CAN on %s interface\n", dev->ifName);
        return;
    }

    printf("CAN configured.\n");
    uint16_t conn_count=0;
    while (conn_count < 5 && connectDev(dev) != CONN_OK) {
        conn_count++;
    }
    
    //setup the connection to the technosoft drive
    if(conn_count<5) {
        // If we can talk with the drive, start the processing threads
        dev->runFlag=1;
        dev->rxThreadHandle = CreateThread(NULL, 0, ThreadRXFunc, dev, 0, NULL);
        dev->txThreadHandle = CreateThread(NULL, 0, ThreadTXFunc, dev, 0, NULL);
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
    while (dev->cmd_produce_idx - dev->cmd_consume_idx == MAX_CMD_PEND)
    {
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

// Build message which does not require a return message from motor drive
static void buildMsgTypeA(msg_t* msg_s, uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{
    if(words > 2 || msg_s == NULL)
    {
        words = 2;
    }

    uint8_t* msg = msg_s->msg;
    uint8_t msg_size = MSG_TYPEA_BASE+(words*2);
    uint16_t dest_addr_formatted = build_addr(axis_id,0,group);

    msg[MSG_SIZE_OFFSET] = msg_size-2; 
    GETBYTES_FROM_WORD(dest_addr_formatted, &msg[MSG_ADDR_OFFSET]);
    GETBYTES_FROM_WORD(optcode, &msg[MSG_OPT_OFFSET]);
   
    if(words == 1)
    {
        GETBYTES_FROM_WORD(0xFFFF&data, &msg[MSG_TYPEA_BASE-1]);
    }
    else if(words == 2)
    {
        GETBYTES_FROM_LONG(data, &msg[MSG_TYPEA_BASE-1]);
    }

    msg[msg_size-1] = gen_cksum(msg, msg_size); 

    msg_s->size = msg_size;
}

// Build message which requires a response (such as a read of a register)
static void buildMsgTypeB(msg_t* msg_s, uint8_t axis_id, uint8_t return_addr, uint16_t reg_addr, uint8_t words)
{
    uint8_t* msg = msg_s->msg;
    uint16_t optcode;
    uint16_t dest_addr_formatted = build_addr(axis_id,0,0);
    uint16_t send_addr_formatted = build_addr(return_addr,1,0);

    if (words == 1)
        optcode=OPT_1WORD_REQ;
    else
        optcode=OPT_2WORD_REQ;

    msg[MSG_SIZE_OFFSET] = MSG_TYPEB_BASE-2; 

    GETBYTES_FROM_WORD(dest_addr_formatted, &msg[MSG_ADDR_OFFSET]);
    GETBYTES_FROM_WORD(optcode, &msg[MSG_OPT_OFFSET]);
    GETBYTES_FROM_WORD(send_addr_formatted, &msg[MSG_SEND_ADDR_OFFSET]);
    GETBYTES_FROM_WORD(reg_addr, &msg[MSG_GIVE_REG_ADDR_OFFSET]);

    msg[MSG_TYPEB_BASE-1] = gen_cksum(msg, MSG_TYPEB_BASE);

    msg_s->size = MSG_TYPEB_BASE;
}

// Format and place type b message into buffer (requires response from drive)
static void sendMsgTypeA(devHandle_t* dev, uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{ 
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;

    buildMsgTypeA(msgSlot, axis_id, optcode, data, words, group);
    dev->cmd_produce_idx++;
}

// Format and place a type b message into buffer (requires response from drive)
static void sendMsgTypeB(devHandle_t* dev, uint8_t axis_id, uint8_t return_addr, uint16_t reg_addr, uint8_t words)
{    
    msg_t* msgSlot;

    if((msgSlot = getMsgSlot(dev)) == NULL)
        return;

    buildMsgTypeB(msgSlot, axis_id, return_addr, reg_addr, words);
    dev->cmd_produce_idx++;
}

// Wrapper function for sending Type A message to eye control 
void sendMsgTypeAEye(uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{
    sendMsgTypeA(&eye, axis_id, optcode, data, words, group);
}

// Wrapper function for sending Type B message to eye control
void sendMsgTypeBEye(uint8_t axis_id, uint16_t reg_addr, uint8_t words)
{
    sendMsgTypeB(&eye, axis_id, eye.host_id, reg_addr, words);
}

// Wrapper function for sending Type A message to neck control
void sendMsgTypeANeck(uint8_t axis_id, uint16_t optcode, uint32_t data, uint8_t words, uint8_t group)
{
    sendMsgTypeA(&neck, axis_id, optcode, data, words, group);
}

// Wrapper function sending Type B message to neck control
void sendMsgTypeBNeck(uint8_t axis_id, uint16_t reg_addr, uint8_t words)
{
    sendMsgTypeB(&neck, axis_id, neck.host_id, reg_addr, words);
}

// Start the library
void start(void)
{
    int err; 
    struct timespec currTime = {0,0};
    clock_gettime(CLOCK_MONOTONIC, &currTime);
    timebase = currTime.tv_sec*NANO_TO_SECS + currTime.tv_nsec;

    int n;

    // Elevate priority of current process
    if(setpriority(PRIO_PROCESS, CURRENT_PID, PROCESS_PRIORITY)) {
        err = errno;
        printf("setpriority failed with errno=%d\n", err);
    }


    disEyePollData();
    startDev(&eye);

    disNeckPollData();
    startDev(&neck);

    printf("Initialised.\n");
}

void cleanup(void)
{
    shutdownDev(&eye);
    shutdownDev(&neck);
}

eyeData_t* getEyeData(void)
{
    return &eyeData;
}

eyeCalData_t* getEyeCalData(void)
{
    return &eyeCalData;
}

neckData_t* getNeckData(void)
{
    return &neckData;
}

void enEyePollData(void)
{
    sendMsgTypeAEye(1, OPT_GOTO, MOTION_LOOP_IP, 1, 1);
    return;
}

void disEyePollData(void)
{
    sendMsgTypeAEye(1, OPT_GOTO, WAIT_LOOP_IP, 1, 1);
    return;
}

void enNeckPollData(void)
{
    sendMsgTypeANeck(1, OPT_GOTO, MOTION_LOOP_NECK_IP, 1, 1);
    return;
}

void disNeckPollData(void)
{
    sendMsgTypeANeck(1, OPT_GOTO, WAIT_LOOP_NECK_IP, 1, 1);
    return;
}

#define MAX_FIXED_POINT 32767.999969
#define MIN_FIXED_POINT -32767.999969

static uint32_t getFixedPoint(double value)
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

static uint32_t getLong(int32_t value)
{
    uint32_t result = ((uint32_t)value << 16) | ((value>>16)&0xFFFF);
    return result;
}

void setEyePos(uint8_t axis, double pos_m)
{
    uint32_t data = getLong(pos_m*M_TO_IU);
    sendMsgTypeAEye(axis, OPT_CPOS, data, 2, 0);
}

void setNeckPos(uint8_t axis, double pos_rad)
{
    uint32_t data;
    if (axis == NECK_YAW_AXIS)
        data = getLong(pos_rad*RAD_TO_IU_YAW);
    else
        data = getLong(pos_rad*RAD_TO_IU);
    
    sendMsgTypeANeck(axis, OPT_CPOS, data, 2, 0);
}

void setNeckSpeed(uint8_t axis, double speed_rps)
{
    uint32_t data;
    
    if(axis == NECK_YAW_AXIS)
        data = getFixedPoint(speed_rps*RADPS_TO_SPEED_IU_YAW);
    else
        data = getFixedPoint(speed_rps*RADPS_TO_SPEED_IU);
    
    sendMsgTypeANeck(axis, OPT_CSPD, data, 2, 0);
}

void setEyeSpeed(uint8_t axis, double speed_mps)
{
    uint32_t data = getFixedPoint(speed_mps*MPS_TO_SPEED_IU);
    sendMsgTypeAEye(axis, OPT_CSPD, data, 2, 0);
}

void setNeckAccel(uint8_t axis, double accel_rpss)
{
    uint32_t data;
    
    if(axis == NECK_YAW_AXIS)
        data = getFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU_YAW);
    else
        data = getFixedPoint(accel_rpss*RADPSS_TO_ACCEL_IU);
    
    sendMsgTypeANeck(axis, OPT_CACC, data, 2, 0);
}

void setEyeAccel(uint8_t axis, double accel_mpss)
{
    uint32_t data = getFixedPoint(accel_mpss*MPSS_TO_ACCEL_IU);
    sendMsgTypeAEye(axis, OPT_CACC, data, 2, 0);
}

void startEyeCal(uint8_t axis, double pos_m)
{
    if(axis <=0 || axis > NUM_EYE_AXIS)
    {
        return;
    }

    uint32_t data;

    setEyePos(axis, pos_m);
    eyeCalData.complete[axis-1] = CAL_RUNNING;

    if(pos_m > 0)
    {
        sendMsgTypeAEye(axis, OPT_GOTO, FOR_CAL_IP, 1, 0);
    }
    else
    {
        sendMsgTypeAEye(axis, OPT_GOTO, REV_CAL_IP, 1, 0);
    }
}