#ifndef MACAQUE_LINUX_H
#define MACAQUE_LINUX_H

#include <stdint.h>
#include <pthread.h>
#include "../sock_interface/sock.h"

#define SHUTDOWN_TIMEOUT_MS 	    (2000)
#define RX_TIMEOUT_MS		        (1000)
#define SEND_TIMEOUT_S		        (1.0)
#define BUFLEN 			            (100)  	
#define LOG_BUFLEN		            (1024)
#define LOG_FILELEN                 (15)
#define EYE_LOG_FILENAME            ("./eye_log.csv")
#define NECK_LOG_FILENAME           ("./neck_log.csv")
#define MAX_ACK_PEND 		        (8)  //limit how much we pound the drives with polls for data
#define MAX_CMD_PEND 		        (uint8_t)(8)  //must be a power of 2

#define CONN_MSG1		            ((uint8_t[]){0x01,0xA4,0x06})
#define CONN_RESP1		            ((uint8_t)0x02)
#define CONN_MSG2                   ((uint8_t[]){0x03,0x80,0x25,0x00,0x00})
#define CONN_RESP2                  ((uint8_t)0x04)
#define CONN_MSG3                   ((uint8_t[]){0x07,0x00})
#define CONN_RESP3                  ((uint8_t)0x08)
#define CONN_MSG4	                ((uint8_t[]){0x03,0x00,0xC2,0x01,0x00})//1Mbit config
#define CONN_RESP4      	        ((uint8_t)0x04)
#define NUM_CONN_MSG                (4)
#define CONN_SYNC_RESP		        (0x0Du)
#define CONN_SYNC_RESP_BYTES        (15)
#define CONN_SYNC_BYTES		        (15)
#define CONN_SYNC   		        (0xFFu)
#define DISCONN_MSG		            ((uint8_t[]){0x05})
#define DISCONN_RESP		        ((uint8_t)0x06)

typedef enum {
    CONN_OK,
    CONN_ERR,
    CONN_CONFIG_ERR,
    CONN_SYNC_ERR,
} conn_state_t;

//#define STARTUP_TEST
#ifndef STARTUP_TEST
#define HOST_ID                     (120)
#define LOCAL_IP                    ("192.168.2.8")
#define NECK_LOCAL_PORT		        (51244u)
#define EYE_LOCAL_PORT		        (51243u)
#define NECK_IP			            ("192.168.2.15")
#define EYE_IP			            ("192.168.2.14")
#define CMD_PORT                    (uint16_t)(1700u)
#define CONN_PORT		            (uint16_t)(30689u)
#define MSG_ACK			            ((uint8_t)0x4F)
#else
#define HOST_ID                     (120)
#define LOCAL_IP                    ("192.168.2.8")
#define NECK_LOCAL_PORT		        (51244u)
#define EYE_LOCAL_PORT		        (51243u)
#define NECK_IP			            ("192.168.2.2")
#define EYE_IP			            ("192.168.2.2")
#define CMD_PORT                    (uint16_t)(1700u)
#define CONN_PORT		            (uint16_t)(30689u)
#define EYE_CMD_PORT                CMD_PORT
#define EYE_CONN_PORT               CONN_PORT
#define NECK_CMD_PORT               (CMD_PORT+1)
#define NECK_CONN_PORT              (CONN_PORT+1)
#define MSG_ACK			            ((uint8_t)0x4F)
#endif

typedef RS232_MSG msg_t;

// Helpers to transfer between axis id and data array idx
#define AXISID_TO_DATAIDX(id)       (id-1)
#define DATAIDX_TO_AXISID(idx)      (idx+1)

// For accessing eyeData pos array
typedef enum {
    RIGHT_EYE_YAW_IDX,
    RIGHT_EYE_PITCH_IDX,
    LEFT_EYE_PITCH_IDX,
    LEFT_EYE_YAW_IDX,
    NUM_EYE_AXIS,
} eye_pos_data_idx_t;

// For accessing eyeCalData and eyeData torque array
typedef enum {
    LEFT_EYE_LEFT_AXIS_IDX,
    LEFT_EYE_RIGHT_AXIS_IDX,
    RIGHT_EYE_LEFT_AXIS_IDX,
    RIGHT_EYE_RIGHT_AXIS_IDX,
} eye_data_idx_t;

// Axis id mapping () -> directions from behind head
typedef enum {
    RIGHT_EYE_RIGHT_AXIS = 1,
    RIGHT_EYE_LEFT_AXIS,
    LEFT_EYE_RIGHT_AXIS,
    LEFT_EYE_LEFT_AXIS,
} eye_axis_t;

// Log data id
typedef enum {
    EYE_YAW_LEFT_OFFSET,
    EYE_PITCH_LEFT_OFFSET,
    EYE_YAW_RIGHT_OFFSET,
    EYE_PITCH_RIGHT_OFFSET,
    EYE_LEFT_LEFT_POS,
    EYE_LEFT_RIGHT_POS,
    EYE_RIGHT_LEFT_POS,
    EYE_RIGHT_RIGHT_POS,
    EYE_LEFT_LEFT_POSERR,
    EYE_LEFT_RIGHT_POSERR,
    EYE_RIGHT_LEFT_POSERR,
    EYE_RIGHT_RIGHT_POSERR,
    EYE_LEFT_LEFT_TARGET,
    EYE_LEFT_RIGHT_TARGET,
    EYE_RIGHT_LEFT_TARGET,
    EYE_RIGHT_RIGHT_TARGET,
    EYE_LEFT_LEFT_FORCE,
    EYE_LEFT_RIGHT_FORCE,
    EYE_RIGHT_LEFT_FORCE,
    EYE_RIGHT_RIGHT_FORCE,
} eye_log_data_id_t;

typedef enum {
    EYE_ENCODER,
    EYE_HALL,
    EYE_POSERR,
    EYE_TARGET,
    EYE_CURRENT,
} eye_log_data_types_t;

// Neck drive defines
// Neck axis id mapping
typedef enum {
    NECK_YAW_AXIS = 1,
    NECK_PITCH_AXIS,
    NECK_ROLL_AXIS,
} neck_drive_t;

// For accessing torque/pos arrays in neck data
typedef enum {
    NECK_YAW_AXIS_IDX,
    NECK_PITCH_AXIS_IDX,
    NECK_ROLL_AXIS_IDX,
    NUM_NECK_AXIS,
} neck_data_idx_t;

// Log data id
typedef enum {
    NECK_YAW_POS,
    NECK_PITCH_POS,
    NECK_ROLL_POS,
    NECK_YAW_TORQUE,
    NECK_PITCH_TORQUE,
    NECK_ROLL_TORQUE,
} neck_log_data_id_t;

// Log data id
typedef enum {
    NECK_POS,
    NECK_TORQUE,
} neck_log_data_types_t;

// Calibration states
typedef enum {
    CAL_COMPLETE,
    CAL_RUNNING,
} eye_cal_states_t;

// Position struct for pid
typedef struct _pos_t{
    double pos;
    double time;
    pthread_mutex_t mutex;
} pos_t;

typedef struct eyeCalData {
    double time;
    double pos[NUM_EYE_AXIS]; // see eye_data_idx_t for idx ordering 
    double err[NUM_EYE_AXIS]; // see eye_data_idx_t for idx ordering
    double tpos[NUM_EYE_AXIS]; // see eye_data_idx_t for idx ordering
    uint16_t complete[NUM_EYE_AXIS]; // see eye_data_idx_t for idx ordering
} eyeCalData_t;

typedef struct eyeData {
    double time;
    pos_t pos[NUM_EYE_AXIS]; // see eye_pos_data_idx_t for idx ordering
    double offset[NUM_EYE_AXIS]; // see eye_pos_data_idx_t for idx ordering
    double force[NUM_EYE_AXIS]; // see eye_data_idx_t for idx ordering
} eyeData_t;

typedef struct neckData {
    double time;
    pos_t pos[NUM_NECK_AXIS];
    double torque[NUM_NECK_AXIS];
    uint16_t ready[NUM_NECK_AXIS];
} neckData_t;

/* neck controller functions */
// Get some data
neckData_t* GetNeckData(void);
// Position Control
void InitNeckPosnCtrl();
void DisableNeckCtrl(void);
// Set axis = 0 for broadcast
void SetNeckPosn(uint8_t axis, double pos_rad);
void SetNeckSpeed(uint8_t axis, double speed_rps);
void SetNeckAccel(uint8_t axis, double accel_rpss);
void UpdateNeck(uint8_t axis);
// Torque Control
void InitNeckTorqueCtrl(uint8_t axis);
void SetNeckTorque(uint8_t axis, double torque_nm);
/* neck controller function */

/* eye controller functions */
// Get some data
eyeData_t* GetEyeData(void);
// Calibration sequence
eyeCalData_t* GetEyeCalData(void);
void StartEyeCal(uint8_t axis, double pos_m);
// Position control
void InitEyePosnCtrl(void);
void DisableEyeCtrl(void);
// Set axis = 0 for broadcast
void SetEyePosn(uint8_t axis, double pos_m);
void SetEyeSpeed(uint8_t axis, double speed_mps);
void SetEyeAccel(uint8_t axis, double accel_mpss);
void UpdateEye(uint8_t axis);
// Torque control
void InitEyeForceCtrl(uint8_t axis);
void SetEyeForce(uint8_t axis, double force_n);
/* eye controller functions */

// Flush log buffers into corresponding files
void FlushLogs();

// Constructors/Destructor for the library
void Start(void);
void Cleanup(void);

// For RS232 module to write frames to buffer
void AddCmdEye(msg_t* cmd);
void AddCmdNeck(msg_t* cmd);

#endif 