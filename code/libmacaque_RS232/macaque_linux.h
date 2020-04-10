#ifndef MACAQUE_LINUX_H
#define MACAQUE_LINUX_H

#include <stdint.h>
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
#define MAX_CMD_PEND 		        (uint8_t)(16)  //must be a power of 2

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
#define CONN_SYNC_RESP_BYTES        (1)
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

#define STARTUP_TEST
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

// Eye drive defines
typedef enum {
    EYE_LEFT,
    EYE_RIGHT,
    NUM_EYES,
    NUM_EYE_AXIS = (NUM_EYES*2),
} eye_idx_t;

// Axis id mapping
typedef enum {
    EYE_YAW_LEFT_AXIS = 1,
    EYE_YAW_RIGHT_AXIS,
    EYE_PITCH_LEFT_AXIS,
    EYE_PITCH_RIGHT_AXIS,
} eye_axis_t;

// Log data id
typedef enum {
    EYE_YAW_LEFT,
    EYE_PITCH_LEFT,
    EYE_YAW_RIGHT,
    EYE_PITCH_RIGHT,
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
//axis id mapping
typedef enum {
    NECK_YAW_AXIS = 1,
    NECK_PITCH_AXIS,
    NECK_ROLL_AXIS,
    NUM_NECK_AXIS = NECK_ROLL_AXIS,
} neck_drive_t;

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
} cal_states_t;

typedef struct eyeCalData {
    double time;
    double pos[NUM_EYE_AXIS];
    double err[NUM_EYE_AXIS];
    double tpos[NUM_EYE_AXIS];
    uint16_t complete[NUM_EYE_AXIS];
} eyeCalData_t;

typedef struct eyeData {
    double time;
    double yaw[NUM_EYES];
    double yaw_offset[NUM_EYES];
    double pitch[NUM_EYES];
    double pitch_offset[NUM_EYES];
    double torque[NUM_EYE_AXIS];
} eyeData_t;

typedef struct neckData {
    double time;
    double pos[NUM_NECK_AXIS];
    double torque[NUM_NECK_AXIS];
} neckData_t;

/* neck controller functions */
// Get some data
neckData_t* GetNeckData(void);
// Position Control
void DisableNeckCtrl(void);
void SetNeckPosn(uint8_t axis, double pos_rad);
void SetNeckSpeed(uint8_t axis, double speed_rps);
void SetNeckAccel(uint8_t axis, double accel_rpss);
void UpdateNeckPos();
// Torque Control
void InitNeckTorqueCtrl();
void SetNeckTorque(uint8_t axis, double torque_nm);
/* neck controller function */

/* eye controller functions */
// Get some data
eyeData_t* GetEyeData(void);
// Calibration sequence
eyeCalData_t* GetEyeCalData(void);
void StartEyeCal(uint8_t axis, double pos_m);
// Position control
void DisableEyeCtrl(void);
void SetEyePosn(uint8_t axis, double pos_m);
void SetEyeSpeed(uint8_t axis, double speed_mps);
void SetEyeAccel(uint8_t axis, double accel_mpss);
void UpdateEyePos();
// Torque control
void InitEyeForceCtrl();
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