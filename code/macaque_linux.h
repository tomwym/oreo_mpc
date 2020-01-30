#ifndef MACAQUE_LINUX_H
#define MACAQUE_LINUX_H
#include <stdint.h>

// Configure debug modes
#define DEBUG_CAN           // Print out all rx/tx CAN messages

// Configurable settings for the library
#define SHUTDOWN_TIMEOUT_MS 	    2000
#define RX_TIMEOUT_MS		        1000
#define SEND_TIMEOUT_S		        1.0
#define BUFLEN 			            32  	
#define LOG_BUFLEN		            1024
#define MAX_ACK_PEND 		        4  //limit how much we pound the drives with polls for data
#define MAX_CMD_PEND 		        8  //must be a power of 2

// Configurable settings for the motor controllers
#define HOST_ID                     88
#define NECK_GROUP_ID		        1
#define EYE_GROUP_ID		        2

// Technosoft drive program specific addresses (Locations of specific motion profiles)
#define MOTION_LOOP_IP		        0x4022u     
#define WAIT_LOOP_IP		        0x401Bu     
#define MOTION_LOOP_NECK_IP	        0x403Bu     
#define WAIT_LOOP_NECK_IP	        0x4034u     
#define REV_CAL_IP		            0x4061u     
#define FOR_CAL_IP		            0x4027u     
#define CAL_RUN_VAR		            0x03B3u     
#define CAL_APOS2_OFF_VAR	        0x03B0u     

// Eye drive defines
typedef enum {
    LEFT,
    RIGHT,
    NUM_EYES,
    NUM_EYE_AXIS = (NUM_EYES*2),
} eye_drive_t;

// Axis id mapping
typedef enum {
    EYE_YAW_LEFT_AXIS = 129,
    EYE_PITCH_LEFT_AXIS,
    EYE_PITCH_RIGHT_AXIS,
    EYE_YAW_RIGHT_AXIS,
} eye_axis_id_t;

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
} eye_log_data_id_t;

// Neck drive defines
// Axis id mapping
typedef enum {
    NECK_YAW_AXIS = 1,
    NECK_PITCH_AXIS,
    NECK_ROLL_AXIS,
    NUM_NECK_AXIS = NECK_ROLL_AXIS,
} neck_drive_t;

// Log data id
typedef enum {
    NECK_YAW,
    NECK_PITCH,
    NECK_ROLL,
} neck_log_data_id_t;

// Calibration states
typedef enum {
    CAL_COMPLETE,
    CAL_RUNNING,
} cal_states_t;

typedef struct eyeCalData
{
    double time;
    double pos[NUM_EYE_AXIS];
    double err[NUM_EYE_AXIS];
    double tpos[NUM_EYE_AXIS];
    uint16_t complete[NUM_EYE_AXIS];
} eyeCalData_t;

typedef struct eyeData
{
    double yaw[NUM_EYES];
    double yaw_offset[NUM_EYES];
    double pitch[NUM_EYES];
    double pitch_offset[NUM_EYES];
    double time;
} eyeData_t;

typedef struct neckData
{
    double yaw;
    double pitch;
    double roll;
    double time;
} neckData_t;

// Message types supported by library
typedef enum msgType
{
    CMD_GOTO,
    CMD_GETDATA_16,
    CMD_GETDATA_32,
    CMD_GETDATA2_16,
    CMD_GETDATA2_32,
    CMD_SETDATA_16,
    CMD_SETDATA_32,
    //CMD_SETDATA_16_GROUP,
    //CMD_SETDATA_32_GROUP,
    CMD_STA,
    CMD_AXISON,
    CMD_AXISOFF,
    CMD_UPDATEMODE_0,
    CMD_UPDATEMODE_1,
    CMD_MOTIONMODE_PP,
    CMD_MOTIONMODE_PP1,
    CMD_MOTIONMODE_PP3,
    CMD_SET_POSABS,
    CMD_SET_POSREL,
    CMD_SET_MASTERID,
    NUM_CMDS,
} msgType_t;



// Functions to interface with neck controller
neckData_t* getNeckData(void);
void SetNeckPollData(void);
void ResetNeckPollData(void);
void SetNeckPosn(uint8_t axis, double pos_rad);
void SetNeckSpeed(uint8_t axis, double speed_rps);
void SetNeckAccel(uint8_t axis, double accel_rpss);


// Functions to interface with eye controller
eyeData_t* GetEyeData(void);
eyeCalData_t* GetEyeCalData(void);
void SetEyePollData(void);
void ResetEyePollData(void);
void SetEyePosn(uint8_t axis, double pos_m);
void SetEyeSpeed(uint8_t axis, double speed_mps);
void SetEyeAccel(uint8_t axis, double accel_mpss);
void StartEyeCal(uint8_t axis, double pos_m);

// Constructors/Destructor for the library
void Start(void);
void Cleanup(void);

// Wrapper functions for communication with motors
void SendMotorCommandEye(msgType_t msgType, uint8_t id, uint8_t group);
void SendMotorCommandNeck(msgType_t msgType, uint8_t id, uint8_t group);
void GetDataMemEye(msgType_t msgType, uint8_t id, uint16_t regAddr);
void GetDataMemNeck(msgType_t msgType, uint8_t id, uint16_t regAddr);
void SetDataMemEye(msgType_t msgType, uint8_t axis_id, uint16_t regAddr, uint32_t data);
void SetDataMemNeck(msgType_t msgType, uint8_t axis_id, uint16_t regAddr, uint32_t data);


#endif