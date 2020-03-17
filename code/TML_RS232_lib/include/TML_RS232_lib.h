#ifndef TML_RS232_LIB_H
#define TML_RS232_LIB_H
#include <stdint.h>
#include <stdbool.h>
#include "../../sock_interface/sock.h"

// Common register addresses
#define REG_CACC              (uint16_t)(0x02A2u)   // fixed (32bits)
#define REG_CPOS              (uint16_t)(0x029Eu)   // long
#define REG_CSPD              (uint16_t)(0x02A0u)   // fixed (32bits)
#define REG_APOS              (uint16_t)(0x0228u)   // long
#define REG_APOS2             (uint16_t)(0x081Cu)   // long
#define REG_TPOS              (uint16_t)(0x02B2u)   // long
#define REG_MASTERID          (uint16_t)(0x0927u)   // int
#define REG_POSERR            (uint16_t)(0x022Au)   // int
#define REG_EREFT             (uint16_t)(0x02A9u)   // int
#define REG_IQREF             (uint16_t)(0x022Fu)   // int
#define REG_IQ                (uint16_t)(0x0230u)   // int
#define VAR_CAL_RUN           (uint16_t)(0x03B3u)   
#define VAR_CAL_APOS2_OFF     (uint16_t)(0x03B0u)
#define REG_MASK              (uint16_t)(0x77Fu)


// Common instruction pointer addresses
#define POSN_LOOP_EYE_IP		0x4025u     // Motion(eye) loop
#define WAIT_LOOP_EYE_IP		0x401Eu     // Wait(eye) loop
#define FORCE_LOOP_EYE_IP       0x402Au     // Torque control loop
#define POSN_LOOP_NECK_IP	    0x403Bu     // Motion(neck) loop
#define WAIT_LOOP_NECK_IP	    0x4034u     // Wait(neck) loop
#define TORQUE_LOOP_NECK_IP     0xFFFFu     // Torque control loop
#define REV_CAL_EYE_IP          0x4069u     // Reverse calibration
#define FOR_CAL_EYE_IP	        0x402Fu     // Forward calibration

// type of id
typedef enum {
    ID_TYPE_AXIS,
    ID_TYPE_GROUP,
    ID_TYPE_BROADCAST,
} id_type_t;

// message dest id
typedef struct {
    id_type_t type;
    uint8_t id;
} motor_id_t;

#define VERSION_SIZE          (4)

// enum for CAN baudrates
typedef enum {
    BAUDRATE_9600,
    BAUDRATE_19200,
    BAUDRATE_38400,
    BAUDRATE_56600,
    BAUDRATE_115200,
} serial_baudrate_t;

// enum for external reference types
typedef enum {
    EXTREF_ONLINE,
    EXTREF_ANALOG,
    EXTREF_DIGITAL,
} ext_ref_t;

// enum to specify which device to write to
typedef enum {
    DEV_EYE,
    DEV_NECK,
} dest_dev_t;

// Holds the id recognized as host on the network
void InitLib(uint8_t id, serial_baudrate_t rate);

// Function to parse response from drive
int8_t ParseResponse(RS232_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr);

// Function to format msg requesting data from specific axis
void SendGiveMeData(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, bool is16bit);

// Function to format msg requesting data from axis group
void SendGiveMeData2(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, bool is16bit);

// Function to format msg to GOTO certain place in TML prog
void SendGoTo(dest_dev_t dev, motor_id_t* dest, uint16_t addr);

// Function to format msg to modify 16 bit val in memory
void SetVal16(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, uint16_t val);

// Helper to transform float to fixed point representation
uint32_t DoubleToFixed(double num);

// Hlper to transform fixed to float
double FixedToDouble(uint32_t num);

// Function to format msg to modify 16 bit val in memory
void SetVal32(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, uint32_t val);

// Function to format set target posn = actual posn 
void SendSTA(dest_dev_t dev, motor_id_t* dest);

// Function to set control of Axis (on or off)
void SetAxisControl(dest_dev_t dev, motor_id_t* dest, bool turnOn);

// Function to format set Target Update Mode msg
void SetTUM(dest_dev_t dev, motor_id_t* dest, uint8_t mode);

// Wrapper function for MODEPP -- same as MODEPP1 on older firmware
void SetModePP(dest_dev_t dev, motor_id_t* dest);

// Function to format MODEPP1 -- obsolete in new firmware
void SetModePP1(dest_dev_t dev, motor_id_t* dest);

// Function to format MODEPP3 -- obsolete in new firmware
void SetModePP3(dest_dev_t dev, motor_id_t* dest);

// Wrapper function for MODE_TES -- torque slow loop
void SetModeTorqueSlow(dest_dev_t dev, motor_id_t* dest);

// Wrapper function for MODE_TEF -- torque fast loop
void SetModeTorqueFast(dest_dev_t dev, motor_id_t* dest);

// Wrapper for setting relative position reference
void SetCPR(dest_dev_t dev, motor_id_t* dest);

// Wrapper for setting absolute position reference
void SetCPA(dest_dev_t dev, motor_id_t* dest);

// Set Master Id
void SetMasterId(dest_dev_t dev, motor_id_t* dest, uint16_t newId);

// Broadcast ping message
void SendPing(dest_dev_t dev, motor_id_t* dest);

// Get relevant data from pong message
// Returns false if not a pong message
int8_t ParsePong(RS232_MSG* frame, uint8_t* axis, char version[VERSION_SIZE]);

// Immediately update position with stored parameters
// For broadcast message, set as group, id = 0
void UpdatePosn(dest_dev_t dev, motor_id_t* dest);

// Enable control-loop sync messages on axes
// Period sets time-between sync messages
// Set period to 0 to disable sync
void SetSync(dest_dev_t dev, motor_id_t* id);

// Set serial baud rate
void SetBaudRate(dest_dev_t dev, serial_baudrate_t rate);

// Set external reference mode
void SetExtRefMode(dest_dev_t dev, motor_id_t* dest, uint32_t mode);

// Set external reference mode to online (send messages from host)
void SetExtRefOnline(dest_dev_t dev, motor_id_t* dest);

// Set external reference mode to analog (use analog input)
void SetExtRefAnalog(dest_dev_t dev, motor_id_t* dest);

// Set external reference mode to digital (use digital input)
void SetExtRefDigital(dest_dev_t dev, motor_id_t* dest);
#endif