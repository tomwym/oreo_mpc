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
#define MOTION_LOOP_IP		0x4022u     // Motion(eye) loop
#define WAIT_LOOP_IP		0x401Bu     // Wait(eye) loop
#define MOTION_LOOP_NECK_IP	0x403Bu     // Motion(neck) loop
#define WAIT_LOOP_NECK_IP	0x4034u     // Wait(neck) loop
#define REV_CAL_IP		    0x4061u     // Reverse calibration
#define FOR_CAL_IP		    0x4027u     // Forward calibration

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

// Holds the id recognized as host on the network
void InitLib(uint8_t id, serial_baudrate_t rate);

// Function to parse response from drive
int8_t ParseResponse(RS232_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr);

// Function to format msg requesting data from specific axis
void FormatGiveMeData(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, bool is16bit);//__attribute__((always_inline));

// Function to format msg requesting data from axis group
void FormatGiveMeData2(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, bool is16bit);//__attribute__((always_inline));

// Function to format msg to modify 16 bit val in memory
void FormatSetVal16(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, uint16_t val);//__attribute__((always_inline));

// Helper to transform float to fixed point representation
uint32_t DoubleToFixed(double num);

// Hlper to transform fixed to float
double FixedToDouble(uint32_t num);

// Function to format msg to modify 16 bit val in memory
void FormatSetVal32(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, uint32_t val);//__attribute__((always_inline));

// Function to format msg to GOTO certain place in TML prog
void FormatGoTo(RS232_MSG* frame, motor_id_t* dest, uint16_t addr);//__attribute__((always_inline));

// Function to format set target posn = actual posn 
void FormatSTA(RS232_MSG* frame, motor_id_t* dest);//__attribute__((always_inline));

// Function to set control of Axis (on or off)
void FormatSetAxisControl(RS232_MSG* frame, motor_id_t* dest, bool turnOn);//__attribute__((always_inline));

// Function to format set Target Update Mode msg
void FormatTUM(RS232_MSG* frame, motor_id_t* dest, uint8_t mode);//__attribute__((always_inline));

// Function to format msg to change motion mode
void FormatSetMotionMode(RS232_MSG* frame, motor_id_t* dest, uint32_t mode);

// Wrapper function for MODEPP -- same as MODEPP1 on older firmware
void FormatSetModePP(RS232_MSG* frame, motor_id_t* dest);//__attribute__((always_inline));

// Function to format MODEPP1 -- obsolete in new firmware
void FormatSetModePP1(RS232_MSG* frame, motor_id_t* dest);//__attribute__((always_inline));

// Function to format MODEPP3 -- obsolete in new firmware
void FormatSetModePP3(RS232_MSG* frame, motor_id_t* dest);//__attribute__((always_inline));

// Function to set position reference mode
// Set rel to 1 for relative position reference
// Set rel to 0 for absolute position reference
void FormatSetPosRef(RS232_MSG* frame, motor_id_t* dest, uint8_t rel);//__attribute__((always_inline));

// Wrapper for setting relative position reference
void FormatSetCPR(RS232_MSG* frame, motor_id_t* dest);

// Wrapper for setting absolute position reference
void FormatSetCPA(RS232_MSG* frame, motor_id_t* dest);

// Set Master Id
void FormatSetMasterId(RS232_MSG* frame, motor_id_t* dest, uint16_t newId);//__attribute__((always_inline));

// Set serial baud rate
void FormatSetBaudRate(RS232_MSG* frame, serial_baudrate_t rate);

// Broadcast ping message
void FormatPing(RS232_MSG* frame, motor_id_t* dest);

// Get relevant data from pong message
// Returns false if not a pong message
int8_t ParsePong(RS232_MSG* frame, uint8_t* axis, char version[VERSION_SIZE]);

// Immediately update position with stored parameters
// For broadcast message, set as group, id = 0
void FormatUpdatePosn(RS232_MSG* frame, motor_id_t* dest);

// Enable control-loop sync messages on axes
// Period sets time-between sync messages
// Set period to 0 to disable sync
void FormatSetSync(RS232_MSG* frame, motor_id_t* id);

// Set external reference mode to analog (use analog input)
void SetExtRefAnalog(RS232_MSG* frame, motor_id_t* dest);

// Set external reference mode to digital (use digital input)
void SetExtRefDigital(RS232_MSG* frame, motor_id_t* dest);

// Set external reference mode to online (send messages from host)
void SetExtRefOnline(RS232_MSG* frame, motor_id_t* dest);

// Set external reference mode
void SetExtRefMode(RS232_MSG* frame, motor_id_t* dest, uint32_t mode);

#endif