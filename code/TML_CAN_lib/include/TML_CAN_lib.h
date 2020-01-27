#ifndef TML_CAN_LIB_H
#define TML_CAN_LIB_H

#include <stdint.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdbool.h>
#include "../../cansock_interface/cansock.h"

// Helper macros for defining filters
#define FORMAT_MASK_DEST(mask)                  ((uint32_t)(mask) << 13)
#define FILTER_ID_DEST(id)                      (((uint32_t)id*16+1) << 13)
#define FORMAT_MASK_SOURCE(mask)                ((uint32_t)(0xFFu))
#define FILTER_ID_SOURCE(id)                    ((uint32_t)id)

// Common register addresses
#define REG_CACC              (uint16_t)(0x02A2u)
#define REG_CPOS              (uint16_t)(0x029Eu)
#define REG_CSPD              (uint16_t)(0x02A0u)
#define REG_APOS              (uint16_t)(0x0228u)
#define REG_APOS2             (uint16_t)(0x081Cu)
#define REG_TPOS              (uint16_t)(0x02B2u)
#define REG_MASTERID          (uint16_t)(0x0927u)
#define REG_POSERR            (uint16_t)(0x022Au)
#define VAR_CAL_RUN           (uint16_t)(0x03B3u)
#define VAR_CAL_APOS2_OFF     (uint16_t)(0x03B0u)
#define REG_MASK            (0x77Fu)

#define VERSION_SIZE          (4)

// enum for CAN baudrates
typedef enum canBaudRate{
    BAUDRATE_125KB,
    BAUDRATE_250KB,
    BAUDRATE_500KB,
    BAUDRATE_800KB,
    BAUDRATE_1MB,
} canBaudRate_t;

// Holds the id recognized as host on the network
void SetHostId(uint8_t id);

// Function to parse response from drive
int8_t ParseTMLCAN(CAN_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr);

// Function to format msg requesting data from specific axis
void FormatGiveMeData(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, bool is16bit);//__attribute__((always_inline));

// Function to format msg requesting data from axis group
void FormatGiveMeData2(CAN_MSG* frame, uint8_t group, uint16_t reg_addr, bool is16bit);//__attribute__((always_inline));

// Function to format msg to modify 16 bit val in memory
void FormatSetVal16(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, uint16_t val);//__attribute__((always_inline));

// Function to format msg to modify 16 bit val in memory
void FormatSetVal32(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, uint32_t val);//__attribute__((always_inline));

// Function to format msg to GOTO certain place in TML prog
void FormatGoTo(CAN_MSG* frame, uint8_t id, uint16_t addr, uint8_t isGroup);//__attribute__((always_inline));

// Function to format set target posn = actual posn 
void FormatSTA(CAN_MSG* frame, uint8_t axis);//__attribute__((always_inline));

// Function to set control of Axis (on or off)
void FormatSetAxisControl(CAN_MSG* frame, uint8_t axis, bool isOn);//__attribute__((always_inline));

// Function to format set Target Update Mode msg
void FormatTUM(CAN_MSG* frame, uint8_t axis, uint8_t mode);//__attribute__((always_inline));

// Function to change motion mode
void FormatSetMotionMode(CAN_MSG* frame, uint8_t axis, uint32_t mode);//__attribute__((always_inline));

// Wrapper function for MODEPP -- same as MODEPP1 on older firmware
void FormatSetModePP(CAN_MSG* frame, uint8_t axis);//__attribute__((always_inline));

// Function to format MODEPP1 -- obsolete in new firmware
void FormatSetModePP1(CAN_MSG* frame, uint8_t axis);//__attribute__((always_inline));

// Function to format MODEPP3 -- obsolete in new firmware
void FormatSetModePP3(CAN_MSG* frame, uint8_t axis);//__attribute__((always_inline));

void FormatSetPosRef(CAN_MSG* frame, uint8_t axis, uint8_t rel);//__attribute__((always_inline));

// Function to set to relative position reference
//void FormatSetCPR(CAN_MSG* frame, uint8_t axis)__attribute__((always_inline));

// Function to set to absolute position reference
//void FormatSetCPA(CAN_MSG* frame, uint8_t axis)__attribute__((always_inline));

// Set Master Id
void FormatSetMasterId(CAN_MSG* frame, uint8_t axis, uint16_t newId);//__attribute__((always_inline));



// Following functions are un-tested**********

// Set CAN-bus baud rate
void FormatSetBaudRate(CAN_MSG* frame, canBaudRate_t rate);

// Broadcast ping message
void FormatPing(CAN_MSG* frame, uint8_t group);

// Get relevant data from pong message
// Returns false if not a pong message
int8_t ParsePong(CAN_MSG* frame, uint8_t* axis, char version[VERSION_SIZE]);

// Immediately update position with stored parameters
// For broadcast message, set as group, id = 0
void FormatUpdatePosn(CAN_MSG* frame, uint8_t id, uint8_t isGroup);

// Enable control-loop sync messages on axes
// Period sets time-between sync messages
// Set period to 0 to disable sync
void FormatSetSync(CAN_MSG* frame, uint8_t group, uint32_t period);

#endif