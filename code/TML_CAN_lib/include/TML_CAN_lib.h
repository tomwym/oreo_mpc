#ifndef TML_CAN_LIB_H
#define TML_CAN_LIB_H

#include <stdint.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdbool.h>
#include "../../cansock_interface/cansock.h"

// Function to parse response from drive
int8_t ParseTMLCAN(CAN_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr);

// Function to format msg requesting data from specific axis
void FormatGiveMeData(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, bool is16bit);

// Function to format msg requesting data from axis group
void FormatGiveMeData2(CAN_MSG* frame, uint8_t group, uint16_t reg_addr, bool is16bit);

// Function to format msg to GOTO certain place in TML prog
void FormatGoTo(CAN_MSG* frame, uint8_t axis, uint16_t addr);

// Function to format msg to modify 16 bit val in memory
void FormatSetVal16(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, uint16_t val);

// Function to format msg to modify 16 bit val in memory
void FormatSetVal32(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, uint32_t val);

// Function to format set target posn = actual posn 
void FormatSTA(CAN_MSG* frame, uint8_t axis);

// Function to set control of Axis (on or off)
void FormatSetAxisControl(CAN_MSG* frame, uint8_t axis, bool isOn);

// Function to format set Target Update Mode msg
void FormatTUM(CAN_MSG* frame, uint8_t axis, bool isMode0);

// Function to change motion mode
void FormatSetMotionMode(CAN_MSG* frame, uint8_t axis, uint32_t mode);

// Wrapper function for MODEPP -- same as MODEPP1 on older firmware
void FormatSetModePP(CAN_MSG* frame, uint8_t axis);

// Function to format MODEPP1 -- obsolete in new firmware
void FormatSetModePP1(CAN_MSG* frame, uint8_t axis);

// Function to format MODEPP3 -- obsolete in new firmware
void FormatSetModePP3(CAN_MSG* frame, uint8_t axis);

// Function to set to relative position reference
void FormatSetCPR(CAN_MSG* frame, uint8_t axis);

// Function to set to absolute position reference
void FormatSetCPA(CAN_MSG* frame, uint8_t axis);

// Set Master Id
void FormatSetMasterId(CAN_MSG* frame, uint8_t axis, uint16_t newId);

#endif