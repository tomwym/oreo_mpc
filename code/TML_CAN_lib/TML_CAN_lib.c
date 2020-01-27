/*
TODO:
- add some error checking to GiveMeData messages to ensure asking for correct size
- fix group id checking (Goto and GiveMeData2 messages)
- Add support for sending group/broadcast commands
- Add support for setting data in memory types other than RAM
- Will eventually need to add functionality to setvar16 and setvar32 to support full register addressing (see p. 456 of manual)
- Need to add error checking on the inputs to the functions
    - axis 1-255
    - group is 1-8
    - reg addr is between 0x0200 - 0x03ff and 0x0800 - 0x08ff
- Add new functionality
    - Read Communication Error Register (6.3.4.5 of manual)
    - Read Motion Error Register (6.3.4.10 of manual)
    - Perhaps need to modify System Configuration Register
*/

#include "./include/TML_CAN_lib.h"
#include "./include/TML_CAN_def.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

static uint8_t hostId = 0;

// Set var which holds id recognized as host on the network
void SetHostId(uint8_t id)
{
    hostId = id;
}

// Function to parse response from drive
// Calls the type-specific parse functions to get data
int8_t ParseTMLCAN(CAN_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr)
{
    uint32_t frameId = frame->identifier;

    // Get the optcode from can_id (first 7 bits and last 9 bits)
    uint16_t optCode = OPT_CODE(frameId);

    // Check message recipient
    uint8_t id = DEST_ID(frameId);
    if(id != hostId) {
        printf("Message not destined for host (%x). Discarded\n", id);
        return -1;
    }

    // Can check host bit to differentiate between Type B and Type C messages
    //uint8_t isTypeC = (HOST_BIT(frameId) > 0) ? 1 : 0;

    // Sort based on type of message    
    // Take Data 1 message
    if((HIBYTE(optCode) == TAKE_DATA)) {
        // The first two bytes of payload is source axisId
        *axis = MIDBYTE(frame->CAN_data[0], frame->CAN_data[1]);
        
        // LSB determines size of data returned
        if(LOBYTE(optCode) & 0x1) {
            // Check that frame size matches expected
            if(frame->length != TAKE_32BIT_SIZE) {
                printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE_32BIT_SIZE, frame->length);
                return -1;
            }
            
            // Parse payload
            *data = ((uint32_t)(((uint16_t)frame->CAN_data[7]) << 8 | frame->CAN_data[6]) << 16) | ((uint16_t)frame->CAN_data[5]) << 8 | frame->CAN_data[4];
            *reg_addr = ((uint16_t)(frame->CAN_data[0])|(frame->CAN_data[1] << 8));
        } else {
            // Check that frame size matches expected
            if(frame->length != TAKE_16BIT_SIZE) {
                printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE_16BIT_SIZE, frame->length);
                return -1;
            }
            
            // Parse payload
            *data = ((uint16_t)(frame->CAN_data[5]) << 8 | frame->CAN_data[4]);
            *reg_addr = ((uint16_t)(frame->CAN_data[2])|(frame->CAN_data[3] << 8));
        }
    } else if(HIBYTE(optCode) == TAKE_DATA2_16) {
        // 2 bytes after Group Bit is axisId
        *axis = (uint8_t)(HIBYTE(optCode) & 0xFF);
        
        if(frame->length != TAKE2_16BIT_SIZE) {
            printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE2_16BIT_SIZE, frame->length);
            return -1;
        }

        // 16 bits returned
        *data = ((uint16_t)(frame->CAN_data[2])|(frame->CAN_data[3] << 8));
        *reg_addr = ((uint16_t)(frame->CAN_data[0])|(frame->CAN_data[1] << 8));
    } else if(HIBYTE(optCode) == TAKE_DATA2_32) {
        // 2 bytes after Group Bit is axisId
        *axis = (uint8_t)(HIBYTE(optCode) & 0xFF);

        if(frame->length != TAKE2_32BIT_SIZE) {
            printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE2_32BIT_SIZE, frame->length);
            return -1;
        }
        
        // 32 bits returned
        *data = ((uint32_t)(((uint16_t)frame->CAN_data[5]) << 8 | frame->CAN_data[4]) << 16) | ((uint16_t)frame->CAN_data[3]) << 8 | frame->CAN_data[2];
        *reg_addr = ((uint16_t)(frame->CAN_data[0])|(frame->CAN_data[1] << 8));
    } else {
        printf("Recvd CAN message with unexpected optCode (%x)\n", optCode);
        return -1;
    }

    return 0;    
}

// Ask specific axis for data from memory
inline void FormatGiveMeData(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, bool is16bit)
{
    // Current only support reading from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA | TM_DATA;

    if(!is16bit)
        optCode |= 0x1;

    // Make full 16 bit ID code
    uint16_t idCode = AXIS_ID_CODE(axis);

    // Can Id construction
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // Payload
    uint16_t masterId = HOST_TO_MASTER(hostId);
    frame->CAN_data[0] = LOBYTE(masterId);
    frame->CAN_data[1] = HIBYTE(masterId); 
    frame->CAN_data[2] = LOBYTE(reg_addr);
    frame->CAN_data[3] = HIBYTE(reg_addr);
    frame->length = sizeof(masterId) + sizeof(reg_addr);
    frame->type = EXTENDED_FRAME;

    return;
}

// Function to format msg requesting data from axis group
inline void FormatGiveMeData2(CAN_MSG* frame, uint8_t group, uint16_t reg_addr, bool is16bit)
{
    // Group id has max of 8
    if(group > MAX_GROUP_ID) {
        printf("Max group id exceeded. Setting to max\n");
        group = MAX_GROUP_ID;
    }
    
    // Currently only support reading from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA2 | TM_DATA;

    if(!is16bit)
        optCode |= 0x1u;

    // Make full 16 bit ID code
    uint16_t idCode = GROUP_ID_CODE(group);

    // Can Id construction
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // Payload
    uint16_t masterId = HOST_TO_MASTER(hostId);
    frame->CAN_data[0] = LOBYTE(masterId);
    frame->CAN_data[1] = HIBYTE(masterId); 
    frame->CAN_data[2] = LOBYTE(reg_addr);
    frame->CAN_data[3] = HIBYTE(reg_addr);
    frame->length = sizeof(masterId) + sizeof(reg_addr);
    frame->type = EXTENDED_FRAME;

    return;
}

// Function to format msg to GOTO certain place in TML prog
void FormatGoTo(CAN_MSG* frame, uint8_t id, uint16_t addr, uint8_t isGroup)
{
    uint16_t optCode = OPT_GOTO_LABEL;
    if(isGroup > 0) {
        isGroup = 1;
        if(id > MAX_GROUP_ID) {
            printf("Max group id exceeded. Setting to max\n");
            id = MAX_GROUP_ID;
        }
    }
        
    uint16_t idCode;
    if(isGroup) {
        idCode = GROUP_ID_CODE(id);
    } else {
        idCode = AXIS_ID_CODE(id);
    }
    
    frame->identifier = TML_CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE(addr);
    frame->CAN_data[1] = HIBYTE(addr);
    frame->length = sizeof(addr);
    frame->type = EXTENDED_FRAME;

    return;
}

// Currently only supporting setting values for addresses in RAM
// Function to format msg to modify 16 bit val in memory
void FormatSetVal16(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, uint16_t val)
{
    // Optcode changes based on address
    uint16_t optCode = OPT_SET16_200;
    if(reg_addr >= SET16_ADDR_LIMIT) {
        optCode = OPT_SET16_800;
    }
    optCode |= (LSB9_MASK & reg_addr);
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE(val);
    frame->CAN_data[1] = HIBYTE(val);
    /* Alternate way
        uint8_t* ptr = (uint8_t*)&val;
    for(int i = 0; i < sizeof(val); i++) {
      frame->CAN_data[i] = ptr[i];
    }
    */
    frame->length = sizeof(val);
    frame->type = EXTENDED_FRAME;

    return;
}

// Currently only supporting setting values for addresses in RAM
// Function to format msg to modify 32 bit val in memory
void FormatSetVal32(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, uint32_t val)
{
    // OptCode changes based on address
    uint16_t optCode = OPT_SET32_200;
    if(reg_addr >= SET32_ADDR_LIMIT) {
        optCode = OPT_SET32_800;
    }
    optCode |= (LSB9_MASK & reg_addr);
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE((uint16_t)val);
    frame->CAN_data[1] = HIBYTE((uint16_t)val);
    frame->CAN_data[2] = LOBYTE((uint16_t)(val >> 16));
    frame->CAN_data[3] = HIBYTE((uint16_t)(val >> 16));
    /* Alternate way
        uint8_t* ptr = (uint8_t*)&val;
    for(int i = 0; i < sizeof(val); i++) {
      frame->CAN_data[i] = ptr[i];
    }
    */

    frame->length = sizeof(val);
    frame->type = EXTENDED_FRAME;

    return;
}

// Function to format set target posn = actual posn 
void FormatSTA(CAN_MSG* frame, uint8_t axis)
{
    uint16_t optCode = OPT_STA;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE(STA_PAYLOAD);
    frame->CAN_data[1] = HIBYTE(STA_PAYLOAD);
    frame->length = sizeof(STA_PAYLOAD);
    frame->type = EXTENDED_FRAME;

    return;
}

// Function to set control of axis (on or off)
void FormatSetAxisControl(CAN_MSG* frame, uint8_t axis, bool isOn)
{
    uint16_t optCode = (isOn == true) ? OPT_AXISON : OPT_AXISOFF;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // No payload
    frame->length = 0;
    frame->type = EXTENDED_FRAME;
}

// Function to format set Target Update Mode 0 msg
void FormatTUM(CAN_MSG* frame, uint8_t axis, uint8_t mode)
{
    uint16_t optCode = OPT_TUM;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // Payload depends on the mode
    uint32_t payload;
    if(mode == 0) {
        payload = TUM0_PAYLOAD; 
    } else {
        payload = TUM1_PAYLOAD;
    }
    uint8_t* ptr = (uint8_t*)(&payload);
    for(uint8_t i = 0; i < sizeof(payload); i++) {
        frame->CAN_data[i] = ptr[i];
    }
    frame->length = sizeof(payload);
    frame->type = EXTENDED_FRAME;

    return;
}

// Function to format msg to set motion mode
void FormatSetMotionMode(CAN_MSG* frame, uint8_t axis, uint32_t mode)
{
    uint16_t optCode = OPT_MODE;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    uint8_t* ptr = (uint8_t*)(&mode);
    for(int i = 0; i < sizeof(mode); i++) {
        frame->CAN_data[i] = ptr[i];
    }
    frame->length = sizeof(mode);
    frame->type = EXTENDED_FRAME;
}

// Wrapper function for MODEPP
void FormatSetModePP(CAN_MSG* frame, uint8_t axis)
{
    FormatSetMotionMode(frame, axis, MODE_PP_PAYLOAD);
}

// Wrapper function for MODEPP1 -- obsolete in new firmware
void FormatSetModePP1(CAN_MSG* frame, uint8_t axis)
{
    FormatSetMotionMode(frame, axis, MODE_PP1_PAYLOAD);
}

// Wrapper function for MODEPP3 -- obsolete in new firmware
void FormatSetModePP3(CAN_MSG* frame, uint8_t axis)
{
    FormatSetMotionMode(frame, axis, MODE_PP3_PAYLOAD);
}

// Function to set position reference mode
// Set rel to 1 for relative position reference
// Set rel to 0 for absolute position reference
void FormatSetPosRef(CAN_MSG* frame, uint8_t axis, uint8_t rel)
{
    uint32_t payload;
    uint16_t optCode = OPT_POSREF;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);
    if(rel > 0) {
        payload = CPR_PAYLOAD;
    } else {
        payload = CPA_PAYLOAD;
    }
    uint8_t* ptr = (uint8_t*)(&payload);
    for(int i = 0; i < sizeof(payload); i++) {
        frame->CAN_data[i] = ptr[i];
    }
    frame->length = sizeof(payload);
    frame->type = EXTENDED_FRAME;
}

// Function to set to relative position reference
/*void FormatSetCPR(CAN_MSG* frame, uint8_t axis)
{
    uint16_t optCode = OPT_POSREF;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    for(int i = 0; i < sizeof(CPR_PAYLOAD); i++) {
        frame->CAN_data[i] = (uint8_t)(CPR_PAYLOAD >> i);
    }
    frame->length = sizeof(CPR_PAYLOAD);
    frame->type = EXTENDED_FRAME;
}*/

// Function to set to absolute position reference
/*void FormatSetCPA(CAN_MSG* frame, uint8_t axis)
{
    uint16_t optCode = OPT_POSREF;
    uint16_t idCode = AXIS_ID_CODE(axis);
    frame->identifier = TML_CAN_ID(optCode, idCode);

    for(int i = 0; i < sizeof(CPA_PAYLOAD); i++) {
        frame->CAN_data[i] = (uint8_t)(CPA_PAYLOAD >> i);
    }
    frame->length = sizeof(CPA_PAYLOAD);
    frame->type = EXTENDED_FRAME;
}*/

// Set the masterid for axis
void FormatSetMasterId(CAN_MSG* frame, uint8_t axis, uint16_t newId)
{
    hostId = newId;
    FormatSetVal16(frame, axis, REG_MASTERID, HOST_TO_MASTER(newId));
}

// Set CAN-bus baud rate
void FormatSetBaudRate(CAN_MSG* frame, canBaudRate_t rate)
{
    // Setting CAN Bus baud rate requires a broadcast message
    uint16_t idCode = BROADCAST_ID_CODE;
    uint16_t optCode = OPT_CANBR;
    frame->identifier = TML_CAN_ID(optCode, idCode);

    uint16_t payload;
    switch(rate) {
        case BAUDRATE_125KB:
            payload = CAN_BAUDRATE_125KB;
            break;
        case BAUDRATE_250KB:
            payload = CAN_BAUDRATE_250KB;
            break;
        case BAUDRATE_500KB:
            payload = CAN_BAUDRATE_500KB;
            break;
        case BAUDRATE_800KB:
            payload = CAN_BAUDRATE_800KB;
            break;
        case BAUDRATE_1MB:
            payload = CAN_BAUDRATE_1MB;
            break;
        default:
            printf("Tried to set baudrate to invalid value. Using default\n");
            payload = CAN_BAUDRATE_500KB;
            break;
    }

    // Payload is the hexcode corresponding to rate
    frame->CAN_data[0] = LOBYTE(payload);
    frame->CAN_data[1] = HIBYTE(payload);
    frame->length = sizeof(payload);
    frame->type = EXTENDED_FRAME;
}

void FormatPing(CAN_MSG* frame, uint8_t group)
{
    // Send ping message to group
    uint16_t idCode = GROUP_ID_CODE(group);
    uint16_t optCode = OPT_PING;
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // Payload master id and latency (0s for CAN)
    uint16_t masterId = HOST_TO_MASTER(hostId);
    uint16_t latency = 0x0000;
    frame->CAN_data[0] = LOBYTE(masterId);
    frame->CAN_data[1] = HIBYTE(masterId);
    frame->CAN_data[2] = LOBYTE(latency);
    frame->CAN_data[3] = LOBYTE(latency);
    frame->length = sizeof(masterId) + sizeof(latency);
    frame->type = EXTENDED_FRAME;
}

int8_t ParsePong(CAN_MSG* frame, uint8_t* axis, char version[VERSION_SIZE])
{
    uint32_t frameId = frame->identifier;

    // Get the optcode from can_id (first 7 bits and last 9 bits)
    uint16_t optCode = OPT_CODE(frameId);
    if(HIBYTE(optCode) != HIBYTE(OPT_PONG)) {
        printf("Received frame was not pong response\n");
        return -1;
    }

    // Check message recipient
    uint8_t id = DEST_ID(frameId);
    if(id != hostId) {
        printf("Message not destined for host (%x). Discarded\n", id);
        return -1;
    }    

    // Get axis id from optcode and firmware version from payload
    // Sanity check to ensure expected payload size
    if(frame->length != VERSION_SIZE) {
        printf("Unexpected pong payload size\n");
        return -1;
    }
    *axis = LOBYTE(optCode);
    memcpy(version, frame->CAN_data, frame->length);

    return 0;
}

void FormatUpdatePosn(CAN_MSG* frame, uint8_t id, uint8_t isGroup)
{
    // Send ping message to group
    uint16_t idCode;
    if(isGroup) {
        if(id == 0) {
            idCode = BROADCAST_ID_CODE; 
        } else {
            idCode = GROUP_ID_CODE(id);
        }
    } else {
        idCode = AXIS_ID_CODE(id);
    }
    uint16_t optCode = OPT_UPD;
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // No payload
    frame->length = 0;
    frame->type = EXTENDED_FRAME;
}

// Start synchroization messages between motors
void FormatSetSync(CAN_MSG* frame, uint8_t group, uint32_t period)
{
    uint16_t idCode;
    if(group == 0) {
        idCode = BROADCAST_ID_CODE;
    } else {
        idCode = GROUP_ID_CODE(group);
    }

    uint16_t optCode = OPT_SETSYNC;
    frame->identifier = TML_CAN_ID(optCode, idCode);

    // Payload
    frame->CAN_data[0] = LOBYTE((uint16_t)period);
    frame->CAN_data[1] = HIBYTE((uint16_t)period);
    frame->CAN_data[2] = LOBYTE((uint16_t)(period >> 16));
    frame->CAN_data[3] = HIBYTE((uint16_t)(period >> 16));
    frame->length = sizeof(period);
    frame->type = EXTENDED_FRAME;
}

