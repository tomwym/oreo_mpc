#include "./include/TML_CAN_lib.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>

// Parsing information from Can ID
#define LOW_OPTCODE_MASK    (0x1FF)
#define OPT_CODE(x) ((uint16_t)(x >> 13))|(x & LOW_OPTCODE_MASK)
#define DEST_ID(x) (uint8_t)(x >> 13)
#define IS_GROUP(x) (uint8_t)((x >> 21) & 0x1u)

// Build message
#define ID_CODE(grp, id)      (((uint16_t)id << 4)|(grp<<12))
#define CAN_ID(opt, id)             ((((uint32_t)(opt & 0xFE00u)) << 13) | ((uint32_t)id << 9) | (opt & 0x01FFu))
#define MASTER_ID(id)           ((uint16_t)id*16+1)

// Common helpers
#undef LOBYTE
#define LOBYTE(x)    ((uint8_t)(x))
#undef HIBYTE
#define HIBYTE(x)    ((uint8_t)((uint16_t)(x) >> 8))
#define LSB9_MASK   0x01FFu
#define HOST_TO_MASTER(id)  ((uint16_t)(id)*16+1)

// Receive Message Types
#define TAKE_DATA       (0xB4)                // TakeData1 message High Byte
#define TAKE_DATA2_16   (0xD4)                // TakeData2 (16 bits) message High Byte
#define TAKE_DATA2_32   (0xD5)                // TakeData2 (16 bits) message High Byte

// Expected Size of data
#define TAKE_32BIT_SIZE      (8)
#define TAKE_16BIT_SIZE      (6)
#define TAKE2_32BIT_SIZE     (6)
#define TAKE2_16BIT_SIZE     (4)

// Send Message OptCodes
#define OPT_GIVE_ME_DATA     (uint16_t)(0xB000u)
#define OPT_GIVE_ME_DATA2    (uint16_t)(0xB200u)
#define OPT_GOTO_LABEL       (uint16_t)(0x7400u)
#define OPT_SET16_200        (uint16_t)(0x2000u)
#define OPT_SET16_800        (uint16_t)(0x2200u)
#define SET16_ADDR_LIMIT     (uint16_t)(0x0800u)      // Address limit at which opt code changes
#define OPT_SET32_200        (uint16_t)(0x2400u)
#define OPT_SET32_800        (uint16_t)(0x2600u)
#define SET32_ADDR_LIMIT     (uint16_t)(0x0800u)      // Address limit at which opt code changes
#define OPT_STA              (uint16_t)(0x2CB2u)
#define STA_PAYLOAD          (uint16_t)(0x0228u)
#define OPT_AXISON           (uint16_t)(0x0102u)
#define OPT_AXISOFF          (uint16_t)(0x0002u)
#define OPT_TUM              (uint16_t)(0x5909u)
#define TUM0_PAYLOAD         (uint32_t)(0xBFFF0000u)
#define TUM1_PAYLOAD         (uint32_t)(0xFFFF4000u)
#define OPT_MODE             (uint16_t)(0x5909u)
#define OPT_CPR              (uint16_t)(0x5909u)
#define CPR_PAYLOAD          (uint16_t)(0xDFFF0000u)
#define OPT_CPA              (uint16_t)(0x5909u)
#define CPA_PAYLOAD          (uint16_t)(0xFFFF2000u)

// Motion Modes
#define MODE_PP_PAYLOAD      (uint32_t)(0xBFC18701u)
#define MODE_PP1_PAYLOAD     MODE_PP_PAYLOAD
#define MODE_PP3_PAYLOAD     (uint32_t)(0xBDC1850u)

// Memory Types
#define TM_DATA              (0x4u)

// Common register addresses
#define REG_CACC              (uint16_t)(0x02A2u)
#define REG_CPOS              (uint16_t)(0x029Eu)
#define REG_CSPD              (uint16_t)(0x02A0u)
#define REG_APOS              (uint16_t)(0x0228u)
#define REG_APOS2             (uint16_t)(0x081Cu)
#define REG_ADDR_TPOS         (uint16_t)(0x02B2u)
#define REG_MASTERID          (uint16_t)(0x0927u)


// Common instruction pointer addresses
#define MOTION_LOOP_IP		0x4022u     // Motion(eye) loop
#define WAIT_LOOP_IP		0x401Bu     // Wait(eye) loop
#define MOTION_LOOP_NECK_IP	0x403Bu     // Motion(neck) loop
#define WAIT_LOOP_NECK_IP	0x4034u     // Wait(neck) loop
#define REV_CAL_IP		    0x4061u     // Reverse calibration
#define FOR_CAL_IP		    0x4027u     // Forward calibration
#define CAL_RUN_VAR		    0x03B3u     // Calibration has been run
#define CAL_APOS2_OFF_VAR	0x03B0u     // 

static uint8_t hostId = 0;

// Function to parse response from drive
// Calls the type-specific parse functions to get data
int8_t ParseTMLCAN(CAN_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr)
{
    uint32_t frameId = frame->identifier;

    // Get the optcode from can_id (first 7 bits and last 9 bits)
    uint16_t optCode = OPT_CODE(frameId);

    // Check message recipient
    //bool isGroupDest = IS_GROUP(id);
    uint8_t id = DEST_ID(frameId);
    if(id != hostId) {
        printf("Message not destined for host (%x). Discarded\n", id);
        return -1;
    }
        
    // Sort based on type of message    
    // Take Data 1 message
    if((HIBYTE(optCode) == TAKE_DATA)) {
        // The first two bytes of payload is axisId
        *axis = ((uint16_t)(frame->CAN_data[0]) | (frame->CAN_data[1] << 8));
        
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
        
        // 16 bits returned
        *data = ((uint16_t)(frame->CAN_data[2])|(frame->CAN_data[3] << 8));
        *reg_addr = ((uint16_t)(frame->CAN_data[0])|(frame->CAN_data[1] << 8));
    } else if(HIBYTE(optCode) == TAKE_DATA2_32) {
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
void FormatGiveMeData(CAN_MSG* frame, uint8_t axis, uint16_t reg_addr, bool is16bit)
{
    // Current only support reading from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA | TM_DATA;

    if(!is16bit)
        optCode |= 0x1;

    // Make full 16 bit ID code
    uint16_t idCode = ID_CODE(0, axis);

    // Can Id construction
    frame->identifier = CAN_ID(optCode, idCode);

    // Payload
    uint16_t masterId = MASTER_ID(hostId);
    frame->CAN_data[0] = LOBYTE(masterId);
    frame->CAN_data[1] = HIBYTE(masterId); 
    frame->CAN_data[2] = LOBYTE(reg_addr);
    frame->CAN_data[3] = HIBYTE(reg_addr);
    frame->length = sizeof(masterId) + sizeof(reg_addr);

    return;
}

// Function to format msg requesting data from axis group
void FormatGiveMeData2(CAN_MSG* frame, uint8_t group, uint16_t reg_addr, bool is16bit)
{
    // Currently only support reading from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA2 | TM_DATA;

    if(!is16bit)
        optCode |= 0x1u;

    // Make full 16 bit ID code
    uint16_t idCode = ID_CODE(1, group);

    // Can Id construction
    frame->identifier = CAN_ID(optCode, idCode);

    // Payload
    uint16_t masterId = MASTER_ID(hostId);
    frame->CAN_data[0] = LOBYTE(masterId);
    frame->CAN_data[1] = HIBYTE(masterId); 
    frame->CAN_data[2] = LOBYTE(reg_addr);
    frame->CAN_data[3] = HIBYTE(reg_addr);
    frame->length = sizeof(masterId) + sizeof(reg_addr);
    return;
}

// Function to format msg to GOTO certain place in TML prog
void FormatGoTo(CAN_MSG* frame, uint8_t axis, uint16_t addr)
{
    uint16_t optCode = OPT_GOTO_LABEL;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE(addr);
    frame->CAN_data[1] = HIBYTE(addr);
    frame->length = sizeof(addr);

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
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE(val);
    frame->CAN_data[1] = HIBYTE(val);
    frame->length = sizeof(val);

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
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE((uint16_t)val);
    frame->CAN_data[1] = HIBYTE((uint16_t)val);
    frame->CAN_data[2] = LOBYTE((uint16_t)(val >> 16));
    frame->CAN_data[2] = HIBYTE((uint16_t)(val >> 16));
    frame->length = sizeof(val);

    return;
}

// Function to format set target posn = actual posn 
void FormatSTA(CAN_MSG* frame, uint8_t axis)
{
    uint16_t optCode = OPT_STA;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    frame->CAN_data[0] = LOBYTE(STA_PAYLOAD);
    frame->CAN_data[1] = HIBYTE(STA_PAYLOAD);
    frame->length = sizeof(STA_PAYLOAD);

    return;
}

// Function to set control of axis (on or off)
void FormatSetAxisControl(CAN_MSG* frame, uint8_t axis, bool isOn)
{
    uint16_t optCode = (isOn == true) ? OPT_AXISON : OPT_AXISOFF;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    // No payload
    frame->length = 0;
}

// Function to format set Target Update Mode 0 msg
void FormatTUM(CAN_MSG* frame, uint8_t axis, bool isMode0)
{
    uint16_t optCode = OPT_TUM;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    // Payload depends on the mode
    uint32_t payload;
    if(isMode0) {
        payload = TUM0_PAYLOAD; 
    } else {
        payload = TUM1_PAYLOAD;
    }
    for(int i = 0; i < sizeof(payload); i++) {
        frame->CAN_data[i] = (uint8_t)(payload >> i);
    }
    frame->length = sizeof(payload);

    return;
}

// Function to format msg to set motion mode
void FormatSetMotionMode(CAN_MSG* frame, uint8_t axis, uint32_t mode)
{
    uint16_t optCode = OPT_MODE;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    for(int i = 0; i < sizeof(mode); i++) {
        frame->CAN_data[i] = (uint8_t)(mode >> i);
    }
    frame->length = sizeof(mode);
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

// Wrapper function for MODEPP1 -- obsolete in new firmware
void FormatSetModePP3(CAN_MSG* frame, uint8_t axis)
{
    FormatSetMotionMode(frame, axis, MODE_PP3_PAYLOAD);
}

// Function to set to relative position reference
void FormatSetCPR(CAN_MSG* frame, uint8_t axis)
{
    uint16_t optCode = OPT_CPR;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    for(int i = 0; i < sizeof(CPR_PAYLOAD); i++) {
        frame->CAN_data[i] = (uint8_t)(CPR_PAYLOAD >> i);
    }
    frame->length = sizeof(CPR_PAYLOAD);
}

// Function to set to absolute position reference
void FormatSetCPA(CAN_MSG* frame, uint8_t axis)
{
    uint16_t optCode = OPT_CPA;
    uint16_t idCode = ID_CODE(0, axis);
    frame->identifier = CAN_ID(optCode, idCode);

    for(int i = 0; i < sizeof(CPA_PAYLOAD); i++) {
        frame->CAN_data[i] = (uint8_t)(CPA_PAYLOAD >> i);
    }
    frame->length = sizeof(CPA_PAYLOAD);
}

void FormatSetMasterId(CAN_MSG* frame, uint8_t axis, uint16_t newId)
{
    hostId = newId;
    FormatSetVal16(frame, axis, REG_MASTERID, HOST_TO_MASTER(newId));
}