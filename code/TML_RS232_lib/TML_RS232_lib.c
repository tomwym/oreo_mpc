#include "./include/TML_RS232_lib.h"
#include "./include/TML_RS232_def.h"
#include "../libmacaque_RS232/macaque_linux.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <float.h>

#define ADD_BYTES(hi,lo)        (uint16_t)(((uint16_t)hi << 8)|(lo))
#define ADD_WORDS(hi, lo)       (uint32_t)(((uint32_t)hi << 16)|lo)
#define GET_ID(code)            (uint8_t)(code >> 4)

static uint8_t hostId = 0;
static uint16_t baudRate = BAUDRATE_115200;

// Perform checksum
// msg_bytes is # bytes in msg - checksum
static uint8_t CalcChecksum(RS232_MSG* frame, uint8_t bytes)
{
    uint32_t sum = 0;
    for(uint8_t i = 0; i < bytes; i++) {
        sum += frame->RS232_data[i];
    }

    return sum % CSUM_MODULO;
}

// Get idcode
static uint16_t GetIdCode(motor_id_t* dest)
{
    uint16_t idCode;
    switch(dest->type) {
        case ID_TYPE_AXIS:
            idCode = AXIS_ID_CODE(dest->id);
            break;
        case ID_TYPE_GROUP:
            if(dest->id > MAX_GROUP_ID) {
                printf("Max group id exceeded. Setting to max\n");
                dest->id = MAX_GROUP_ID;
            }
            idCode = GROUP_ID_CODE(dest->id);
            break;
        case ID_TYPE_BROADCAST:
            idCode = BROADCAST_ID_CODE;
            break;
        default: 
            printf("Unknown id type inputted\n");
            idCode = 0;
    }
    return idCode;
}

// Helper to build message
static void FormatCommand(dest_dev_t dev, motor_id_t* dest, uint16_t optCode, void* payload, uint8_t payloadWordSize)
{
    RS232_MSG frame;

    frame.RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame.RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame.RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame.RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    uint16_t* ptr = (uint16_t*)payload;
    uint8_t idx = OFFSET_DATA_WORD1_HIGH;
    for(uint8_t i = 0; i < payloadWordSize; i++) {
        frame.RS232_data[idx+2*i] = HIBYTE(*(ptr+i));
        frame.RS232_data[idx+2*i+1] = LOBYTE(*(ptr+i));
    }

    uint8_t len = sizeof(optCode) + sizeof(idCode) + payloadWordSize*sizeof(uint16_t);
    frame.RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(&frame, len + sizeof(len));
    uint8_t csum_idx = OFFSET_DATA_WORD1_HIGH+2*payloadWordSize;
    frame.RS232_data[csum_idx] = csum;

    len += sizeof(len) + sizeof(csum);
    frame.length = len;

    if(dev == DEV_EYE) {
        AddCmdEye(&frame);
    } else {
        AddCmdNeck(&frame);
    }
}

// Set var which holds id recognized as host on the network
void InitLib(uint8_t id, serial_baudrate_t rate)
{
    hostId = id;
    baudRate = rate;
}

// Function to parse response from drive
// Calls the type-specific parse functions to get data
int8_t ParseResponse(RS232_MSG* frame, uint32_t* data, uint8_t* axis, uint16_t* reg_addr)
{
    // All offsets are shifted by one due to synchronization byte 0
    // Extract relevant data
    uint8_t length = frame->RS232_data[OFFSET_LENGTH];
    
    uint16_t destCode = ADD_BYTES(frame->RS232_data[OFFSET_IDCODE_HIGH], frame->RS232_data[OFFSET_IDCODE_LOW]);
    uint16_t optCode = ADD_BYTES(frame->RS232_data[OFFSET_OPTCODE_HIGH], frame->RS232_data[OFFSET_OPTCODE_LOW]);
    uint8_t payloadLength = length - sizeof(optCode) - sizeof(destCode);
    // Check message recipient
    uint8_t id = GET_ID(destCode);
    if(id != hostId) {
        printf("Message not destined for host (%x). Discarded\n", id);
        return -1;
    }

    // Sort based on type of message    
    // Take Data 1 message
    if((frame->RS232_data[OFFSET_OPTCODE_HIGH] == TAKE_DATA)) {
        // The first two bytes of payload is source axisId
        *axis = MIDBYTE(frame->RS232_data[OFFSET_DATA_WORD1_LOW], frame->RS232_data[OFFSET_DATA_WORD1_HIGH]);
        
        // LSB determines size of data returned
        if(frame->RS232_data[OFFSET_OPTCODE_LOW] & 0x1) {
            // Check that frame size matches expected
            if(payloadLength != TAKE_32BIT_SIZE) {
                printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE_32BIT_SIZE, payloadLength);
                return -1;
            }
            
            // Parse payload
            *data = ADD_WORDS((ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD4_HIGH], frame->RS232_data[OFFSET_DATA_WORD4_LOW])), ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD3_HIGH], frame->RS232_data[OFFSET_DATA_WORD3_LOW]));
            *reg_addr = (frame->RS232_data[OFFSET_DATA_WORD2_LOW])|((uint16_t)frame->RS232_data[OFFSET_DATA_WORD2_HIGH] << 8);
        } else {
            // Check that frame size matches expected
            if(payloadLength != TAKE_16BIT_SIZE) {
                printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE_16BIT_SIZE, payloadLength);
                return -1;
            }
            
            // Parse payload
            *data = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD3_HIGH], frame->RS232_data[OFFSET_DATA_WORD3_LOW]);
            *reg_addr = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD2_HIGH], frame->RS232_data[OFFSET_DATA_WORD2_LOW]);
        }
    } else if(frame->RS232_data[OFFSET_OPTCODE_HIGH] == TAKE_DATA2_16) {
        // 2 bytes after Group Bit is axisId
        *axis = frame->RS232_data[OFFSET_OPTCODE_LOW];
        
        if(payloadLength != TAKE2_16BIT_SIZE) {
            printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE2_16BIT_SIZE, payloadLength);
            return -1;
        }

        // 16 bits returned
        *data = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD2_HIGH], frame->RS232_data[OFFSET_DATA_WORD2_LOW]);
        *reg_addr = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD1_HIGH], frame->RS232_data[OFFSET_DATA_WORD1_LOW]);
    } else if(frame->RS232_data[OFFSET_OPTCODE_HIGH] == TAKE_DATA2_32) {
        // 2 bytes after Group Bit is axisId
        *axis = frame->RS232_data[OFFSET_OPTCODE_LOW];

        if(payloadLength != TAKE2_32BIT_SIZE) {
            printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE2_32BIT_SIZE, payloadLength);
            return -1;
        }
        
        // 32 bits returned
        *data = ADD_WORDS(ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD3_HIGH], frame->RS232_data[OFFSET_DATA_WORD3_LOW]), ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD2_HIGH], frame->RS232_data[OFFSET_DATA_WORD2_LOW]));
        *reg_addr = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD1_HIGH], frame->RS232_data[OFFSET_DATA_WORD1_LOW]);
    } else {
        printf("Recvd RS232 message with unexpected optCode (%x)\n", optCode);
        return -1;
    }

    return 0;    
}

// Ask specfic axis for data from memory
inline void SendGiveMeData(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, bool is16bit)
{
    // Current only support reading from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA | TM_DATA;
    if(!is16bit) {
        optCode |= 0x1u;
    }       

    // Payload
    uint32_t payload = ADD_WORDS(reg_addr, HOST_TO_MASTER(hostId));

    FormatCommand(dev, dest, optCode, &payload, sizeof(payload)/2);

    return;
}

// Function to format msg requesting data from axis group
inline void SendGiveMeData2(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, bool is16bit)
{
    // Only can read from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA2 | TM_DATA;
    if(!is16bit) {
        optCode |= 0x1u;
    }

    // Payload
    uint32_t payload = ADD_WORDS(reg_addr, HOST_TO_MASTER(hostId));

    FormatCommand(dev, dest, optCode, &payload, sizeof(payload)/2);

    return;
}

// GOTO message to certain place in TML program
void SendGoTo(dest_dev_t dev, motor_id_t* dest, uint16_t addr)
{ 
    uint16_t optCode = OPT_GOTO_LABEL;
    FormatCommand(dev, dest, optCode, &addr, sizeof(addr)/2);
}

// Currently only supporting setting values for addresses in RAM
// Function to format msg to modify 16 bit val in memory
void SetVal16(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, uint16_t val)
{
    uint16_t optCode = OPT_SET16_200;
    if(reg_addr >= SET16_ADDR_LIMIT) {
        optCode = OPT_SET16_800;
    }
    optCode |= (LSB9_MASK & reg_addr);

    FormatCommand(dev, dest, optCode, &val, sizeof(val)/2);
}

// Helper to transform float to fixed point representation
uint32_t DoubleToFixed(double num)
{
    return (uint32_t)(int32_t)(round(num * ((uint32_t)1 << FIXED_POINT_FRACT_BITS)));
}

// Helper to transform fixed to float
double FixedToDouble(uint32_t num)
{
    return (double)(int32_t)num / (double)(1 << FIXED_POINT_FRACT_BITS);
}

// Currently only supporting setting values for addresses in RAM
// Function to format msg to modify 32 bit val in memory
void SetVal32(dest_dev_t dev, motor_id_t* dest, uint16_t reg_addr, uint32_t val)
{
    // OptCode changes based on address
    uint16_t optCode = OPT_SET32_200;
    if(reg_addr >= SET32_ADDR_LIMIT) {
        optCode = OPT_SET32_800;
    }
    optCode |= (LSB9_MASK & reg_addr);

    FormatCommand(dev, dest, optCode, &val, sizeof(val)/2);

    return;
}

// Function to format set target posn = actual posn 
void SendSTA(dest_dev_t dev, motor_id_t* dest)
{
    uint16_t optCode = OPT_STA;
    uint16_t payload = STA_PAYLOAD;
    FormatCommand(dev, dest, optCode, &payload, sizeof(payload)/2);

    return;
}

// Function to set control of axis (on or off)
void SetAxisControl(dest_dev_t dev, motor_id_t* dest, bool turnOn)
{
    uint16_t optCode = (turnOn == true) ? OPT_AXISON : OPT_AXISOFF;
    FormatCommand(dev, dest, optCode, NULL, 0);
}

// Function to format set Target Update Mode 0 msg
void SetTUM(dest_dev_t dev, motor_id_t* dest, uint8_t mode)
{
    uint16_t optCode = OPT_TUM;
    uint32_t payload;
    if(mode == 0)
        payload = TUM0_PAYLOAD;
    else
        payload = TUM1_PAYLOAD;
    
    FormatCommand(dev, dest, optCode, &payload, sizeof(payload)/2);
}

// Function to format msg to change motion mode
static void SetMotionMode(dest_dev_t dev, motor_id_t* dest, uint32_t mode)
{
    uint16_t optCode = OPT_MODE;
    FormatCommand(dev, dest, optCode, &mode, sizeof(mode)/2);
}

// Wrapper function for MODEPP
void SetModePP(dest_dev_t dev, motor_id_t* dest)
{
    SetMotionMode(dev, dest, MODE_PP_PAYLOAD);
}

// Wrapper function for MODEPP1 -- obsolete in new firmware
void SetModePP1(dest_dev_t dev, motor_id_t* dest)
{
    SetMotionMode(dev, dest, MODE_PP1_PAYLOAD);
}

// Wrapper function for MODEPP3 -- obsolete in new firmware
void SetModePP3(dest_dev_t dev, motor_id_t* dest)
{
    SetMotionMode(dev, dest, MODE_PP3_PAYLOAD);
}

// Wrapper function for MODE_TES -- torque slow loop
void SetModeTorqueSlow(dest_dev_t dev, motor_id_t* dest)
{
    SetMotionMode(dev, dest, MODE_TES);
}

// Wrapper function for MODE_TEF -- torque fast loop
void SetModeTorqueFast(dest_dev_t dev, motor_id_t* dest)
{
    SetMotionMode(dev, dest, MODE_TEF);
}

// Function to set position reference mode
// Set rel to 1 for relative position reference
// Set rel to 0 for absolute position reference
static void SetPosRef(dest_dev_t dev, motor_id_t* dest, uint8_t rel)
{
    uint16_t optCode = OPT_POSREF;    
    uint32_t payload;
    if(rel > 0) {
        payload = CPR_PAYLOAD;
    }
    else {
        payload = CPA_PAYLOAD;
    }    
    FormatCommand(dev, dest, optCode, &payload, sizeof(payload)/2);
}

// Wrapper for setting relative position reference
void SetCPR(dest_dev_t dev, motor_id_t* dest) 
{
    SetPosRef(dev, dest, 1);
}

// Wrapper for setting absolute position reference
void SetCPA(dest_dev_t dev, motor_id_t* dest) 
{
    SetPosRef(dev, dest, 0);
}

// Set the masterid for axis
void SetMasterId(dest_dev_t dev, motor_id_t* dest, uint16_t newId)
{
    hostId = newId;
    SetVal16(dev, dest, REG_MASTERID, hostId);
}

// Format message to request all axis ids from the controllers
void SendPing(dest_dev_t dev, motor_id_t* dest)
{
    // Broadcast ping message
    uint16_t optCode = OPT_PING;
    uint16_t masterId = HOST_TO_MASTER(hostId);
    // Scale latency based on baudrate
    uint16_t latency = PING_LATENCY_115200 * (BAUDRATE_115200 - baudRate)*2;
    uint32_t payload = ADD_WORDS(latency, masterId);
    FormatCommand(dev, dest, optCode, &payload, sizeof(payload)/2);
}

// Get relevant data from pong message
// Returns false if not a pong message
int8_t ParsePong(RS232_MSG* frame, uint8_t* axis, char version[VERSION_SIZE])
{    
    uint16_t optCode = (frame->RS232_data[OFFSET_OPTCODE_HIGH] << 8) | (frame->RS232_data[OFFSET_OPTCODE_LOW]); 
    if(frame->RS232_data[OFFSET_OPTCODE_HIGH] != HIBYTE(OPT_PONG)) {
        printf("Received frame was not pong response\n");
        return -1;
    }
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);
     
    *axis = frame->RS232_data[OFFSET_OPTCODE_LOW];
    version[0] = frame->RS232_data[OFFSET_DATA_WORD1_LOW];
    version[1] = frame->RS232_data[OFFSET_DATA_WORD1_HIGH];
    version[2] = frame->RS232_data[OFFSET_DATA_WORD1_LOW];
    version[3] = frame->RS232_data[OFFSET_DATA_WORD1_HIGH];

    return 0;
}

// Immediately update position with stored parameters
// For broadcast message, set as group, id = 0
void UpdatePosn(dest_dev_t dev, motor_id_t* dest)
{
    uint16_t optCode = OPT_UPD;
    FormatCommand(dev, dest, optCode, NULL, 0);
}

// Enable control-loop sync messages on axes
// Period sets time-between sync messages
// Set period to 0 to disable sync
void SetSync(dest_dev_t dev, motor_id_t* dest)
{
    uint16_t optCode = OPT_SETSYNC;
    uint32_t period = SYNC_MSG_PERIOD;
    FormatCommand(dev, dest, optCode, &period, sizeof(period)/2);
}

// Set serial baud rate
void SetBaudRate(dest_dev_t dev, serial_baudrate_t rate)
{
    uint16_t optCode = OPT_SCIBR;
    baudRate = rate;
    motor_id_t dest = {.type = ID_TYPE_BROADCAST, .id = 0};
    FormatCommand(dev, &dest, optCode, &rate, sizeof(baudRate)/2);
}

// Set external reference mode
void SetExtRefMode(dest_dev_t dev, motor_id_t* dest, uint32_t mode)
{
    uint16_t optCode = OPT_EXTREF;
    FormatCommand(dev, dest, optCode, &mode, sizeof(mode)/2);
}

// Set external reference mode to online (send messages from host)
void SetExtRefOnline(dest_dev_t dev, motor_id_t* dest)
{
    SetExtRefMode(dev, dest, EXTREF_ONLINE_PAYLOAD);
}

// Set external reference mode to analog (use analog input)
void SetExtRefAnalog(dest_dev_t dev, motor_id_t* dest)
{
    SetExtRefMode(dev, dest, EXTREF_ANALOG_PAYLOAD);
}

// Set external reference mode to digital (use digital input)
void SetExtRefDigital(dest_dev_t dev, motor_id_t* dest)
{
    SetExtRefMode(dev, dest, EXTREF_DIGITAL_PAYLOAD);
}