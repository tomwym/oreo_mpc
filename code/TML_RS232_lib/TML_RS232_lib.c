#include "./include/TML_RS232_lib.h"
#include "./include/TML_RS232_def.h"

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
    uint8_t length = frame->RS232_data[OFFSET_LENGTH+1];
    
    uint16_t destCode = ADD_BYTES(frame->RS232_data[OFFSET_IDCODE_HIGH+1], frame->RS232_data[OFFSET_IDCODE_LOW+1]);
    uint16_t optCode = ADD_BYTES(frame->RS232_data[OFFSET_OPTCODE_HIGH+1], frame->RS232_data[OFFSET_OPTCODE_LOW+1]);
    uint8_t payloadLength = length - sizeof(optCode) - sizeof(destCode);
    // Check message recipient
    uint8_t id = GET_ID(destCode);
    if(id != hostId) {
        printf("Message not destined for host (%x). Discarded\n", id);
        return -1;
    }

    // Can check host bit to differentiate between Type B and Type C messages
    //uint8_t isTypeC = (HOST_BIT(frameId) > 0) ? 1 : 0;

    // Sort based on type of message    
    // Take Data 1 message
    if((frame->RS232_data[OFFSET_OPTCODE_HIGH+1] == TAKE_DATA)) {
        // The first two bytes of payload is source axisId
        *axis = MIDBYTE(frame->RS232_data[OFFSET_DATA_WORD1_LOW+1], frame->RS232_data[OFFSET_DATA_WORD1_HIGH+1]);
        
        // LSB determines size of data returned
        if(frame->RS232_data[OFFSET_OPTCODE_LOW+1] & 0x1) {
            // Check that frame size matches expected
            if(payloadLength != TAKE_32BIT_SIZE) {
                printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE_32BIT_SIZE, payloadLength);
                return -1;
            }
            
            // Parse payload
            *data = ADD_WORDS((ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD4_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD4_LOW+1])), ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD3_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD3_LOW+1]));
            *reg_addr = (frame->RS232_data[OFFSET_DATA_WORD2_LOW+1])|((uint16_t)frame->RS232_data[OFFSET_DATA_WORD2_HIGH+1] << 8);
        } else {
            // Check that frame size matches expected
            if(payloadLength != TAKE_16BIT_SIZE) {
                printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE_16BIT_SIZE, payloadLength);
                return -1;
            }
            
            // Parse payload
            *data = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD3_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD3_LOW+1]);
            *reg_addr = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD2_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD2_LOW+1]);
        }
    } else if(frame->RS232_data[OFFSET_OPTCODE_HIGH+1] == TAKE_DATA2_16) {
        // 2 bytes after Group Bit is axisId
        *axis = frame->RS232_data[OFFSET_OPTCODE_LOW+1];
        
        if(payloadLength != TAKE2_16BIT_SIZE) {
            printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE2_16BIT_SIZE, payloadLength);
            return -1;
        }

        // 16 bits returned
        *data = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD2_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD2_LOW+1]);
        *reg_addr = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD1_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD1_LOW+1]);
    } else if(frame->RS232_data[OFFSET_OPTCODE_HIGH+1] == TAKE_DATA2_32) {
        // 2 bytes after Group Bit is axisId
        *axis = frame->RS232_data[OFFSET_OPTCODE_LOW+1];

        if(payloadLength != TAKE2_32BIT_SIZE) {
            printf("Expected payload size (%d) does not match recvd size (%d)\n", TAKE2_32BIT_SIZE, payloadLength);
            return -1;
        }
        
        // 32 bits returned
        *data = ADD_WORDS(ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD3_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD3_LOW+1]), ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD2_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD2_LOW+1]));
        *reg_addr = ADD_BYTES(frame->RS232_data[OFFSET_DATA_WORD1_HIGH+1], frame->RS232_data[OFFSET_DATA_WORD1_LOW+1]);
    } else {
        printf("Recvd RS232 message with unexpected optCode (%x)\n", optCode);
        return -1;
    }

    return 0;    
}

// Ask specfic axis for data from memory
inline void FormatGiveMeData(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, bool is16bit)
{
    // Current only support reading from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA | TM_DATA;
    if(!is16bit)
        optCode |= 0x1;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);
    
    // Id code
    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    // Payload
    uint16_t master_id = HOST_TO_MASTER(hostId);
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(master_id);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE(master_id);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE(reg_addr);
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE(reg_addr);

    // Message length
    uint8_t len = sizeof(idCode) + sizeof(optCode) + sizeof(master_id) + sizeof(reg_addr);
    frame->RS232_data[OFFSET_LENGTH] = len;

    // Checksum
    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;
    
    // Total length
    len += sizeof(csum) + sizeof(len);
    frame->length = len;

    return;
}

// Function to format msg requesting data from axis group
inline void FormatGiveMeData2(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, bool is16bit)
{
    // Only can read from data memory
    uint16_t optCode = OPT_GIVE_ME_DATA2 | TM_DATA;
    if(!is16bit) {
        optCode |= 0x1u;
    }
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);
    
    // Id code
    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    // Payload
    uint16_t masterId = HOST_TO_MASTER(hostId);
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(masterId);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE(masterId);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE(reg_addr);
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE(reg_addr);

    // Len byte
    uint8_t len = sizeof(idCode) + sizeof(masterId) + sizeof(reg_addr) + sizeof(optCode);
    frame->RS232_data[OFFSET_LENGTH] = len;

    // Checksum
    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    // Total length
    len += sizeof(csum) + sizeof(len);
    frame->length = len;

    return;
}

// GOTO message to certain place in TML program
void FormatGoTo(RS232_MSG* frame, motor_id_t* dest, uint16_t addr)
{ 
    uint16_t optCode = OPT_GOTO_LABEL;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    // Id codes based on type
    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    // Payload
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(addr);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE(addr);

    // Length
    uint8_t len = sizeof(idCode) + sizeof(optCode) + sizeof(addr);
    frame->RS232_data[OFFSET_LENGTH] = len;

    // Checksum
    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD1_LOW+1] = csum;

    // total length
    len += sizeof(csum) + sizeof(len);
    frame->length = len;
}



// Currently only supporting setting values for addresses in RAM
// Function to format msg to modify 16 bit val in memory
void FormatSetVal16(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, uint16_t val)
{
    uint16_t optCode = OPT_SET16_200;
    if(reg_addr >= SET16_ADDR_LIMIT) {
        optCode = OPT_SET16_800;
    }
    optCode |= (LSB9_MASK & reg_addr);
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(val);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE(val);

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(val);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD1_LOW+1] = csum;

    len += sizeof(csum) + sizeof(len);
    frame->length = len;
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
void FormatSetVal32(RS232_MSG* frame, motor_id_t* dest, uint16_t reg_addr, uint32_t val)
{
    // OptCode changes based on address
    uint16_t optCode = OPT_SET32_200;
    if(reg_addr >= SET32_ADDR_LIMIT) {
        optCode = OPT_SET32_800;
    }
    optCode |= (LSB9_MASK & reg_addr);
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE((uint16_t)val);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE((uint16_t)val);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE((uint16_t)(val >> 16));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE((uint16_t)(val >> 16));

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(val);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    len += sizeof(csum) + sizeof(len);
    frame->length = len;

    return;
}

// Function to format set target posn = actual posn 
void FormatSTA(RS232_MSG* frame, motor_id_t* dest)
{
    uint16_t optCode = OPT_STA;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(STA_PAYLOAD);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE(STA_PAYLOAD);
    
    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(STA_PAYLOAD);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD1_LOW+1] = csum;

    len += sizeof(len) + sizeof(csum);
    frame->length = len;

    return;
}

// Function to set control of axis (on or off)
void FormatSetAxisControl(RS232_MSG* frame, motor_id_t* dest, bool turnOn)
{
    uint16_t optCode = (turnOn == true) ? OPT_AXISON : OPT_AXISOFF;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);
    
    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    uint8_t len = sizeof(optCode) + sizeof(idCode);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_OPTCODE_LOW+1] = csum;

    len += sizeof(csum) + sizeof(len);
    frame->length = len;
}

// Function to format set Target Update Mode 0 msg
void FormatTUM(RS232_MSG* frame, motor_id_t* dest, uint8_t mode)
{
    uint16_t optCode = OPT_TUM;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);
    
    uint32_t payload;
    if(mode == 0)
        payload = TUM0_PAYLOAD;
    else
        payload = TUM1_PAYLOAD;
    
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE((uint16_t)payload);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE((uint16_t)payload);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE(payload >> 16);
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE(payload >> 16);

    uint8_t len = sizeof(payload) + sizeof(idCode) + sizeof(optCode);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    len += sizeof(csum) + sizeof(len);
    frame->length = len;  
}

// Function to format msg to change motion mode
void FormatSetMotionMode(RS232_MSG* frame, motor_id_t* dest, uint32_t mode)
{
    uint16_t optCode = OPT_MODE;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE((uint16_t)mode);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE((uint16_t)mode);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE(mode >> 16);
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE(mode >> 16);

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(mode);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    len += sizeof(csum) + sizeof(len);
    frame->length = len;
}

// Wrapper function for MODEPP
void FormatSetModePP(RS232_MSG* frame, motor_id_t* dest)
{
    FormatSetMotionMode(frame, dest, MODE_PP_PAYLOAD);
}

// Wrapper function for MODEPP1 -- obsolete in new firmware
void FormatSetModePP1(RS232_MSG* frame, motor_id_t* dest)
{
    FormatSetMotionMode(frame, dest, MODE_PP1_PAYLOAD);
}

// Wrapper function for MODEPP3 -- obsolete in new firmware
void FormatSetModePP3(RS232_MSG* frame, motor_id_t* dest)
{
    FormatSetMotionMode(frame, dest, MODE_PP3_PAYLOAD);
}

// Function to set position reference mode
// Set rel to 1 for relative position reference
// Set rel to 0 for absolute position reference
void FormatSetPosRef(RS232_MSG* frame, motor_id_t* dest, uint8_t rel)
{
    uint16_t optCode = OPT_POSREF;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);
    
    uint32_t payload;
    if(rel > 0)
        payload = CPR_PAYLOAD;
    else    
        payload = CPA_PAYLOAD;
    
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE((uint16_t)payload);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE((uint16_t)payload);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE(payload >> 16);
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE(payload >> 16);

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(payload);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len+sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    len += sizeof(len) + sizeof(csum);
    frame->length = len;
}

// Wrapper for setting relative position reference
void FormatSetCPR(RS232_MSG* frame, motor_id_t* dest) 
{
    FormatSetPosRef(frame, dest, 1);
}

// Wrapper for setting absolute position reference
void FormatSetCPA(RS232_MSG* frame, motor_id_t* dest) 
{
    FormatSetPosRef(frame, dest, 0);
}

// Set the masterid for axis
void FormatSetMasterId(RS232_MSG* frame, motor_id_t* dest, uint16_t newId)
{
    hostId = newId;
    FormatSetVal16(frame, dest, REG_MASTERID, hostId);
}

// Format message to request all axis ids from the controllers
void FormatPing(RS232_MSG* frame, motor_id_t* dest)
{
    // Broadcast ping message
    uint16_t optCode = OPT_PING;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    uint16_t masterId = HOST_TO_MASTER(hostId);
    // Scale latency based on baudrate
    uint16_t latency = PING_LATENCY_115200 * (BAUDRATE_115200 - baudRate)*2;
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(masterId);
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = LOBYTE(masterId);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE(latency);
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE(latency);

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(masterId) + sizeof(latency);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len + sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    len += sizeof(len) + sizeof(csum);
    frame->length = len;
}

// Get relevant data from pong message
// Returns false if not a pong message
int8_t ParsePong(RS232_MSG* frame, uint8_t* axis, char version[VERSION_SIZE])
{
    uint16_t optCode = (frame->RS232_data[OFFSET_OPTCODE_HIGH] << 8) | (frame->RS232_data[OFFSET_OPTCODE_LOW]); 
    if(frame->RS232_data[OFFSET_OPTCODE_HIGH] != HIBYTE(OPT_PONG)) {
        printf("Recevied frame was not pong response\n");
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
void FormatUpdatePosn(RS232_MSG* frame, motor_id_t* dest)
{
    uint16_t optCode = OPT_UPD;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    uint8_t len = sizeof(optCode) + sizeof(idCode);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len + sizeof(len));
    frame->RS232_data[OFFSET_OPTCODE_LOW+1] = csum;

    len += sizeof(len) + sizeof(csum);
    frame->length = len;
}

// Enable control-loop sync messages on axes
// Period sets time-between sync messages
// Set period to 0 to disable sync
void FormatSetSync(RS232_MSG* frame, motor_id_t* dest)
{
    uint16_t optCode = OPT_SETSYNC;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    uint32_t period = SYNC_MSG_PERIOD;
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE((uint16_t)period);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE((uint16_t)period);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE((uint16_t)(period >> 16));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE((uint16_t)(period >> 16));

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(period);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len + sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW+1] = csum;

    len += sizeof(csum) + sizeof(len);
    frame->length = len;
}

// Set serial baud rate
void FormatSetBaudRate(RS232_MSG* frame, serial_baudrate_t rate)
{
    uint16_t optCode = OPT_SCIBR;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = BROADCAST_ID_CODE;
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);

    baudRate = rate;
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE(baudRate);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE(baudRate);

    uint8_t len = sizeof(optCode) + sizeof(idCode) + sizeof(baudRate);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len + sizeof(len));
    frame->RS232_data[OFFSET_DATA_WORD1_LOW+1] = csum;

    len += sizeof(len) + sizeof(csum);
    frame->length = len;
}

// Set external reference mode
void SetExtRefMode(RS232_MSG* frame, motor_id_t* dest, uint32_t mode)
{
    uint16_t optCode = OPT_EXTREF;
    frame->RS232_data[OFFSET_OPTCODE_HIGH] = HIBYTE(optCode);
    frame->RS232_data[OFFSET_OPTCODE_LOW] = LOBYTE(optCode);

    uint16_t idCode = GetIdCode(dest);
    frame->RS232_data[OFFSET_IDCODE_HIGH] = HIBYTE(idCode);
    frame->RS232_data[OFFSET_IDCODE_LOW] = LOBYTE(idCode);
    
    frame->RS232_data[OFFSET_DATA_WORD1_HIGH] = HIBYTE((uint16_t)mode);
    frame->RS232_data[OFFSET_DATA_WORD1_LOW] = LOBYTE((uint16_t)mode);
    frame->RS232_data[OFFSET_DATA_WORD2_HIGH] = HIBYTE((uint16_t)(mode >> 16));
    frame->RS232_data[OFFSET_DATA_WORD2_LOW] = LOBYTE((uint16_t)(mode >> 16));

    uint8_t len = sizeof(idCode) + sizeof(optCode) + sizeof(mode);
    frame->RS232_data[OFFSET_LENGTH] = len;

    uint8_t csum = CalcChecksum(frame, len + sizeof(len));
    frame->RS232_data[OFFSET_CHECKSUM] = csum;

    len += sizeof(len) + sizeof(csum);
    frame->length = len;
}

// Set external reference mode to online (send messages from host)
void SetExtRefOnline(RS232_MSG* frame, motor_id_t* dest)
{
    SetExtRefMode(frame, dest, EXTREF_ONLINE_PAYLOAD);
}

// Set external reference mode to analog (use analog input)
void SetExtRefAnalog(RS232_MSG* frame, motor_id_t* dest)
{
    SetExtRefMode(frame, dest, EXTREF_ANALOG_PAYLOAD);
}

// Set external reference mode to digital (use digital input)
void SetExtRefDigital(RS232_MSG* frame, motor_id_t* dest)
{
    SetExtRefMode(frame, dest, EXTREF_DIGITAL_PAYLOAD);
}