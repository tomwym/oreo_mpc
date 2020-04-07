#include "../sock_interface/sock.h"
#include "./include/TML_RS232_lib.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// Additional registers for testing
#define REG_SRH               (uint16_t)(0x090Fu)   // uint
#define REG_AAR               (uint16_t)(0x030Cu)   // uint
#define REG_CADIN             (uint16_t)(0x025Cu)   // int
#define REG_NLINES            (uint16_t)(0x0984u)   // ulong
#define REG_HOMESPD           (uint16_t)(0x0994u)   // fixed
#define REG_POSINIT           (uint16_t)(0x02F2u)   // ulong

// Motion Modes for testing
#define MODE_PP_PAYLOAD      (uint32_t)(0x8701BFC1u)
#define MODE_PP1_PAYLOAD     (uint32_t)(0x8501BDC1)
#define MODE_PP3_PAYLOAD     MODE_PP_PAYLOAD
#define MODE_TEF             (uint32_t)(0x8120B1E0u)
#define MODE_TES             (uint32_t)(0x8100B1C0u)

#define TEST_HOSTID     (8)
#define BUFF_SIZE       (255)

static char* get_msg_str(RS232_MSG* frame, char* buff)
{
    snprintf(buff, BUFF_SIZE, "0x");
    char* ptr = buff+2;
    for(int i = 0; i < frame->length; i++) {
        snprintf(ptr, BUFF_SIZE, " %02x", frame->RS232_data[i]);
        ptr+=sizeof(frame->RS232_data[i]) + 2;
    }

    return buff;
}

static bool msg_comp(RS232_MSG* frame, RS232_MSG* exp_frame) 
{
    // Simple check
    if(frame->length != exp_frame->length) {
        printf("Size: Frame=%d Expected=%d\n", frame->length, exp_frame->length);
        return false;
    }
    
    // Compare byte-by-byte 
    if(memcmp(frame, exp_frame, frame->length) == 0) {
        return true;
    }

    char frame_str[BUFF_SIZE];
    char exp_frame_str[BUFF_SIZE];
    printf("Mismatch: Frame=%s Expected=%s\n", get_msg_str(frame, frame_str), get_msg_str(exp_frame, exp_frame_str));
    
    return false;
}

#define NUM_GIVEMEDATA_TESTS    3
static void Test_GiveMeData()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_GIVEMEDATA_TESTS] = {ID_TYPE_AXIS, ID_TYPE_AXIS, ID_TYPE_AXIS};
    uint8_t ids[NUM_GIVEMEDATA_TESTS] = {34, 58, 8};
    uint16_t reg_addr[NUM_GIVEMEDATA_TESTS] = {REG_CACC, REG_POSERR, REG_MASTERID};
    bool is16bit[NUM_GIVEMEDATA_TESTS] = {false, true, true};

    RS232_MSG exp_frame[NUM_GIVEMEDATA_TESTS] = {
        {.RS232_data = {0x08, 0x02, 0x20, 0xB0, 0x05, 0x00, 0x81, 0x02, 0xA2, 0x04}, .length=10},
        {.RS232_data = {0x08, 0x03, 0xA0, 0xB0, 0x04, 0x00, 0x81, 0x02, 0x2A, 0x0C}, .length=10},
        {.RS232_data = {0x08, 0x00, 0x80, 0xB0, 0x04, 0x00, 0x81, 0x09, 0x27, 0xED}, .length=10},
    };
    
    printf("*** %s Results ***\n", __func__);
    for (int i = 0; i < NUM_GIVEMEDATA_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatGiveMeData(&frame, &id, reg_addr[i], is16bit[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }
    
    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_GIVEMEDATA_TESTS);
    
    return;
}

#define NUM_GIVEMEDATA2_TESTS       6
static void Test_GiveMeData2()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_GIVEMEDATA2_TESTS] = {ID_TYPE_AXIS, ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_GIVEMEDATA2_TESTS] = {34, 58, 4, 8, 0, 8};
    uint16_t reg_addr[NUM_GIVEMEDATA2_TESTS] = {REG_CACC, REG_POSERR, REG_CACC, REG_POSERR, REG_APOS, REG_APOS2};
    bool is16bit[NUM_GIVEMEDATA2_TESTS] = {false, true, false, true, false, false};

    RS232_MSG exp_frame[NUM_GIVEMEDATA2_TESTS] = {
        {.RS232_data = {0x08, 0x02, 0x20, 0xB2, 0x05, 0x00, 0x81, 0x02, 0xA2, 0x06}, .length = 10},
        {.RS232_data = {0x08, 0x03, 0xA0, 0xB2, 0x04, 0x00, 0x81, 0x02, 0x2A, 0x0E}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x80, 0xB2, 0x05, 0x00, 0x81, 0x02, 0xA2, 0x74}, .length = 10},
        {.RS232_data = {0x08, 0x18, 0x00, 0xB2, 0x04, 0x00, 0x81, 0x02, 0x2A, 0x83}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x00, 0xB2, 0x05, 0x00, 0x81, 0x02, 0x28, 0x7A}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0xB2, 0x05, 0x00, 0x81, 0x08, 0x1C, 0xE4}, .length = 10}
    };
    
    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_GIVEMEDATA2_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatGiveMeData2(&frame, &id, reg_addr[i], is16bit[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_GIVEMEDATA2_TESTS);


    return;
}

#define NUM_SETVAL16_TESTS        4
static void Test_SetVal16()
{
    uint8_t passed = 0;
    uint16_t val[NUM_SETVAL16_TESTS] = {0x8524, -1502, 0x8524, 0x8524};
    id_type_t idTypes[NUM_SETVAL16_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_SETVAL16_TESTS] = {188, 5, 0, 8};
    uint16_t reg_addr[NUM_SETVAL16_TESTS] = {REG_POSERR, REG_SRH, REG_CADIN, REG_AAR};
    RS232_MSG exp_frame[NUM_SETVAL16_TESTS] = {
        {.RS232_data = {0x06, 0x0B, 0xC0, 0x20, 0x2A, 0x85, 0x24, 0xC4}, .length = 8},
        {.RS232_data = {0x06, 0x11, 0x00, 0x23, 0x0F, 0xFA, 0x22, 0x65}, .length = 8},
        {.RS232_data = {0x06, 0x10, 0x00, 0x20, 0x5C, 0x85, 0x24, 0x3B}, .length = 8},
        {.RS232_data = {0x06, 0x00, 0x80, 0x21, 0x0C, 0x85, 0x24, 0x5C}, .length = 8},
    };
    
    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_SETVAL16_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetVal16(&frame, &id, reg_addr[i], val[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_SETVAL16_TESTS);
    return;
}

#define NUM_SETVAL32_TESTS      4
static void Test_SetVal32()
{
    uint8_t passed = 0;
    uint32_t val[NUM_SETVAL32_TESTS] = {DoubleToFixed(-981.85696), -708439, 0x20987643, DoubleToFixed(12.4856)};
    id_type_t idTypes[NUM_SETVAL32_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_SETVAL32_TESTS] = {24, 1, 0, 8};
    uint16_t reg_addr[NUM_SETVAL32_TESTS] = {REG_HOMESPD, REG_POSINIT, REG_APOS2, REG_CACC};
    RS232_MSG exp_frame[NUM_SETVAL32_TESTS] = {
        {.RS232_data = {0x08, 0x01, 0x80, 0x27, 0x94, 0x24, 0x9E, 0xFC, 0x2A, 0x2C}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x10, 0x24, 0xF2, 0x30, 0xA9, 0xFF, 0xF5, 0x0B}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x00, 0x26, 0x1C, 0x76, 0x43, 0x20, 0x98, 0xCB}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0x24, 0xA2, 0x7C, 0x50, 0x00, 0x0C, 0x26}, .length = 10},
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_SETVAL32_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetVal32(&frame, &id, reg_addr[i], val[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_SETVAL32_TESTS);
    return;
}

#define NUM_GOTO_TESTS      4
static void Test_GoTo()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_GOTO_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_GOTO_TESTS] = {186, 7, 0, 8};
    uint16_t addr[NUM_GOTO_TESTS] = {POSN_LOOP_EYE_IP, WAIT_LOOP_EYE_IP, POSN_LOOP_NECK_IP, WAIT_LOOP_NECK_IP};
    RS232_MSG exp_frame[NUM_GOTO_TESTS] = {
        {.RS232_data = {0x06, 0x0B, 0xA0, 0x74, 0x00, 0x40, 0x25, 0x87}, .length = 8},
        {.RS232_data = {0x06, 0x14, 0x00, 0x74, 0x00, 0x40, 0x1E, 0xE9}, .length = 8},
        {.RS232_data = {0x06, 0x10, 0x00, 0x74, 0x00, 0x40, 0x3B, 0x05}, .length = 8},
        {.RS232_data = {0x06, 0x00, 0x80, 0x74, 0x00, 0x40, 0x34, 0x6E}, .length = 8},
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_GOTO_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatGoTo(&frame, &id, addr[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_GOTO_TESTS);
    return;
}

#define NUM_STA_TESTS       4
static void Test_STA()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_STA_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_STA_TESTS] = {190, 8, 0, 8};
    RS232_MSG exp_frame[NUM_STA_TESTS] = {
        {.RS232_data = {0x06, 0x0B, 0xE0, 0x2C, 0xB2, 0x02, 0x28, 0xF9}, .length = 8},
        {.RS232_data = {0x06, 0x18, 0x00, 0x2C, 0xB2, 0x02, 0x28, 0x26}, .length = 8},
        {.RS232_data = {0x06, 0x10, 0x00, 0x2C, 0xB2, 0x02, 0x28, 0x1E}, .length = 8},
        {.RS232_data = {0x06, 0x00, 0x80, 0x2C, 0xB2, 0x02, 0x28, 0x8E}, .length = 8},
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_STA_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSTA(&frame, &id);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_STA_TESTS);
    return;
}

#define NUM_AXISCTRL_TESTS          4
static void Test_SetAxisControl()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_AXISCTRL_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_AXISCTRL_TESTS] = {201, 4, 0, 8};
    bool turnOn[NUM_AXISCTRL_TESTS] = {true, false, true, false};
    RS232_MSG exp_frame[NUM_AXISCTRL_TESTS] = {
        {.RS232_data = {0x04, 0x0C, 0x90, 0x01, 0x02, 0xA3}, .length = 6},
        {.RS232_data = {0x04, 0x10, 0x80, 0x00, 0x02, 0x96}, .length = 6},
        {.RS232_data = {0x04, 0x10, 0x00, 0x01, 0x02, 0x17}, .length = 6},
        {.RS232_data = {0x04, 0x00, 0x80, 0x00, 0x02, 0x87}, .length = 6},
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_AXISCTRL_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetAxisControl(&frame, &id, turnOn[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_AXISCTRL_TESTS);
    return;
}

#define NUM_TUM_TESTS          4
static void Test_TUM()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_TUM_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_AXISCTRL_TESTS] = {87, 3, 0, 8};
    uint8_t mode[NUM_AXISCTRL_TESTS] = {0, 1, 0, 1};
    RS232_MSG exp_frame[NUM_TUM_TESTS] = {
        {.RS232_data = {0x08, 0x05, 0x70, 0x59, 0x09, 0xBF, 0xFF, 0x00, 0x00, 0x9D}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x40, 0x59, 0x09, 0xFF, 0xFF, 0x40, 0x00, 0xF8}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x00, 0x59, 0x09, 0xBF, 0xFF, 0x00, 0x00, 0x38}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0x59, 0x09, 0xFF, 0xFF, 0x40, 0x00, 0x28}, .length = 10}
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_TUM_TESTS; i++) {
        RS232_MSG frame;
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatTUM(&frame, &id, mode[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_AXISCTRL_TESTS);
    return;
}

#define NUM_MOTIONMODE_TESTS        5
static void Test_SetMotionMode()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_MOTIONMODE_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS, ID_TYPE_AXIS};
    uint8_t ids[NUM_MOTIONMODE_TESTS] = {7, 1, 0, 8, 8};
    uint32_t mode[NUM_MOTIONMODE_TESTS] = {MODE_PP_PAYLOAD, MODE_PP1_PAYLOAD, MODE_PP3_PAYLOAD, MODE_TES, MODE_TEF};
    RS232_MSG exp_frame[NUM_MOTIONMODE_TESTS] = {
        {.RS232_data = {0x08, 0x00, 0x70, 0x59, 0x09, 0xBF, 0xC1, 0x87, 0x01, 0xE2}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x10, 0x59, 0x09, 0xBD, 0xC1, 0x85, 0x01, 0x8E}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x00, 0x59, 0x09, 0xBF, 0xC1, 0x87, 0x01, 0x82}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0x59, 0x09, 0xB1, 0xC0, 0x81, 0x00, 0xDC}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0x59, 0x09, 0xB1, 0xE0, 0x81, 0x20, 0x1C}, .length = 10},
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_MOTIONMODE_TESTS; i++) {
        RS232_MSG frame; 
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetMotionMode(&frame, &id, mode[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_MOTIONMODE_TESTS);
    return;
}

#define NUM_POSREF_TESTS        4
static void Test_SetPosRef()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_POSREF_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_POSREF_TESTS] = {2, 3, 0, 8};
    uint8_t refType[NUM_POSREF_TESTS] = {0, 1, 0, 1};
    RS232_MSG exp_frame[NUM_POSREF_TESTS] = {
        {.RS232_data = {0x08, 0x00, 0x20, 0x59, 0x09, 0xFF, 0xFF, 0x20, 0x00, 0xA8}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x40, 0x59, 0x09, 0xDF, 0xFF, 0x00, 0x00, 0x98}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x00, 0x59, 0x09, 0xFF, 0xFF, 0x20, 0x00, 0x98}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0x59, 0x09, 0xDF, 0xFF, 0x00, 0x00, 0xC8}, .length = 10}
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_POSREF_TESTS; i++) {
        RS232_MSG frame; 
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetPosRef(&frame, &id, refType[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_POSREF_TESTS);
    return;
}

#define NUM_MASTERID_TESTS      4
static void Test_SetMasterId()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_MASTERID_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_MASTERID_TESTS] = {243, 2, 0, 8};
    uint8_t newId = 8;
    RS232_MSG exp_frame[NUM_MASTERID_TESTS] = {
        {.RS232_data = {0x06, 0x0F, 0x30, 0x23, 0x27, 0x00, 0x08, 0x97}, .length = 8},
        {.RS232_data = {0x06, 0x10, 0x20, 0x23, 0x27, 0x00, 0x08, 0x88}, .length = 8},
        {.RS232_data = {0x06, 0x10, 0x00, 0x23, 0x27, 0x00, 0x08, 0x68}, .length = 8},
        {.RS232_data = {0x06, 0x00, 0x80, 0x23, 0x27, 0x00, 0x08, 0xD8}, .length = 8},
    };

    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_MASTERID_TESTS; i++) {
        RS232_MSG frame; 
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetMasterId(&frame, &id, newId);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_MASTERID_TESTS);
    return;
}

#define NUM_BAUDRATE_TESTS      1
static void Test_SetBaudRate()
{
    uint8_t passed = 0;
    serial_baudrate_t rates[NUM_BAUDRATE_TESTS] = {BAUDRATE_115200};
    RS232_MSG exp_frame[NUM_BAUDRATE_TESTS] = {
        {.RS232_data = {0x06, 0x10, 0x00, 0x08, 0x20, 0x00, 0x04, 0x4D}, .length = 8}
    };
    
    printf("*** %s Results ***\n", __func__);
    for(int i = 0; i < NUM_BAUDRATE_TESTS; i++) {
        RS232_MSG frame;
        FormatSetBaudRate(&frame, rates[i]);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_BAUDRATE_TESTS);
    return;
}

#define NUM_UPDATE_TESTS        4
static void Test_UpdPosn()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_UPDATE_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_UPDATE_TESTS] = {112, 3, 0, 8};
    RS232_MSG exp_frame[NUM_UPDATE_TESTS] = {
        {.RS232_data = {0x04, 0x07, 0x00, 0x01, 0x08, 0x14}, .length = 6},
        {.RS232_data = {0x04, 0x10, 0x40, 0x01, 0x08, 0x5D}, .length = 6},
        {.RS232_data = {0x04, 0x10, 0x00, 0x01, 0x08, 0x1D}, .length = 6},
        {.RS232_data = {0x04, 0x00, 0x80, 0x01, 0x08, 0x8D}, .length = 6}
    };
    
    printf("*** %s Testing ***\n", __func__);
    for(int i = 0; i < NUM_UPDATE_TESTS; i++) {
        RS232_MSG frame; 
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatUpdatePosn(&frame, &id);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_UPDATE_TESTS);
    return;
}

#define NUM_SETSYNC_TESTS       4
static void Test_SetSync()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_SETSYNC_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_SETSYNC_TESTS] = {2, 3, 0, 8};

    RS232_MSG exp_frame[NUM_SETSYNC_TESTS] = {
        {.RS232_data = {0x08, 0x00, 0x20, 0x14, 0x04, 0x00, 0x14, 0x00, 0x00, 0x54}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x40, 0x14, 0x04, 0x00, 0x14, 0x00, 0x00, 0x84}, .length = 10},
        {.RS232_data = {0x08, 0x10, 0x00, 0x14, 0x04, 0x00, 0x14, 0x00, 0x00, 0x44}, .length = 10},
        {.RS232_data = {0x08, 0x00, 0x80, 0x14, 0x04, 0x00, 0x14, 0x00, 0x00, 0xB4}, .length = 10}
    };

    printf("*** %s Testing ***\n", __func__);
    for(int i = 0; i < NUM_SETSYNC_TESTS; i++) {
        RS232_MSG frame; 
        motor_id_t id = {.type = idTypes[i], .id = ids[i]};
        FormatSetSync(&frame, &id);
        if(msg_comp(&frame, &(exp_frame[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_SETSYNC_TESTS);
    return;
}

/*#define NUM_PING_TESTS      4
static void Test_Ping()
{
    uint8_t passed = 0;
    id_type_t idTypes[NUM_PING_TESTS] = {ID_TYPE_AXIS, ID_TYPE_GROUP, ID_TYPE_BROADCAST, ID_TYPE_AXIS};
    uint8_t ids[NUM_PING_TESTS] = {243, 1, 0, 8};
    RS232_MSG exp_frame[NUM_PING_TESTS] = {
        {.RS232_data = {}, .length = 8},
    };
}*/
typedef struct
{
    uint8_t id;
    uint16_t reg_addr;
    uint32_t data;
} ret_t;
typedef enum {
    IDX_ID,
    IDX_DATA,
    IDX_REG,
    NUM_IDX,
} cmp_t;
static bool ret_cmp(ret_t* act_ret, ret_t* exp_ret)
{
    char err_str[3][BUFF_SIZE];
    
    int cnt = 0;
    if(act_ret->id != exp_ret->id) {
        snprintf(err_str[cnt], BUFF_SIZE, "ID(exp=%d act=%d)", exp_ret->id, act_ret->id);
        cnt++;
    }
    if(act_ret->data != exp_ret->data) {
        snprintf(err_str[cnt], BUFF_SIZE, "Data(exp=0x%x act=0x%x)", exp_ret->data, act_ret->data);
        cnt++;
    }
    if(act_ret->reg_addr != exp_ret->reg_addr) {
        snprintf(err_str[cnt], BUFF_SIZE, "Reg(exp=0x%x act=0x%x)", exp_ret->reg_addr, act_ret->reg_addr);
        cnt++;
    }

    if(cnt > 0) {
        printf("Parse failure: \n");
        for(int i = 0; i < cnt; i++) {
            printf("\t%s\n", err_str[i]);
        }
        return false;
    }  
    return true;
}

#define NUM_TAKEDATA_TESTS      3
static void Test_ParseTakeData()
{
    uint8_t passed = 0;
    RS232_MSG recv_frame[NUM_TAKEDATA_TESTS] = {
        {.RS232_data = {0x4F, 0x0C, 0x00, 0x81, 0xB4, 0x05, 0x02, 0x20, 0x02, 0xA2, 0xCE, 0xBF, 0xFB, 0xB5, 0x49}, .length = 15},
        {.RS232_data = {0x4F, 0x0A, 0x00, 0x81, 0xB4, 0x04, 0x03, 0xA0, 0x02, 0x2A, 0xFC, 0x16, 0x24}, .length = 13},
        {.RS232_data = {0x4F, 0x0C, 0x00, 0x81, 0xB4, 0x05, 0x00, 0x80, 0x09, 0x94, 0xD5, 0xA8, 0x28, 0x09, 0x11}, .length = 15},
    };

    ret_t exp_ret[NUM_TAKEDATA_TESTS] = {
        {.id = 34, .data = DoubleToFixed(-1098.1924), .reg_addr = REG_CACC},
        {.id = 58, .data = (uint16_t)(-1002), .reg_addr = REG_POSERR},
        {.id = 8, .data = DoubleToFixed(10249.8346), .reg_addr = REG_HOMESPD},
    };

    printf("*** %s Testing ***\n", __func__);
    for(int i = 0; i < NUM_TAKEDATA_TESTS; i++) {
        ret_t act_ret;
        if(ParseResponse(&recv_frame[i], &act_ret.data, &act_ret.id, &act_ret.reg_addr) < 0) {
            printf("test %d parse error\n", i);
        } else if(ret_cmp(&act_ret, &(exp_ret[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_TAKEDATA_TESTS);
    return;
}

#define NUM_TAKEDATA2_TESTS     3
static void Test_ParseTakeData2()
{
    uint8_t passed = 0;
    RS232_MSG recv_frame[NUM_TAKEDATA2_TESTS] = {
        {.RS232_data = {0x4F, 0x0A, 0x00, 0x81, 0xD5, 0x22, 0x02, 0xA2, 0xCE, 0xBF, 0xFB, 0xB5, 0x63}, .length = 13},
        {.RS232_data = {0x4F, 0x08, 0x00, 0x81, 0xD4, 0x3A, 0x02, 0x2A, 0xFC, 0x16, 0xD5}, .length = 11},
        {.RS232_data = {0x4F, 0x0A, 0x00, 0x81, 0xD5, 0x08, 0x09, 0x94, 0xD5, 0xA8, 0x28, 0x09, 0xB3}, .length = 13},
    };

    ret_t exp_ret[NUM_TAKEDATA2_TESTS] = {
        {.id = 34, .data = DoubleToFixed(-1098.1924), .reg_addr = REG_CACC},
        {.id = 58, .data = (uint16_t)(-1002), .reg_addr = REG_POSERR},
        {.id = 8, .data = DoubleToFixed(10249.8346), .reg_addr = REG_HOMESPD},
    };

    printf("*** %s Testing ***\n", __func__);
    for(int i = 0; i < NUM_TAKEDATA2_TESTS; i++) {
        ret_t act_ret;
        if(ParseResponse(&recv_frame[i], &act_ret.data, &act_ret.id, &act_ret.reg_addr) < 0) {
            printf("test %d parse error\n", i);
        } else if(ret_cmp(&act_ret, &(exp_ret[i])) == false) {
            printf("%s failed test %d\n", __func__, i);
        } else {
            passed++;
        }
    }

    printf("%s passed %d of %d tests\n\n\n", __func__, passed, NUM_TAKEDATA2_TESTS);
    return;
}

int main()
{
    InitLib(TEST_HOSTID, BAUDRATE_115200);
    Test_GiveMeData();
    Test_GiveMeData2();
    Test_SetVal16();
    Test_SetVal32();
    Test_GoTo();
    Test_STA();
    Test_SetAxisControl();
    Test_TUM();
    Test_SetMotionMode();
    Test_SetPosRef();
    Test_SetMasterId();
    Test_SetBaudRate();
    Test_UpdPosn();
    Test_SetSync();
    Test_ParseTakeData();
    Test_ParseTakeData2();
}
