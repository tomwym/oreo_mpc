#ifndef TML_CAN_DEF_H
#define TML_CAN_DEF_H

#include <stdint.h>

// Parsing information from Can ID
#define LOW_OPTCODE_MASK            (0x1FF)
#define OPT_CODE(x)                 (((uint16_t)(x >> 22)) << 9)|(x & LOW_OPTCODE_MASK)
#define DEST_ID(x)                  (uint8_t)(x >> 13)
#define IS_GROUP(x)                 (uint8_t)((x >> 21) & 0x1u)
#define HOST_BIT(x)                 (uint8_t)((x >> 9) & 0x1u)

// Build message
#define AXIS_ID_CODE(id)            (((uint16_t)id << 4))
#define GROUP_ID_CODE(id)           (((uint16_t)1 << id)|(1<<12))
#define BROADCAST_ID_CODE           ((uint16_t)(1 << 12))
#define TML_CAN_ID(opt, id)             ((((uint32_t)(opt & 0xFE00u)) << 13) | ((uint32_t)id << 9) | (opt & 0x01FFu))

// Common helpers
#undef LOBYTE
#define LOBYTE(x)                   ((uint8_t)(x))
#undef HIBYTE
#define HIBYTE(x)                   ((uint8_t)((uint16_t)(x) >> 8))
#define MIDBYTE(low, hi)            ((uint8_t)(((hi & 0x0Fu) << 4)|(low >> 4)))
#define LSB9_MASK                   (0x01FFu)
#define HOST_TO_MASTER(id)          ((uint16_t)(id)*16+1)

// Receive Message Types
#define TAKE_DATA           (0xB4)                // TakeData1 message High Byte
#define TAKE_DATA2_16       (0xD4)                // TakeData2 (16 bits) message High Byte
#define TAKE_DATA2_32       (0xD5)                // TakeData2 (16 bits) message High Byte

// Expected Size of payload
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
#define TUM0_PAYLOAD         (uint32_t)(0x0000BFFFu)
#define TUM1_PAYLOAD         (uint32_t)(0x4000FFFFu)
#define OPT_MODE             (uint16_t)(0x5909u)
#define OPT_POSREF           (uint16_t)(0x5909u)
#define CPR_PAYLOAD          (uint32_t)(0x0000DFFFu)
#define CPA_PAYLOAD          (uint32_t)(0x2000FFFFu)
#define OPT_CANBR            (uint16_t)(0x0804u)
#define OPT_PING             (uint16_t)(0xD600u)
#define OPT_PONG             (uint16_t)(0xD600u)
#define OPT_UPD              (uint16_t)(0x0108u)
#define OPT_SETSYNC          (uint16_t)(0x1401u)

// Motion Modes
#define MODE_PP_PAYLOAD      (uint32_t)(0x8701BFC1u)
#define MODE_PP1_PAYLOAD     (uint32_t)(0x8501BDC1)
#define MODE_PP3_PAYLOAD     MODE_PP_PAYLOAD

// Memory Types
#define TM_DATA              (0x4u)

// Common instruction pointer addresses
#define MOTION_LOOP_IP		0x4022u     // Motion(eye) loop
#define WAIT_LOOP_IP		0x401Bu     // Wait(eye) loop
#define MOTION_LOOP_NECK_IP	0x403Bu     // Motion(neck) loop
#define WAIT_LOOP_NECK_IP	0x4034u     // Wait(neck) loop
#define REV_CAL_IP		    0x4061u     // Reverse calibration
#define FOR_CAL_IP		    0x4027u     // Forward calibration
#define CAL_RUN_VAR		    0x03B3u     // Calibration has been run
#define CAL_APOS2_OFF_VAR	0x03B0u     // 

// CAN bus baudrates
#define CAN_BAUDRATE_125KB        (0xFC63)
#define CAN_BAUDRATE_250KB        (0x7363)
#define CAN_BAUDRATE_500KB        (0x3273)
#define CAN_BAUDRATE_800KB        (0x412A)
#define CAN_BAUDRATE_1MB          (0x1273)

// Misc
#define MAX_GROUP_ID        (8)



#endif