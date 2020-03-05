#ifndef TML_RS232_DEF_H
#define TML_RS232_DEF_H

// Message building
#define AXIS_ID_CODE(id)            (((uint16_t)id << 4))
#define GROUP_ID_CODE(id)           (((uint16_t)1 << (id-1)<<4)|(1<<12))
#define BROADCAST_ID_CODE           ((uint16_t)(1 << 12))
#define CSUM_MODULO                 (256)

// Common helpers
#undef LOBYTE
#define LOBYTE(x)                   ((uint8_t)(x))
#undef HIBYTE
#define HIBYTE(x)                   ((uint8_t)((uint16_t)(x) >> 8))
#define MIDBYTE(low, hi)            ((uint8_t)(((hi & 0x0Fu) << 4)|(low >> 4)))
#define LSB9_MASK                   (0x01FFu)
#define HOST_TO_MASTER(id)          ((uint16_t)(id)*16+1)                           // +1 sets the host bit
#define FIXED_POINT_FRACT_BITS      (16)
#define GROUP_BIT_OFFSET            (12)

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
#define OPT_SCIBR            (uint16_t)(0x0820u)
#define OPT_PING             (uint16_t)(0xD600u)
#define OPT_PONG             (uint16_t)(0xD600u)
#define OPT_UPD              (uint16_t)(0x0108u)
#define OPT_SETSYNC          (uint16_t)(0x1404u)
#define OPT_EXTREF           (uint16_t)(0x5909u)

// Motion Modes
#define MODE_PP_PAYLOAD      (uint32_t)(0x8701BFC1u)
#define MODE_PP1_PAYLOAD     (uint32_t)(0x8501BDC1)
#define MODE_PP3_PAYLOAD     MODE_PP_PAYLOAD
#define MODE_TEF             (uint32_t)(0x8120B1E0u)
#define MODE_TES             (uint32_t)(0x8100B1C0u)
#define SYNC_MSG_PERIOD      (uint32_t)(20) // ms

// External Ref Mode
#define EXTREF_ONLINE_PAYLOAD   (uint32_t)(0xFF3F0000u)
#define EXTREF_ANALOG_PAYLOAD   (uint32_t)(0xFF7F0040u)
#define EXTREF_DIGITAL_PAYLOAD  (uint32_t)(0xFFBF0080u)

// Memory Types
#define TM_DATA              (0x4u)

// Expected Size of payload
#define TAKE_32BIT_SIZE      (8)
#define TAKE_16BIT_SIZE      (6)
#define TAKE2_32BIT_SIZE     (6)
#define TAKE2_16BIT_SIZE     (4)

// Receive Message Types
#define TAKE_DATA           (0xB4)                // TakeData1 message High Byte
#define TAKE_DATA2_16       (0xD4)                // TakeData2 (16 bits) message High Byte
#define TAKE_DATA2_32       (0xD5)                // TakeData2 (16 bits) message High Byte

// Misc
#define MAX_GROUP_ID        (8)                   // max possible group id
#define PING_LATENCY_115200 (2000)                // latency requred for ping at baudrate = 115200

// Offsets to get info from payload
typedef enum {
    OFFSET_LENGTH,
    OFFSET_IDCODE_HIGH,
    OFFSET_IDCODE_LOW,
    OFFSET_OPTCODE_HIGH,
    OFFSET_OPTCODE_LOW,
    OFFSET_DATA_WORD1_HIGH,
    OFFSET_DATA_WORD1_LOW,
    OFFSET_DATA_WORD2_HIGH,
    OFFSET_DATA_WORD2_LOW,
    OFFSET_DATA_WORD3_HIGH,
    OFFSET_DATA_WORD3_LOW,
    OFFSET_DATA_WORD4_HIGH,
    OFFSET_DATA_WORD4_LOW,
    OFFSET_CHECKSUM,
    RS232_MAX_BYTES,
} offset_t;

#endif