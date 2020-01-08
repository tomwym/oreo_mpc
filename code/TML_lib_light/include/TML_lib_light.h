/*
 *  TML_Lib_light.h
 *
 *  Copyright (c) 2018, TECHNOSOFT
 *  All rights reserved.
 *  Created on: Apr 9, 2015
 *  Author: g_blujdea
 *  Version: 1.5
 *
 *
 *    Redistribution and use in source and binary forms, with or without
 *    modification, are permitted provided that the following conditions are met:
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in the
 *          documentation and/or other materials provided with the distribution.
 *        * Neither the name of the TECHNOSOFT nor the
 *          names of its contributors may be used to endorse or promote products
 *          derived from this software without specific prior written permission.
 *
 *    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *    DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INCLUDE_TML_LIB_LIGHT_H_
#define INCLUDE_TML_LIB_LIGHT_H_

#include <stdint.h>
#include <stdbool.h>

#define RS232_COM	// comment the definition for TMLCAN communication

#undef LOBYTE
#define LOBYTE(x)    ((uint8_t)(x))
#undef HIBYTE
#define HIBYTE(x)    ((uint8_t)((uint16_t)(x) >> 8))

#undef LOWORD
#define LOWORD(x)    ((uint16_t)(x))
#undef HIWORD
#define HIWORD(x)    ((uint16_t)((uint32_t)(x) >> 16))

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
	uint16_t opCode;
	uint8_t length;
	uint16_t TML_data[4];
}TML_INSTR;

typedef struct {
	uint32_t identifier;
	uint8_t length;
	uint8_t CAN_data[8];
}CAN_MSG;

typedef struct {
	uint8_t length;
	uint8_t RS232_data[12];
}RS232_MSG;

#define RS232_HEADER	6 // RS232 message header length: axis ID 2 bytes + operation code 2 bytes + checksum 1 byte + message length 1 byte
#define HOST_BIT		0x0001

/*CAN node ID of the PC*/
#define MASTER_ID	120

/*Constants used for the register for function TSL_ReadStatus*/
#define REG_SRL		0
#define REG_SRH		1
#define REG_SR		2
#define REG_MER		3
#define REG_DER		4

/*Constants for registers addresses*/
#define SRL_ADDR 	0x090E
#define SRH_ADDR	0x090F
#define SR_ADDR		0x090E
#define MER_ADDR	0x08FC
#define DER_ADDR	0x03AD

#define INSTATUS_ADDR	0x0908

#define FUNCTION_TABLE_POINTER 0x09C9

/*Constants used to select or set the group*/
#define GROUP_0		0
#define GROUP_1		1
#define GROUP_2		2
#define GROUP_3		3
#define GROUP_4		4
#define GROUP_5		5
#define GROUP_6		6
#define GROUP_7		7
#define GROUP_8		8


/***** PROTOTYPES *******************/
/*Translate TML instructions to TMLCAN messages*/
bool TSL_TML_to_TMLCAN(uint16_t AxisID, TML_INSTR *TML_instruction, CAN_MSG *CAN_message);

/*Translate TML instructions to RS232 messages*/
bool TSL_TML_to_RS232(uint16_t AxisID, TML_INSTR *TML_instruction, RS232_MSG *RS232_message);

/*The function reads a status register from the drive*/
bool TSL_ReadStatus(uint16_t AxisID, uint16_t Register, uint16_t *RegisterValue);

/*The function sends the ENDINIT instruction*/
bool TSL_InitializeDrive(uint16_t AxisID);

/*The function reads MER.2 to determine if the setup table is valid or not*/
bool TSL_CheckSetupTable(uint16_t AxisID, bool *SetupTableStatus);

/*The function enables/disables the power stage of the drive */
bool TSL_Power(uint16_t AxisID, bool PowerSwitch);

/*The function reads the functions table generated from EasyMotion Studio and stored in the non-volatile memory of the drive */
bool TSL_ReadFunctionsTable(uint16_t AxisID, uint16_t* FunctionsAddresses, uint8_t* FunctionNo);

/*The function triggers the execution of the TML function stored on the drive */
bool TSL_StartFunction(uint16_t AxisID, uint16_t FunctionAddress);

/*The function sends the TML command to the drive */
bool TSL_ExecuteTML(uint16_t AxisID, uint16_t OpCode, uint16_t TMLData1, uint16_t TMLData2, uint16_t TMLData3, uint16_t TMLData4, uint8_t NOWords);

/*The function set/reset the output of the drive */
bool TSL_SetOutput(uint16_t AxisID, uint8_t nIO, uint8_t OutValue);

/*The function reads the input of the drive */
bool TSL_ReadInput(uint16_t AxisID, uint8_t nIO, bool* InValue);

/*read INSTATUS variable from the drive and apply a mask on it to retrieve the input state*/

bool TSL_Write16bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint16_t writeValue);

bool TSL_Write32bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint32_t writeValue);

bool TSL_Read16bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint16_t* readValue);

bool TSL_Read32bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint32_t* readValue);

bool TSL_SetCANBaudRate(uint16_t AxisID, uint32_t Baudrate);

#ifndef RS232_COM
	extern bool SendMessage(CAN_MSG * CAN_TX_message);
	extern bool ReceiveMessage(CAN_MSG * CAN_RX_message);
#else
	extern bool SendMessage(RS232_MSG * RS232_TX_message);
	extern bool ReceiveMessage(RS232_MSG * RS232_RX_message);
#endif

#ifdef __cplusplus
}
#endif
    
#endif /* INCLUDE_TML_LIB_H_ */
