/*
 *  TML_Lib_light.c
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

#include <stdint.h>
#include "./include/TML_lib_light.h"
#include "./include/TML_instructions.h"

bool TSL_ReadStatus(uint16_t AxisID, uint16_t Register, uint16_t * RegisterValue)
{
	#ifndef RS232_COM
		CAN_MSG CAN_RX_message;
		CAN_MSG CAN_TX_message;
	#else
		RS232_MSG RS232_TX_message;
		RS232_MSG RS232_RX_message;
	#endif

	TML_INSTR TML_instruction;

	TML_instruction.opCode = READ_16_BIT_RAM;
	TML_instruction.length = 2; /*Data request instructions have 2 words of data*/
#ifndef RS232_COM
	TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
    TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
    RS232_RX_message.length = 10;
#endif
	switch(Register){
		case REG_SRL:
			TML_instruction.TML_data[1] = SRL_ADDR;
			break;
		case REG_SRH:
			TML_instruction.TML_data[1] = SRH_ADDR;
			break;
		case REG_MER:
			TML_instruction.TML_data[1] = MER_ADDR;
			break;
		case REG_DER:
			TML_instruction.TML_data[1] = DER_ADDR;
			break;
		default:
			return false;
			break;
	}
	
		/*Send the message with the TML instruction - HW dependent*/
#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		if (!SendMessage(&RS232_TX_message))
			return false;	
#endif

	/*Wait for drive to reply*/

	/*Decode the received message - HW dependent*/
	#ifndef RS232_COM
		if (!ReceiveMessage(&CAN_RX_message))
			return false;
		*RegisterValue = ((uint16_t)CAN_RX_message.CAN_data[3] << 8) | CAN_RX_message.CAN_data[2];

	#else
		if (!ReceiveMessage(&RS232_RX_message))
			return false;
		*RegisterValue = ((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8];
	#endif

	return true;

}

bool TSL_InitializeDrive(uint16_t AxisID)
{
#ifndef RS232_COM
		CAN_MSG CAN_TX_message;
	#else
		RS232_MSG RS232_TX_message;
	#endif

	TML_INSTR TML_instruction;

	TML_instruction.opCode = ENDINIT_OP_CODE;
	TML_instruction.length = 0; /*ENDINIT has no data words*/
		
		/*Send the message with the TML instruction - HW dependent*/
	#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
	#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);		
		if (!SendMessage(&RS232_TX_message))	
			return false;
	#endif	

	return true;
}

bool TSL_CheckSetupTable(uint16_t AxisID, bool *SetupTableStatus)
{
	#ifndef RS232_COM
		CAN_MSG CAN_RX_message;
		CAN_MSG CAN_TX_message;
	#else
		RS232_MSG RS232_TX_message;
		RS232_MSG RS232_RX_message;
	#endif

	TML_INSTR TML_instruction;

	TML_instruction.opCode = READ_16_BIT_RAM;
	TML_instruction.length = 2; /*Data request instructions has 2 data words*/
#ifndef RS232_COM
	TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
    TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
#endif
	TML_instruction.TML_data[1] = MER_ADDR;
	
	/*Send the message with the TML instruction - HW dependent*/
	#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
	#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		if (!SendMessage(&RS232_TX_message))	
			return false;
	#endif

	/*Wait for drive to reply*/
	
	/*Decode the received message - HW dependent*/
	#ifndef RS232_COM
		if (!ReceiveMessage(&CAN_RX_message))
			return false;
		*SetupTableStatus = !((((uint16_t)CAN_RX_message.CAN_data[3] << 8) | CAN_RX_message.CAN_data[2]) & 0x0004);
	#else
		if (!ReceiveMessage(&RS232_RX_message))
			return false;		
		*SetupTableStatus = (bool)!((((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8]) & 0x0004);
	#endif
	return true;
}

bool TSL_Power(uint16_t AxisID, bool PowerSwitch)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
#endif

	TML_INSTR TML_instruction;

	TML_instruction.opCode = (PowerSwitch == false) ? AXISOFF_OP_CODE : AXISON_OP_CODE;
	TML_instruction.length = 0; /*AXISON/AXISOFF has 1 data word*/
	
	/*Send the message with the TML instruction - HW dependent*/
#ifndef RS232_COM
	TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
	if (!SendMessage(&CAN_TX_message))
		return false;
#else
	TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
	if (!SendMessage(&RS232_TX_message))
		return false;	
#endif

	return true;
}

bool TSL_ReadFunctionsTable(uint16_t AxisID, uint16_t* FunctionsAddresses, uint8_t* FunctionNo)
{
#ifndef RS232_COM
	CAN_MSG CAN_RX_message;
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
	RS232_MSG RS232_RX_message;
#endif
	TML_INSTR TML_instruction;
	uint16_t FunctionTableEEPROM = 0;
	uint16_t tmpFunctionAddress;

	/*Read function table pointer*/
	TML_instruction.opCode = READ_16_BIT_RAM;
	TML_instruction.length = 2; /*Data request instructions has 2 data words*/
#ifndef RS232_COM
	TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
    TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
#endif
	TML_instruction.TML_data[1] = FUNCTION_TABLE_POINTER;

	/*Build CAN message to read function table pointer from RAM*/
	/*Send the message with the TML instruction - HW dependent*/
#ifndef RS232_COM
	TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
	if (!SendMessage(&CAN_TX_message))
		return false;
#else
	TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
	if (!SendMessage(&RS232_TX_message))
		return false;	
#endif

	/*Wait for drive to reply*/

	/*Decode the received message - HW dependent*/
#ifndef RS232_COM
	if ((CAN_RX_message.identifier & 0x000000FF) == AxisID)
	{
		if (!ReceiveMessage(&CAN_RX_message))
			return false;

		FunctionTableEEPROM = ((uint16_t)CAN_RX_message.CAN_data[3] << 8)|CAN_RX_message.CAN_data[2];
	}
#else
	if (RS232_RX_message.RS232_data[4] == AxisID)
	{
		if (!ReceiveMessage(&RS232_RX_message))
			return false;		

		FunctionTableEEPROM = ((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8];
	}
#endif

	if (!FunctionTableEEPROM)
	{
		/*No functions defined on the drive*/
		return false;
	}

	for (uint8_t j=0; j<10;j++)
	{
		TML_instruction.opCode = READ_16_BIT_EEPROM;
		TML_instruction.length = 2; /*Data request instructions has 2 words*/
#ifndef RS232_COM
		TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
        TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
#endif
		TML_instruction.TML_data[1] = FunctionTableEEPROM;
		/*Send the message with TML instruction - HW dependent*/		
#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		if (!SendMessage(&RS232_TX_message))
			return false;	
#endif

	/*Wait for drive to reply*/

	/*Decode the received message - HW dependent*/
#ifndef RS232_COM
		if ((CAN_RX_message.identifier & 0x000000FF) == AxisID)
		{		
			if (!ReceiveMessage(&CAN_RX_message))
				return false;

			tmpFunctionAddress = ((uint16_t)CAN_RX_message.CAN_data[3] << 8) | CAN_RX_message.CAN_data[2];

			if(tmpFunctionAddress == 0)
			{
				/*No more function address*/
				break;
			}

			FunctionsAddresses[j]= tmpFunctionAddress;

			*FunctionNo = *FunctionNo + 1;
			FunctionTableEEPROM++; /*increment EEPROM pointer for next location*/
		}
	}
#else
		if (RS232_RX_message.RS232_data[4] == AxisID)
		{
			if (!ReceiveMessage(&RS232_RX_message))
				return false;		

			tmpFunctionAddress = ((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8];
			if (tmpFunctionAddress == 0)
			{
				/*No more function address*/
				break;
			}

			FunctionsAddresses[j] = tmpFunctionAddress;

			*FunctionNo = *FunctionNo + 1;
			FunctionTableEEPROM++; /*increment EEPROM pointer for next location*/
		}
	}
#endif
	return true;

}


bool TSL_StartFunction(uint16_t AxisID, uint16_t FunctionAddress)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
#endif

	TML_INSTR TML_instruction;

	if(!FunctionAddress || FunctionAddress < EEPROM_LOWER_ADDR || FunctionAddress> EEPROM_UPPER_ADDR)
	{
		/*if FunctionAddress is zero or outside of EEPROM boundries terminate function*/
		return false;
	}

	TML_instruction.opCode = CALL_FUNCTION;
	TML_instruction.length = 1; /*CALL has 1 data word*/
	TML_instruction.TML_data[0] = FunctionAddress;

	/*Send the message with TML instruction - HW dependent*/
#ifndef RS232_COM
	TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
	if (!SendMessage(&CAN_TX_message))
		return false;
#else
	TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
	if (!SendMessage(&RS232_TX_message))
		return false;	
#endif

	return true;
}

bool TSL_ExecuteTML(uint16_t AxisID, uint16_t OpCode, uint16_t TMLData1, uint16_t TMLData2, uint16_t TMLData3, uint16_t TMLData4, uint8_t NOWords)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message; 
#else
	RS232_MSG RS232_TX_message;
#endif
	TML_INSTR TML_instruction;

	TML_instruction.opCode = OpCode;
	TML_instruction.length = NOWords;
	TML_instruction.TML_data[0] = TMLData1;
	TML_instruction.TML_data[1] = TMLData2;
	TML_instruction.TML_data[2] = TMLData3;
	TML_instruction.TML_data[3] = TMLData4;

	/*Send the message with TML instruction - HW dependent*/
#ifndef RS232_COM
	TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
	if (!SendMessage(&CAN_TX_message))
		return false;
#else
	TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
	if (!SendMessage(&RS232_TX_message))
		return false;	
#endif

	return true;
}

bool TSL_SetOutput(uint16_t AxisID, uint8_t nIO, uint8_t OutValue)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
#endif
	TML_INSTR TML_instruction;

	if (nIO > 3)
	{
		return false;
	}

	TML_instruction.opCode = SET_RESET_OUT;
	TML_instruction.length = 2;/*OUT instruction has 2 data word*/
	TML_instruction.TML_data[0] = 1 << nIO;
	TML_instruction.TML_data[1] = (OutValue == 0) ? 0: 1;
	
	/*Send the message with TML instruction - HW dependent*/
#ifndef RS232_COM
	TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
	if (!SendMessage(&CAN_TX_message))
		return false;
#else
	TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
	if (!SendMessage(&RS232_TX_message))
		return false;	
#endif

	return true;
}

bool TSL_ReadInput(uint16_t AxisID, uint8_t nIO, bool* InValue)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message;
	CAN_MSG CAN_RX_message;
#else
	RS232_MSG RS232_TX_message;
	RS232_MSG RS232_RX_message;
#endif
	TML_INSTR TML_instruction;

	if (nIO > 5)
	{
		return false;
	}

	TML_instruction.opCode = READ_16_BIT_RAM;
	TML_instruction.length = 2;/*Data request instructions has 2 data word*/
#ifndef RS232_COM
	TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
    TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
    RS232_RX_message.length = 10;
#endif
	TML_instruction.TML_data[1] = INSTATUS_ADDR;

	/*Send the message with TML instruction - HW dependent*/
#ifndef RS232_COM
	TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
	if (!SendMessage(&CAN_TX_message))
		return false;
#else
	TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
	if (!SendMessage(&RS232_TX_message))
		return false;	
#endif

	/*Wait for drive to reply*/
	
	/*Decode the received message - HW dependent*/
#ifndef RS232_COM
	if ((CAN_RX_message.identifier & 0x000000FF) == AxisID)
	{
		if (!ReceiveMessage(&CAN_RX_message))
			return false;
		*InValue = (bool)((((uint16_t)CAN_RX_message.CAN_data[3] << 8) | CAN_RX_message.CAN_data[2]) & (uint16_t)(1<<nIO));
	}
#else
	if (RS232_RX_message.RS232_data[4] == AxisID)
	{
		if (!ReceiveMessage(&RS232_RX_message))
			return false;		
		*InValue = (bool)((((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8]) & (uint16_t)(1 << nIO));
	}
#endif
	return true;
}

bool TSL_Write16bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint16_t WriteValue)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
#endif
	TML_INSTR TML_instruction;

	if (MemoryAddress)
	{
		if ((MemoryAddress >= EEPROM_LOWER_ADDR) && (MemoryAddress <= EEPROM_UPPER_ADDR))
		{ 	//The write address is from EEPROM
			//EEPROM writes can be done only with indirect addressing via pointer variable

			//Set the pointer variable
			TML_instruction.opCode = WRITE_16_BIT_RAM | (0x01FF & DOWNLOAD_POINTER);
			TML_instruction.length = 1;/*Write instructions has 2 data word*/
			TML_instruction.TML_data[0] = MemoryAddress;
#ifndef RS232_COM
			TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
			/*Send the message with TML instruction - HW dependent*/
			if (!SendMessage(&CAN_TX_message))
				return false;
#else
			TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
			/*Send the message with TML instruction - HW dependent*/
			if (!SendMessage(&RS232_TX_message))
				return false;
#endif			

			//Write to the EEPROM
			TML_instruction.opCode = WRITE_16_BIT_EEPROM;
			TML_instruction.length = 2;/*Write instructions has 2 data word*/
			TML_instruction.TML_data[0] = DOWNLOAD_POINTER;
			TML_instruction.TML_data[1] = WriteValue;
		}
		else
		{
			TML_instruction.opCode = WRITE_16_BIT_RAM | (0x01FF & MemoryAddress);
			if (MemoryAddress > RAM_PAGE_800)
			{
				TML_instruction.opCode |= WRITE_RAM_PAGE_800_BIT;
			}
			TML_instruction.length = 1;/*Write instructions has 2 data words*/
			TML_instruction.TML_data[0] = WriteValue;
		}

		/*Send the message with TML instruction - HW dependent*/
#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		if (!SendMessage(&RS232_TX_message))
			return false;
#endif
		return true;
	}
	else
		return false;

}

bool TSL_Write32bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint32_t WriteValue)
{
#ifndef RS232_COM
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
#endif
	TML_INSTR TML_instruction;

	if (MemoryAddress)
	{
		if ((MemoryAddress >= EEPROM_LOWER_ADDR) && (MemoryAddress <= EEPROM_UPPER_ADDR))
		{ 	//The read address is from EEPROM
			//EEPROM writes can be done only with indirect addressing via pointer variable

			//Set the pointer variable
			TML_instruction.opCode = WRITE_16_BIT_RAM | (0x01FF & DOWNLOAD_POINTER);
			TML_instruction.length = 1;/*Write instructions has 2 data word*/
			TML_instruction.TML_data[0] = MemoryAddress;
#ifndef RS232_COM
			TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
			/*Send the message with TML instruction - HW dependent*/
			if (!SendMessage(&CAN_TX_message))
				return false;
#else
			TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
			/*Send the message with TML instruction - HW dependent*/
			if (!SendMessage(&RS232_TX_message))
				return false;
#endif

			//Write to the EEPROM
			TML_instruction.opCode = WRITE_32_BIT_EEPROM;
			TML_instruction.length = 3;/*Write instructions has 2 data word*/
			TML_instruction.TML_data[0] = DOWNLOAD_POINTER;
			TML_instruction.TML_data[1] = LOWORD(WriteValue);
			TML_instruction.TML_data[2] = HIWORD(WriteValue);
		}
		else
		{
			TML_instruction.opCode = WRITE_32_BIT_RAM | (0x01FF & MemoryAddress);
			if (MemoryAddress > RAM_PAGE_800)
			{
				TML_instruction.opCode |= WRITE_RAM_PAGE_800_BIT;
			}
			TML_instruction.length = 2;/*Write instructions has 2 data word*/
			TML_instruction.TML_data[0] = LOWORD(WriteValue);
			TML_instruction.TML_data[1] = HIWORD(WriteValue);
		}
		
		/*Send the message with TML instruction - HW dependent*/
#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		if (!SendMessage(&RS232_TX_message))
			return false;
#endif

		return true;
	}
	else
		return false;

}

bool TSL_Read16bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint16_t *ReadValue)
{
#ifndef RS232_COM
	CAN_MSG CAN_RX_message;
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
	RS232_MSG RS232_RX_message;
#endif
	TML_INSTR TML_instruction;

	if (MemoryAddress)
	{
		if ((MemoryAddress >= EEPROM_LOWER_ADDR) && (MemoryAddress <= EEPROM_UPPER_ADDR))
		{ 	//The read address is from EEPROM
			TML_instruction.opCode = READ_16_BIT_EEPROM;
		}
		else
		{
			TML_instruction.opCode = READ_16_BIT_RAM;
		}

		TML_instruction.length = 2; /*Data request instructions have 2 data word*/
#ifndef RS232_COM
		TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
        TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
        RS232_RX_message.length = 10;
#endif
		TML_instruction.TML_data[1] = MemoryAddress;

#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		/*Send the message with TML instruction - HW dependent*/
		if (!SendMessage(&CAN_TX_message))
			return false;
#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		/*Send the message with TML instruction - HW dependent*/
		if (!SendMessage(&RS232_TX_message))
			return false;
#endif

	/*Wait for drive to reply*/

	/*Decode the received message - HW dependent*/
	if ((MemoryAddress >= EEPROM_LOWER_ADDR) && (MemoryAddress <= EEPROM_UPPER_ADDR))
	{
#ifndef RS232_COM
		if (!ReceiveMessage(&CAN_RX_message))
			return false;
		*ReadValue = ((uint16_t)CAN_RX_message.CAN_data[5] << 8) | CAN_RX_message.CAN_data[4];
#else
		if (!ReceiveMessage(&RS232_RX_message))
			return false;		
		*ReadValue = ((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8];
#endif
	}
	else
	{	
#ifndef RS232_COM
		if (!ReceiveMessage(&CAN_RX_message))
			return false;		
		*ReadValue = ((uint16_t)CAN_RX_message.CAN_data[3] << 8) | CAN_RX_message.CAN_data[2];
#else
		if (!ReceiveMessage(&RS232_RX_message))
			return false;		
		*ReadValue = ((uint16_t)RS232_RX_message.RS232_data[7] << 8) | RS232_RX_message.RS232_data[8];
#endif
	}
		return true;
	}
	else
		return false;

}

bool TSL_Read32bitValue(uint16_t AxisID, uint16_t MemoryAddress, uint32_t *ReadValue)
{
#ifndef RS232_COM
	CAN_MSG CAN_RX_message;
	CAN_MSG CAN_TX_message;
#else
	RS232_MSG RS232_TX_message;
	RS232_MSG RS232_RX_message;
#endif
	TML_INSTR TML_instruction;

	if (MemoryAddress)
	{
		if ((MemoryAddress >= EEPROM_LOWER_ADDR) && (MemoryAddress <= EEPROM_UPPER_ADDR))
		{ 	//The read address is from EEPROM
			TML_instruction.opCode = READ_32_BIT_EEPROM;
		}
		else
		{
			TML_instruction.opCode = READ_32_BIT_RAM;
		}

		TML_instruction.length = 2; /*Data request instructions have 2 data word*/
#ifndef RS232_COM
		TML_instruction.TML_data[0] = MASTER_ID << 4;
#else
        TML_instruction.TML_data[0] = (MASTER_ID << 4) | HOST_BIT;
        RS232_RX_message.length = 12;
#endif
		TML_instruction.TML_data[1] = MemoryAddress;
		
		/*Send the message with the TML instruction - HW dependent*/
#ifndef RS232_COM
		TSL_TML_to_TMLCAN(AxisID, &TML_instruction, &CAN_TX_message);
		if (!SendMessage(&CAN_TX_message))
			return false;
#else
		TSL_TML_to_RS232(AxisID, &TML_instruction, &RS232_TX_message);
		if (!SendMessage(&RS232_TX_message))
			return false;	
#endif

		/*Wait for drive to reply*/

		/*Decode the received message - HW dependent*/
		if ((MemoryAddress >= EEPROM_LOWER_ADDR) && (MemoryAddress <= EEPROM_UPPER_ADDR))
		{
#ifndef RS232_COM
			if (!ReceiveMessage(&CAN_RX_message))
				return false;
			*ReadValue = ((uint32_t)(((uint16_t)CAN_RX_message.CAN_data[7]) << 8 | CAN_RX_message.CAN_data[6]) << 16) | ((uint16_t)CAN_RX_message.CAN_data[5]) << 8 | CAN_RX_message.CAN_data[4];
#else
			if (!ReceiveMessage(&RS232_RX_message))
				return false;		
			*ReadValue = ((uint32_t)(((uint16_t)RS232_RX_message.RS232_data[9]) << 8 | RS232_RX_message.RS232_data[10]) << 16) | ((uint16_t)RS232_RX_message.RS232_data[7]) << 8 | RS232_RX_message.RS232_data[8];
#endif
		} 
		else
		{
#ifndef RS232_COM
			if (!ReceiveMessage(&CAN_RX_message))
				return false;
			*ReadValue = ((uint32_t)((CAN_RX_message.CAN_data[5]) << 8 | CAN_RX_message.CAN_data[4]) << 16) | (CAN_RX_message.CAN_data[3]) << 8 | CAN_RX_message.CAN_data[2];
#else
            if (!ReceiveMessage(&RS232_RX_message))
                return false;

            *ReadValue = ((uint32_t)(((uint16_t)RS232_RX_message.RS232_data[9]) << 8 | RS232_RX_message.RS232_data[10]) << 16) | ((uint16_t)RS232_RX_message.RS232_data[7]) << 8 | RS232_RX_message.RS232_data[8];
#endif

		}
			return true;
	}
	else
		return false;

}


bool TSL_TML_to_TMLCAN(uint16_t AxisID, TML_INSTR *TML_instruction, CAN_MSG *CAN_TX_message)
{

	 CAN_TX_message->identifier = ((((uint32_t)(TML_instruction->opCode & 0xFE00)) << 13) | ((uint32_t)AxisID << 13) | (TML_instruction->opCode & 0x01FF));
	 CAN_TX_message->length = TML_instruction->length * 2;

	 for (int j=0; j < TML_instruction->length;j++)
	 {
		 CAN_TX_message->CAN_data[2*j] = LOBYTE(TML_instruction->TML_data[j]);
		 CAN_TX_message->CAN_data[2*j+1] = HIBYTE(TML_instruction->TML_data[j]);
	 }

	return true;
}

bool TSL_TML_to_RS232(uint16_t AxisID, TML_INSTR *TML_instruction, RS232_MSG *RS232_TX_message)
{
	unsigned int checksum = 0;
	int i;

	RS232_TX_message->length = TML_instruction->length * 2 + RS232_HEADER;

	/*The message length sent to the drive doesn't contain the length byte and checksum byte */
	RS232_TX_message->RS232_data[0] = RS232_TX_message->length - 2;

	RS232_TX_message->RS232_data[1] = HIBYTE(AxisID << 4);
	RS232_TX_message->RS232_data[2] = LOBYTE(AxisID << 4);
	RS232_TX_message->RS232_data[3] = HIBYTE(TML_instruction->opCode);
	RS232_TX_message->RS232_data[4] = LOBYTE(TML_instruction->opCode);

	for (int j=0; j < TML_instruction->length;j++)
	{
		RS232_TX_message->RS232_data[2 * j + 5] = HIBYTE(TML_instruction->TML_data[j]);
		RS232_TX_message->RS232_data[2 * j + 6] = LOBYTE(TML_instruction->TML_data[j]);
	}

	for (i = RS232_TX_message->RS232_data[0]; i >= 0; i--)
	{
		checksum += RS232_TX_message->RS232_data[i];
	}
	RS232_TX_message->RS232_data[RS232_TX_message->length-1] = (uint8_t)checksum;

	return true;
}

