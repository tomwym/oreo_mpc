/*
 * TML_instructions.h
 *
 *  Copyright (c) 2018, TECHNOSOFT
 *  All rights reserved.
 *  Created on: Apr 9, 2015
 *  Author: g_blujdea
 *  Version: 1.0
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

#ifndef INCLUDE_TML_INSTRUCTIONS_H_
#define INCLUDE_TML_INSTRUCTIONS_H_

#define EEPROM_LOWER_ADDR	0x4000
#define EEPROM_UPPER_ADDR	0x4FFF	/*for iPOS360x*/

#define DOWNLOAD_POINTER		0x032b

#define READ_16_BIT_RAM		0xB204
#define READ_32_BIT_RAM		0xB205

#define READ_16_BIT_EEPROM	0xB008
#define READ_32_BIT_EEPROM	0xB009

#define WRITE_16_BIT_EEPROM		0x90A8
#define WRITE_16_BIT_EEPROM_A	0x9028	//write operation with autoincrement of the pointer

#define WRITE_32_BIT_EEPROM		0x90A9
#define WRITE_32_BIT_EEPROM_A	0x9029	//write operation with autoincrement of the pointer

#define WRITE_16_BIT_RAM		0x2000
#define WRITE_32_BIT_RAM		0x2400

#define RAM_PAGE_800			0x0800
#define WRITE_RAM_PAGE_800_BIT	0x0800


#define AXISON_OP_CODE	0x0102
#define AXISOFF_OP_CODE	0x0002

#define ENDINIT_OP_CODE	0x0020

#define CALL_FUNCTION 		0x7401
#define CALL_FUNCTION_P 	0x7601

#define CALLS_FUNCTION		0x1C01
#define CALLS_FUNCTION_P	0x1E01

#define SET_RESET_OUT		0xEC00

#define SET_CAN_BR			0x0804

#endif /* INCLUDE_TML_INSTRUCTIONS_H_ */
