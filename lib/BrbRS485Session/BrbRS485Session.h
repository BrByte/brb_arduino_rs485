/*
 * BrbRS485Session.h
 *
 *  Created on: 2018-10-01
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
 *
 * Copyright (c) 2018 BrByte Software (Oliveira Alves & Amorim LTDA)
 * Todos os direitos reservados. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BRB_RS485_SESSION_H_
#define BRB_RS485_SESSION_H_
 /**********************************************************************************************************************/
#include "Arduino.h"
#include <BrbBase.h>
#include <log/BrbLogBase.h>
 /**********************************************************************************************************************/
#define TO_HEX(i) (i <= 9 ? ('0' + i) : ('A' - 10 + i))
#define TO_NUM(a, b) (((a <= '9' ? a - '0' : a - 'A' + 10) << 4) | (b <= '9' ? b - '0' : b - 'A' + 10))

#define RS485_TOGGLE_TRASMIT HIGH
#define RS485_TOGGLE_RECEIVE LOW

#define BRB_RS485_BEGIN '\02'
#define BRB_RS485_END 	'\03'
#define BRB_RS485_MAGIC '\05'

#define BRB_RS485_MAX_PKT_SZ 256

#define BRB_RS485_HANDSHAKE_REPEAT 1
#define BRB_RS485_HANDSHAKE_TIME 15555

// #define BRB_RS485_BAUDRATE 9600
#define BRB_RS485_BAUDRATE 19200
// #define BRB_RS485_BAUDRATE 115200
#define BRB_RS485_HARDWARE_SERIAL 1

#define BRB_RS485_EEPROM_OFFSET BRB_EEPROM_OFFSET + 32

 /**********************************************************************************************************************/
typedef int BrbRS485SessionActionCBH(void *base_ptr, int action_code, const void *data_str, unsigned int data_sz, void *cb_data_ptr);

 /**********************************************************************************************************************/
typedef struct _BrbRS485StatsData
{
	struct
	{
		uint32_t bcast;
		uint32_t srv;
		uint32_t me;
		uint32_t tx;
		uint32_t rx;
	} byte;

	struct
	{
		uint32_t bcast;
		uint32_t srv;
		uint32_t me;
		uint32_t tx;
		uint32_t rx;

		struct
		{
			uint32_t cmd_id;
			uint32_t cmd_no_bcast;
			uint32_t cmd_no_cb;
		} err;
	} pkt;

	struct
	{
		uint32_t crc;
		uint32_t pkt;
		uint32_t bad_char;
		uint32_t overflow;
	} err;
} BrbRS485StatsData;

typedef struct _BrbRS485SessionActionEvents
{
	BrbRS485SessionActionCBH *cb_func;
	void *cb_data;
} BrbRS485SessionActionEvents;

typedef enum
{
	RS485_PKT_TYPE_UNKOWN,
	RS485_PKT_TYPE_HANDSHAKE,
	RS485_PKT_TYPE_ACK,

	RS485_PKT_TYPE_CMD_GET_A,
	RS485_PKT_TYPE_CMD_GET_D,

	RS485_PKT_TYPE_CMD_SET_A,
	RS485_PKT_TYPE_CMD_SET_A_BMP,
	
	RS485_PKT_TYPE_CMD_SET_D,
	RS485_PKT_TYPE_CMD_SET_D_BMP,

	RS485_PKT_TYPE_CMD_SET_ID,

	RS485_PKT_TYPE_CMD_SET_SCRIPT,

	RS485_PKT_TYPE_REPLY,

	RS485_PKT_TYPE_LAST_ITEM

} BrbRS485PacketType;

typedef enum
{
	RS485_PKT_RETURN_ACK_SUCCESS,
	RS485_PKT_RETURN_ACK_FAIL,
	RS485_PKT_RETURN_QUIET,
	RS485_PKT_RETURN_LAST_ITEM

} BrbRS485PacketReply;

/* 8 Bytes = 64 bits */
typedef struct _BrbRS485PacketHdr
{
	uint8_t src;
	uint8_t dst;
	uint8_t magic;
	uint8_t type;
	uint8_t len;
	uint8_t pad0;

	uint16_t id;
} BrbRS485PacketHdr;

/* 12 Bytes = 96 bits - 64 HDR + 32 UUID */
typedef struct _BrbRS485PacketHandShake
{
	BrbRS485PacketHdr hdr;
	uint8_t uuid[4];
} BrbRS485PacketHandShake;

/* 12 Bytes = 96 bits - 64 HDR + 32 UUID */
typedef struct _BrbRS485PacketData
{
	BrbRS485PacketHdr hdr;
	/* This here can be used as map from 16 bits */
	uint32_t val;
} BrbRS485PacketData;

/* 12 Bytes = 96 bits - 64 HDR + 32 BODY */
typedef struct _BrbRS485PacketSetPin
{
	BrbRS485PacketHdr hdr;

	uint16_t pin:6; 	/* 0 - 63 pins 000000-111111 */
	uint16_t type:1; 	/* 0 Analog 1 Digital */
	uint16_t mode:2; 	/* 0 INPUT, 1 OUTPUT, 2 INPUT_PULLUP */
	uint16_t persist:1;
	uint16_t pad0:6;

	uint16_t value; 	/* 1/0 in digital, other values in Analog */

} BrbRS485PacketSetPin;

/* 12 Bytes = 96 bits - 64 HDR + 32 BODY */
typedef struct _BrbRS485PacketPinData
{
	uint16_t pin:6; 	/* 0 - 63 pins 000000-111111 */
	uint16_t type:1; 	/* 0 Analog 1 Digital */
	uint16_t mode:2; 	/* 0 INPUT, 1 OUTPUT, 2 INPUT_PULLUP */
	uint16_t persist:1;
	uint16_t pad0:6;

	uint16_t value; 	/* 1/0 in digital, other values in Analog */

} BrbRS485PacketPinData;


typedef struct _BrbRS485PacketSetPinBmpDig
{
	BrbRS485PacketHdr hdr;

	struct
	{
		uint8_t mode:2; 	/* 0 INPUT, 1 OUTPUT, 2 INPUT_PULLUP 3 IGNORE */
		uint8_t value:1;  	/* */
		uint8_t persist:1;
	} map[64];

} BrbRS485PacketSetPinBmpDig;

typedef struct _BrbRS485PacketSetPinBmpAna
{
	BrbRS485PacketHdr hdr;

	struct 
	{ 
		uint16_t mode:2; 	/* 0 INPUT, 1 OUTPUT, 2 INPUT_PULLUP 3 IGNORE */
		uint16_t value:12;  /* 4095 */
		uint16_t persist:1; /* to persist */
		uint16_t pad0:1;
	} map[32];
} BrbRS485PacketSetPinBmpAna;

typedef struct _BrbRS485PacketSetScript
{
	BrbRS485PacketHdr hdr;
	
    uint8_t script_id;

	uint8_t persist:1;
	uint8_t pad:7;

	BrbMicroCode code;

} BrbRS485PacketSetScript;

typedef struct _BrbRS485Session
{
	BrbBase *brb_base;
	BrbLogBase *log_base;

#ifdef BRB_RS485_HARDWARE_SERIAL
	HardwareSerial *serial;
#else
	SoftwareSerial *serial;
#endif

	long baudrate;


	int pinRO; 		// rx pin
	int pinDI; 		// tx pin
	int pinREDE; 	// ctrl pin

	int timer_id;

	byte byte_cur;

	BrbRS485StatsData stats;
	BrbRS485SessionActionEvents cb[RS485_PKT_TYPE_LAST_ITEM];

	/* Data is persistent to EEPROM, when saved */
	struct
	{
		uint8_t address;
		uint8_t uuid[4];
	} data;

	/* Buff to read data */
	struct
	{
		uint8_t data[BRB_RS485_MAX_PKT_SZ * 2];
		uint8_t sz;
		uint8_t index;
	} buf;

	/* IO static buffers for RS 485 packets */
	struct
	{
		struct
		{
			uint8_t data[BRB_RS485_MAX_PKT_SZ];
			uint8_t sz;
		} in;
		
		struct
		{
			uint8_t data[BRB_RS485_MAX_PKT_SZ];
			uint8_t sz;
		} out;

	} pkt;

	struct
	{
		unsigned int has_begin : 1;
		unsigned int has_end : 1;
		unsigned int has_msg : 1;
		unsigned int accept_bcast_cmd:1; 
		unsigned int wait_first_nible : 1;
	} flags;

} BrbRS485Session;
 /**********************************************************************************************************************/
BrbRS485Session *BrbRS485Session_New(BrbBase *brb_base);

int BrbRS485Session_Init(BrbRS485Session *rs485_sess);
int BrbRS485Session_Loop(BrbRS485Session *rs485_sess);

int BrbRS485Session_DataLoad(BrbRS485Session *rs485_sess);
int BrbRS485Session_DataSave(BrbRS485Session *rs485_sess);

int BrbRS485Session_SetEventCB(BrbRS485Session *rs485_sess, int action_code, BrbRS485SessionActionCBH *cb_func, void *cb_data);

int BrbRS485Session_SendComplemented(BrbRS485Session *rs485_sess, const byte p);
int BrbRS485Session_SendPacket(BrbRS485Session *rs485_sess, const byte *data_ptr, size_t data_sz);

int BrbRS485Session_SendMsg(BrbRS485Session *rs485_sess, byte src, byte dst, char *buffer_ptr, unsigned int buffer_sz);
int BrbRS485Session_SendHandShake(BrbRS485Session *rs485_sess);

int BrbRS485Session_ReadByte(BrbRS485Session *rs485_sess, uint8_t byte_read);
int BrbRS485Session_ReadMessage(BrbRS485Session *rs485_sess);
int BrbRS485Session_Reset(BrbRS485Session *rs485_sess);
 /**********************************************************************************************************************/
#endif /* BRB_RS485_SESSION_H_ */

