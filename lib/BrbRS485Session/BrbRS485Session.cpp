/*
 * BrbRS485Session.cpp
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

#include "BrbRS485Session.h"

static BrbGenericCBH BrbRS485Session_TimerHandShakeCB;
static byte BrbRS485Session_CalcCRC8(const byte *addr, size_t len);

static uint8_t BrbRS485Session_PktParser(BrbRS485Session *rs485_sess, BrbRS485PacketHdr *brb_pkt);
/**********************************************************************************************************************/
BrbRS485Session *BrbRS485Session_New(BrbBase *brb_base)
{
	BrbRS485Session *rs485_sess;

	rs485_sess = (BrbRS485Session *)calloc(1, sizeof(BrbRS485Session));

	rs485_sess->brb_base = brb_base;
	rs485_sess->log_base = brb_base->log_base;

	return rs485_sess;
}
/**********************************************************************************************************************/
int BrbRS485Session_Init(BrbRS485Session *rs485_sess)
{
	/* sanitize */
	if (!rs485_sess)
		return -1;

	/* Read data */
	BrbRS485Session_DataLoad(rs485_sess);

	/* receive pin, transmit pin */
	if (rs485_sess->serial == NULL)
	{
		rs485_sess->serial = &Serial3;
	}

	if (rs485_sess->device_type > 0)
	{
		rs485_sess->data.uuid[0] = rs485_sess->device_type;
	}

#ifdef BRB_RS485_HARDWARE_SERIAL
	/* No pin modifications */
	/* Using Hardware Serial */
#else
	/* define pin modes for tx, rx: */
	pinMode(rs485_sess->pinDI, OUTPUT);
	pinMode(rs485_sess->pinRO, INPUT);
#endif

	/* Initialize SoftwareSerial to RS485 */
	rs485_sess->serial->begin(BRB_RS485_BAUDRATE);

	/* set up various pins */
	if (rs485_sess->pinREDE > 0)
	{
		pinMode(rs485_sess->pinREDE, OUTPUT);
	}

	/* Add timer to send HandShake */
	rs485_sess->timer_id = BrbTimerAdd(rs485_sess->brb_base, BRB_RS485_HANDSHAKE_TIME, BRB_RS485_HANDSHAKE_REPEAT, BrbRS485Session_TimerHandShakeCB, rs485_sess);

	/* Send hello to notify my new ID */
	BrbRS485Session_SendHandShake(rs485_sess);

	return 0;
}
/**********************************************************************************************************************/
int BrbRS485Session_Loop(BrbRS485Session *rs485_sess)
{
	BrbRS485Session_ReadMessage(rs485_sess);

	return 0;
}
/**********************************************************************************************************************/
int BrbRS485Session_DataLoad(BrbRS485Session *rs485_sess)
{
	/* Clear all data before loading */
	memset((uint8_t *)&rs485_sess->data, 0, sizeof(rs485_sess->data));

	/* Read EEPROM */
	BrbBase_EEPROMRead(rs485_sess->brb_base, (uint8_t *)&rs485_sess->data, sizeof(rs485_sess->data), BRB_RS485_EEPROM_OFFSET);

	int changed = 0;

	if (!rs485_sess)
		return -1;

	LOG_DEBUG(rs485_sess->log_base, "RS485 - ADDR %d x%02x\r\n", rs485_sess->data.address, rs485_sess->data.address);

	if (rs485_sess->data.address == 0 || rs485_sess->data.address == 255)
	{
		rs485_sess->data.address = random(1, 254);
		changed = 1;
	}

	for (unsigned int i = 0; i < sizeof(rs485_sess->data.uuid); i++)
	{
		if (rs485_sess->data.uuid[i] == 0 || rs485_sess->data.uuid[i] == 255)
		{
			rs485_sess->data.uuid[i] = random(1, 254);
			changed = 1;
		}
	}

	if (changed)
	{
		BrbRS485Session_DataSave(rs485_sess);
	}

	return 0;
}
/**********************************************************************************************************************/
int BrbRS485Session_DataSave(BrbRS485Session *rs485_sess)
{
	/* Read EEPROM */
	BrbBase_EEPROMWrite(rs485_sess->brb_base, (uint8_t *)&rs485_sess->data, sizeof(rs485_sess->data), BRB_RS485_EEPROM_OFFSET);

	return 0;
}
/**********************************************************************************************************************/
static int BrbRS485Session_TimerHandShakeCB(void *base_ptr, void *cb_data_ptr)
{
	// BrbTimer *timer = (BrbTimer *)base_ptr;
	BrbRS485Session *rs485_sess = (BrbRS485Session *)cb_data_ptr;

	// LOG_INFO(rs485_sess->log_base, "TIMER: %d cb: 0x%02x\n", timer->timer_id, rs485_sess);

	// if ((rs485_sess->brb_base->ms.cur - rs485_sess->stats.ms.last_write) < (BRB_RS485_HANDSHAKE_TIME - 1000))
	// {
	// 	return 0;
	// }

	BrbRS485Session_SendHandShake(rs485_sess);

	return 0;
}
/**********************************************************************************************************************/
int BrbRS485Session_SetEventCB(BrbRS485Session *rs485_sess, int action_code, BrbRS485SessionActionCBH *cb_func, void *cb_data)
{
	if (action_code < 0 || action_code >= RS485_PKT_TYPE_LAST_ITEM)
		return -1;

	rs485_sess->cb[action_code].cb_func = cb_func;
	rs485_sess->cb[action_code].cb_data = cb_data;

	return 0;
}
/**********************************************************************************************************************/
/* send a byte complemented 0F, 1E, 2D, 3C, 4B, 5A, 69, 78, 87, 96, A5, B4, C3, D2, E1, F0 */
/**********************************************************************************************************************/
int BrbRS485Session_SendComplemented(BrbRS485Session *rs485_sess, const byte p)
{
	byte c;

	/* first nibble */
	c = p >> 4;
	rs485_sess->serial->write((c << 4) | (c ^ 0x0F));
	// delayMicroseconds(50);

	/* second nibble */
	c = p & 0x0F;
	rs485_sess->serial->write((c << 4) | (c ^ 0x0F));
	// delayMicroseconds(50);

	return 2;
}
/**********************************************************************************************************************/
int BrbRS485Session_SendPacket(BrbRS485Session *rs485_sess, const byte *data_ptr, size_t data_sz)
{
	int bytes_write = 0;

	/* sanitize */
	if (!rs485_sess || !data_ptr || data_sz < 1)
		return -1;

	/* Calculate CRC */
	byte crc8 = BrbRS485Session_CalcCRC8(data_ptr, data_sz);

	LOG_DEBUG(rs485_sess->log_base, "---> Send PKT: [0x%02x -> 0x%02x] - T [%d] - 8 [0x%02x] - SZ [%u]\r\n",
			  ((BrbRS485PacketHdr *)data_ptr)->src, ((BrbRS485PacketHdr *)data_ptr)->dst, ((BrbRS485PacketHdr *)data_ptr)->type, crc8, data_sz);

	if (rs485_sess->flags.has_begin && ((rs485_sess->buf.index + 1) < rs485_sess->buf.sz))
	{
		LOG_WARN(rs485_sess->log_base, "### Send PKT: b:%d - buf idx:%u - sz:%u\r\n",
				 rs485_sess->flags.has_begin, rs485_sess->buf.index, rs485_sess->buf.sz);
	}

	/* enable sending */
	digitalWrite(rs485_sess->pinREDE, RS485_TOGGLE_TRASMIT);

#ifdef BRB_RS485_HARDWARE_SERIAL
	delayMicroseconds(5);
#else

#endif

	/* Initializer Control Data */
	bytes_write += rs485_sess->serial->write(BRB_RS485_BEGIN);

	delay(1000);

	/* Write data */
	for (byte i = 0; i < data_sz; i++)
	{
		// if (rs485_sess->serial->availableForWrite() < 1 || (((i + 1) % 16) == 0))
		if (((i + 1) % 32) == 0)
		{
			// LOG_DEBUG(rs485_sess->log_base, "### Send PKT: [%d][%d] - avail [%d]\r\n", i, data_sz, rs485_sess->serial->availableForWrite());

			/* flush transmit buffer */
			rs485_sess->serial->flush();
		}

		bytes_write += BrbRS485Session_SendComplemented(rs485_sess, data_ptr[i]);
	}

	/* Finish  Control Data */
	bytes_write += rs485_sess->serial->write(BRB_RS485_END);

	/* Send CRC8 signature */
	bytes_write += BrbRS485Session_SendComplemented(rs485_sess, crc8);

#ifdef BRB_RS485_HARDWARE_SERIAL
	/* flush transmit buffer */
	rs485_sess->serial->flush();
#else
	/* flush transmit buffer */
	rs485_sess->serial->flush();
#endif

	/* disable sending */
	digitalWrite(rs485_sess->pinREDE, RS485_TOGGLE_RECEIVE);

	/* Count packets sended */
	rs485_sess->stats.byte.tx = rs485_sess->stats.byte.tx + bytes_write;
	rs485_sess->stats.pkt.tx++;
	rs485_sess->stats.ms.last_write = rs485_sess->brb_base->ms.cur;

	return bytes_write;
}
/**********************************************************************************************************************/
int BrbRS485Session_SendMsg(BrbRS485Session *rs485_sess, byte src, byte dst, char *buffer_ptr, unsigned int buffer_sz)
{
	/* sanitize */
	if (!rs485_sess || !buffer_ptr)
		return -1;

	LOG_INFO(rs485_sess->log_base, "---> Send MSG: P[%d] - [%d] --> [%d] - [%u][%s]\r\n", src, dst, buffer_sz, buffer_ptr);

	BrbRS485PacketData *rs485_pkt = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;

	/* Adjust to not overflow */
	if (buffer_sz > (sizeof(rs485_sess->pkt.out.data) - sizeof(BrbRS485PacketData)))
		buffer_sz = (sizeof(rs485_sess->pkt.out.data) - sizeof(BrbRS485PacketData));

	rs485_pkt->hdr.src = src;
	rs485_pkt->hdr.dst = dst;
	rs485_pkt->hdr.type = 0;
	rs485_pkt->hdr.len = buffer_sz + sizeof(BrbRS485PacketData);

	memcpy(&rs485_pkt->val, buffer_ptr, buffer_sz);

	/* Send CRC */
	return BrbRS485Session_SendPacket(rs485_sess, (byte *)rs485_pkt, rs485_pkt->hdr.len);
}
/**********************************************************************************************************************/
int BrbRS485Session_SendACK(BrbRS485Session *rs485_sess, BrbRS485PacketHdr *rs485_packet)
{
	BrbRS485PacketHdr *rs485_packet_ack = (BrbRS485PacketHdr *)&rs485_sess->pkt.out.data;

	rs485_packet_ack->src = rs485_sess->data.address;
	rs485_packet_ack->dst = rs485_packet->src;
	rs485_packet_ack->magic = BRB_RS485_MAGIC;
	rs485_packet_ack->type = RS485_PKT_TYPE_ACK;
	rs485_packet_ack->len = sizeof(BrbRS485PacketHdr);
	rs485_packet_ack->id = rs485_packet->id;

	/* Send CRC */
	return BrbRS485Session_SendPacket(rs485_sess, (byte *)rs485_packet_ack, sizeof(BrbRS485PacketHdr));
}
/**********************************************************************************************************************/
int BrbRS485Session_SendHandShake(BrbRS485Session *rs485_sess)
{
	BrbRS485PacketHandShake *rs485_pkt_hs = (BrbRS485PacketHandShake *)&rs485_sess->pkt.out.data;

	rs485_pkt_hs->hdr.src = rs485_sess->data.address;
	rs485_pkt_hs->hdr.dst = 0xFF;
	rs485_pkt_hs->hdr.type = RS485_PKT_TYPE_HANDSHAKE;
	rs485_pkt_hs->hdr.len = sizeof(BrbRS485PacketHandShake);
	rs485_pkt_hs->hdr.id = 0;

	memcpy(&rs485_pkt_hs->uuid, &rs485_sess->data.uuid, sizeof(rs485_pkt_hs->uuid));

	LOG_INFO(rs485_sess->log_base, "Send handshake - b:%u sz:%u buf i:%u s:%u pkt:%u/%u\r\n",
			 rs485_sess->flags.has_begin, rs485_sess->pkt.in.sz, rs485_sess->buf.index, rs485_sess->buf.sz,
			 rs485_sess->stats.pkt.rx, rs485_sess->stats.pkt.tx);

	/* Reading packet, do not send handshake */
	if (rs485_sess->pkt.in.sz > 0 || rs485_sess->flags.has_begin)
		return 0;

	/* Send CRC */
	return BrbRS485Session_SendPacket(rs485_sess, (byte *)rs485_pkt_hs, rs485_pkt_hs->hdr.len);
}
/**********************************************************************************************************************/
int BrbRS485Session_SendPacketData(BrbRS485Session *rs485_sess, uint8_t dst, BrbRS485PacketVal *val)
{
	BrbRS485PacketData *rs485_pkt = (BrbRS485PacketData *)&rs485_sess->pkt.out.data;

	/* Reading packet, do not send data */
	if (rs485_sess->pkt.in.sz > 0 || rs485_sess->flags.has_begin)
		return -1;

	rs485_pkt->hdr.src = rs485_sess->data.address;
	rs485_pkt->hdr.dst = dst;
	rs485_pkt->hdr.type = RS485_PKT_TYPE_DATA;
	rs485_pkt->hdr.len = sizeof(BrbRS485PacketData);
	rs485_pkt->hdr.id = 0;

	memcpy(&rs485_pkt->val, val, sizeof(rs485_pkt->val));

	LOG_INFO(rs485_sess->log_base, "Send data - b:%u sz:%u buf i:%u s:%u pkt:%u/%u\r\n",
			 rs485_sess->flags.has_begin, rs485_sess->pkt.in.sz, rs485_sess->buf.index, rs485_sess->buf.sz,
			 rs485_sess->stats.pkt.rx, rs485_sess->stats.pkt.tx);

	/* Send CRC */
	return BrbRS485Session_SendPacket(rs485_sess, (byte *)rs485_pkt, rs485_pkt->hdr.len);
}
/**********************************************************************************************************************/
int BrbRS485Session_ReadByte(BrbRS485Session *rs485_sess, uint8_t byte_read)
{
	int op_status;

	if (!byte_read)
	{
		// LOG_WARN(rs485_sess->log_base, "Got zero - %d-[%d / %d]\n", idx, rs485_sess->flags.has_begin, rs485_sess->flags.has_end);

		return 0;
	}

	LOG_INFO(rs485_sess->log_base, "---> GOT CHAR 0x%02x - sz [%d]\n", byte_read, rs485_sess->pkt.in.sz);

	rs485_sess->stats.byte.rx++;

	switch (byte_read)
	{
	case BRB_RS485_BEGIN: // start of text

		/* Set flags */
		rs485_sess->flags.has_begin = 1;
		rs485_sess->flags.has_end = 0;
		rs485_sess->flags.wait_first_nible = 1;
		rs485_sess->pkt.in.sz = 0;

		/* Count packets received */
		rs485_sess->stats.pkt.rx++;
		break;

	case BRB_RS485_END: // end of text
		/* Set flags */
		rs485_sess->flags.has_end = 1;
		break;

	default:
		/* wait until packet officially starts */
		if (!rs485_sess->flags.has_begin)
			break;

		/* check byte is in valid form (4 bits followed by 4 bits complemented) */
		if ((byte_read >> 4) != ((byte_read & 0x0F) ^ 0x0F))
		{
			LOG_WARN(rs485_sess->log_base, "---> BAD CHAR 0x%02x\r\n", byte_read);
			BrbLogBase_Info(rs485_sess->log_base);
			BrbLogBase_HexDump(rs485_sess->log_base, (char *)&rs485_sess->pkt.in.data, rs485_sess->pkt.in.sz);

			/* Reset info to process new messages */
			BrbRS485Session_Reset(rs485_sess);

			/* bad character, count it */
			rs485_sess->stats.err.bad_char++;
			break;
		}

		/* convert back */
		byte_read >>= 4;

		/* high-order nibble? */
		if (rs485_sess->flags.wait_first_nible)
		{
			/* end of first nibble, just copy it */
			rs485_sess->byte_cur = byte_read;
			rs485_sess->flags.wait_first_nible = 0;
			break;
		}

		/* low-order nibble */
		rs485_sess->byte_cur <<= 4;
		rs485_sess->byte_cur |= byte_read;
		rs485_sess->flags.wait_first_nible = 1;

		/* if we have the ETX this must be the CRC */
		if (rs485_sess->flags.has_end)
		{
			uint8_t crc8;

			crc8 = BrbRS485Session_CalcCRC8((uint8_t *)&rs485_sess->pkt.in.data, rs485_sess->pkt.in.sz);

			/* Bad CRC */
			if (crc8 != rs485_sess->byte_cur)
			{
				LOG_WARN(rs485_sess->log_base, "---> BAD CRC 0x%02x 0x%02x\r\n", crc8, rs485_sess->byte_cur);
				BrbLogBase_Info(rs485_sess->log_base);
				BrbLogBase_HexDump(rs485_sess->log_base, (char *)&rs485_sess->pkt.in.data, rs485_sess->pkt.in.sz);

				/* Reset info to process new messages */
				BrbRS485Session_Reset(rs485_sess);

				/* bad crc, count it */
				rs485_sess->stats.err.crc++;

				break;
			}

			rs485_sess->flags.has_msg = 1;

			BrbRS485PacketHdr *brb_pkt = (BrbRS485PacketHdr *)&rs485_sess->pkt.in.data;

			/* Dispatch */
			op_status = BrbRS485Session_PktParser(rs485_sess, brb_pkt);

			/* bad packet, count it */
			if (op_status != 0)
			{
				LOG_WARN(rs485_sess->log_base, "### FAIL 0x%02x-->0x%02x - [%d / %d]\r\n",
						 brb_pkt->src, brb_pkt->dst, brb_pkt->type, op_status);

				rs485_sess->stats.err.pkt++;
			}

			/* All things MUST be parsed before, reset to read more messages */
			BrbRS485Session_Reset(rs485_sess);

			/* move on, we have finished here */
			return 0;
		}

		/* keep adding if not full */
		if (rs485_sess->pkt.in.sz < sizeof(rs485_sess->pkt.in.data))
		{
			rs485_sess->pkt.in.data[rs485_sess->pkt.in.sz++] = rs485_sess->byte_cur;
			rs485_sess->pkt.in.data[rs485_sess->pkt.in.sz] = '\0';
		}
		else
		{
			LOG_WARN(rs485_sess->log_base, "### Overflow %d/%d\r\n", rs485_sess->pkt.in.sz, sizeof(rs485_sess->pkt.in.data));

			/* We have finished, reset info */
			BrbRS485Session_Reset(rs485_sess);

			/* overflow, count it */
			rs485_sess->stats.err.overflow++;
		}

		break;

	} // end of switch

	return 1;
}
/**********************************************************************************************************************/
int BrbRS485Session_ReadMessage(BrbRS485Session *rs485_sess)
{
	unsigned int read_avail = 0;
	unsigned int read_counter = 0;

	// flush transmit buffer
	// rs485_sess->serial->flush();

	/* no serial can't go ahead */
	if (!rs485_sess || !rs485_sess->serial)
		return -1;

read_again:

	/* disable sending */
	digitalWrite(rs485_sess->pinREDE, RS485_TOGGLE_RECEIVE);

	// byte read_byte;
	// while (rs485_sess->serial->available())
	// {
	// 	read_byte 	= rs485_sess->serial->readBytes((char *)&rs485_sess->buf.data, read_avail);
	// 	BrbRS485Session_ReadByte(rs485_sess, read_byte);
	// }

	read_avail = rs485_sess->serial->available();

	read_counter++;

	/* Nothing to read */
	if (read_avail <= 0)
		return 0;

	/* Adjust buffer size */
	if (read_avail > sizeof(rs485_sess->buf.data))
		read_avail = sizeof(rs485_sess->buf.data);

	/* Read data from UART */
	rs485_sess->buf.sz = rs485_sess->serial->readBytes((char *)&rs485_sess->buf.data, read_avail);

	/* Read size mismatch */
	if (rs485_sess->buf.sz != read_avail)
	{
		LOG_WARN(rs485_sess->log_base, "Read size mismatch [%d / %d]\r\n", rs485_sess->buf.sz, read_avail);
	}

	// if (read_avail > 60)
	// {
	LOG_INFO(rs485_sess->log_base, "Read [%d / %d] - [%u]\r\n", rs485_sess->buf.sz, read_avail, rs485_sess->pkt.in.sz);
	// }

	/* Update counters */
	rs485_sess->stats.byte.rx += rs485_sess->buf.sz;

	// process_read:

	/* Process byte array */
	for (rs485_sess->buf.index = 0; rs485_sess->buf.index < rs485_sess->buf.sz; rs485_sess->buf.index++)
	{
		BrbRS485Session_ReadByte(rs485_sess, rs485_sess->buf.data[rs485_sess->buf.index]);
		continue;
	}

	rs485_sess->buf.index = 0;
	rs485_sess->buf.sz = 0;

	/* We got some packet starting, read until it ends */
	if (rs485_sess->flags.has_begin > 0 && read_counter < 3)
	{
		// LOG_WARN(rs485_sess->log_base, "Read MORE sz %d - [%d / %d] c: %d \r\n", rs485_sess->pkt.in.sz, rs485_sess->buf.sz, read_avail, read_counter);

#ifdef BRB_RS485_HARDWARE_SERIAL
		delayMicroseconds(175);
#endif

		goto read_again;
	}

	return 1;
}
/**********************************************************************************************************************/
int BrbRS485Session_Reset(BrbRS485Session *rs485_sess)
{
	rs485_sess->flags.has_begin = 0;
	rs485_sess->flags.has_end = 0;
	rs485_sess->flags.has_msg = 0;
	rs485_sess->pkt.in.sz = 0;

	return 0;
}
/**********************************************************************************************************************/
static uint8_t BrbRS485Session_PktParser(BrbRS485Session *rs485_sess, BrbRS485PacketHdr *brb_pkt)
{
	uint8_t pkt_sz = brb_pkt->len;
	uint8_t cb_ret;
	uint8_t do_ack = 0;

	LOG_DEBUG(rs485_sess->log_base, "---> GOT PACKET 0x%02x-->0x%02x - %d len %d/%d/%d val %d\n",
			  brb_pkt->src, brb_pkt->dst, brb_pkt->type, brb_pkt->len, rs485_sess->pkt.in.sz,
			  sizeof(BrbRS485PacketData), brb_pkt->id);

	/* Validate packet size */
	if (pkt_sz != rs485_sess->pkt.in.sz)
	{
		LOG_WARN(rs485_sess->log_base, "---> FAIL [%d/%d] - type [%d]\n", pkt_sz, rs485_sess->pkt.in.sz, brb_pkt->type);
		return 1;
	}

	/* Broadcast packet? */
	if (brb_pkt->dst == 0xFF)
	{
		rs485_sess->stats.pkt.bcast++;

		/* Broadcast packet, and we are only allowing HANDSHAKE on broadcast */
		if (!rs485_sess->flags.accept_bcast_cmd)
		{
			// LOG_WARN(rs485_sess->log_base, "Not bcast cmd type [0x%02x]\r\n", brb_pkt->type);
			rs485_sess->stats.pkt.err.cmd_no_bcast++;
			/* Its not a bad packet, just a packet we don't care about */
			return 0;
		}

		/* We are only allowing HANDSHAKE on broadcast */
		if (brb_pkt->type != RS485_PKT_TYPE_HANDSHAKE)
		{
			LOG_WARN(rs485_sess->log_base, "Not bcast handshake type [0x%02x]\r\n", brb_pkt->type);
			rs485_sess->stats.pkt.err.cmd_no_bcast++;
			/* Its not a bad packet, just a packet we don't care about */
			return 0;
		}
	}
	/* Is it not for my bus addr? */
	else if (rs485_sess->data.address != brb_pkt->dst)
	{
		// LOG_WARN(rs485_sess->log_base, "Not for me [0x%02x] dst [0x%02x]\r\n", rs485_sess->data.address, brb_pkt->dst);
		/* Its not a bad packet, just a packet we don't care about */
		return 0;
	}
	/* This packet is for me */
	else
		rs485_sess->stats.pkt.me++;

	/* Process header */
	switch (brb_pkt->type)
	{
	case RS485_PKT_TYPE_HANDSHAKE:
	{
		if (pkt_sz != sizeof(BrbRS485PacketHandShake))
		{
			LOG_WARN(rs485_sess->log_base, "invalid hs sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketHandShake));
			return 2;
		}

		do_ack = 1;

		break;
	}
	case RS485_PKT_TYPE_ACK:
	{
		if (pkt_sz != sizeof(BrbRS485PacketHdr))
		{
			LOG_WARN(rs485_sess->log_base, "invalid ack sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketHdr));
			return 2;
		}

		do_ack = 0;

		break;
	}
	case RS485_PKT_TYPE_CMD_GET_A:
	case RS485_PKT_TYPE_CMD_GET_D:
	case RS485_PKT_TYPE_DATA:
	{
		if (pkt_sz != sizeof(BrbRS485PacketData))
		{
			LOG_WARN(rs485_sess->log_base, "invalid get sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketData));
			return 2;
		}

		do_ack = 1;

		break;
	}
	case RS485_PKT_TYPE_CMD_SET_A:
	case RS485_PKT_TYPE_CMD_SET_D:
	case RS485_PKT_TYPE_CMD_SET_ID:
	{
		if (pkt_sz != sizeof(BrbRS485PacketSetPin))
		{
			LOG_WARN(rs485_sess->log_base, "invalid set sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketSetPin));
			return 2;
		}

		do_ack = 1;

		break;
	}
	case RS485_PKT_TYPE_CMD_SET_A_BMP:
	{
		if (pkt_sz != sizeof(BrbRS485PacketSetPinBmpAna))
		{
			LOG_WARN(rs485_sess->log_base, "invalid bmp sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketSetPinBmpAna));
			return 2;
		}

		do_ack = 1;

		break;
	}
	case RS485_PKT_TYPE_CMD_SET_D_BMP:
	{
		if (pkt_sz != sizeof(BrbRS485PacketSetPinBmpDig))
		{
			LOG_WARN(rs485_sess->log_base, "invalid bmp sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketSetPinBmpDig));
			return 2;
		}

		do_ack = 1;

		break;
	}
	case RS485_PKT_TYPE_CMD_SET_SCRIPT:
	{
		if (pkt_sz != sizeof(BrbRS485PacketSetScript))
		{
			LOG_WARN(rs485_sess->log_base, "invalid script sz [%d/%d]\r\n", pkt_sz, sizeof(BrbRS485PacketSetScript));
			return 2;
		}

		do_ack = 1;

		break;
	}

		// int aux_int;

		// /* Skip own character */
		// buffer_ptr++;
		// buffer_sz--;

		// /* What remain is the ID, overflow will replace anything */
		// aux_int = TO_NUM(buffer_ptr[0], buffer_ptr[1]);

		// /* Set the new Address */
		// BrbRS485Session_SetAddress(rs485_sess, aux_int);

		// /* Send hello to notify my new ID */
		// BrbRS485Session_SendHandShake(rs485_sess);

	default:
		LOG_WARN(rs485_sess->log_base, "UnHandled [%u] PKT_ID [%u]\r\n", brb_pkt->type, brb_pkt->id);
		rs485_sess->stats.pkt.err.cmd_id++;
		return 3;
	}

	if (!rs485_sess->cb[brb_pkt->type].cb_func)
	{
		rs485_sess->stats.pkt.err.cmd_no_cb++;
		return 0;
	}

	cb_ret = rs485_sess->cb[brb_pkt->type].cb_func(rs485_sess, brb_pkt->type, brb_pkt, pkt_sz, rs485_sess->cb[brb_pkt->type].cb_data);

	if (do_ack && (cb_ret == RS485_PKT_RETURN_ACK_SUCCESS || cb_ret == RS485_PKT_RETURN_ACK_FAIL))
	{
		BrbRS485Session_SendACK(rs485_sess, brb_pkt);
	}

	return 0;
}
/**********************************************************************************************************************/
static byte BrbRS485Session_CalcCRC8(const byte *addr, size_t len)
{
	byte crc = 0;
	while (len--)
	{
		byte inbyte = *addr++;
		for (byte i = 8; i; i--)
		{
			byte mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
				crc ^= 0x8C;
			inbyte >>= 1;
			continue;
		}
	}
	return crc;
}
/**********************************************************************************************************************/
