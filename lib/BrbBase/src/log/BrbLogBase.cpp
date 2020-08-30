/*
 * BrbLogBase.cpp
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

#include "BrbBase.h"

/**********************************************************************************************************************/
BrbLogBase *BrbLogBase_New(void)
{
	BrbLogBase *log_base;

	log_base = (BrbLogBase *)calloc(1, sizeof(BrbLogBase));

	return log_base;
}
 /**********************************************************************************************************************/
int BrbLogBase_Init(BrbLogBase *log_base, HardwareSerial *serial)
{
	/* Sanitize */
	if (!log_base)
		return -1;

	log_base->serial 	= serial;

	/* Initialize Serial Console */
	log_base->serial->begin(LOG_BAUDRATE);

	return 0;
}
 /**********************************************************************************************************************/
int BrbLogBase_PrintFmt(BrbLogBase *log_base, const char *format, ...)
{
	/* Sanitize */
	if (!log_base || !log_base->serial)
		return -1;

	char buf[LOG_MAX_STRING_LEN];
	va_list argv;
	va_start(argv, format);

#ifdef __AVR__
	vsnprintf_P(buf, sizeof(buf), (PGM_P)format, argv); // progmem for AVR
#else
	vsnprintf(buf, sizeof(buf), format, argv);			 // for the rest of the world
#endif
	log_base->serial->print(buf);
	va_end(argv);

	return 0;
}
 /**********************************************************************************************************************/
void BrbLogBase_HexDump(BrbLogBase *log_base, char *data, int size)
{
	char buf[32];
	int i;

	log_base->serial->print("HEXDUMP ");
	snprintf(buf, sizeof(buf), " [%d]", size);
	log_base->serial->print(buf);
	log_base->serial->print(" bytes \r\n");
		
	for (i = 0; i < size; i++)
	{
		snprintf(buf, sizeof(buf), " [x%02x] ", data[i]);
		log_base->serial->print(buf);

		// log_base->serial->print("[");
		// log_base->serial->print(data[i], HEX);
		// log_base->serial->print("]");

		if (i % 16 == 1)
			log_base->serial->print("\r\n");
	
		continue;
	}

	log_base->serial->print(". . .\r\n");
	return;
}
 /**********************************************************************************************************************/
int BrbLogBase_Info(BrbLogBase *log_base)
{
	return BrbLogBase_PrintFmt(log_base, PSTR("HEAP: T %ld - M %d\r\n"), millis(), BrbBase_FreeRAM());
}
 /**********************************************************************************************************************/
