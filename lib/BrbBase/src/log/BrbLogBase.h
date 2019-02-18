/*
 * BrbLogBase.h
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

#ifndef BRB_LOG_BASE_H_
#define BRB_LOG_BASE_H_

/**********************************************************************************************************************/
#include "Arduino.h"
#include <HardwareSerial.h>

#define LOG_ENABLE 1
#define LOG_DISABLE_INFO 1
// #define LOG_DISABLE_NOTICE 1
// #define LOG_DISABLE_WARN 1
// #define LOG_DISABLE_DEBUG 1

#ifndef LOG_BAUDRATE
// #define LOG_BAUDRATE 9600
#define LOG_BAUDRATE 115200
#endif

#ifndef LOG_MAX_STRING_LEN
#define LOG_MAX_STRING_LEN 64
#endif

/**********************************************************************************************************************/
typedef struct _BrbLogBase
{
    HardwareSerial *serial;

    struct {
        unsigned int foo:1;
    } flags;

} BrbLogBase;
/**********************************************************************************************************************/
BrbLogBase *BrbLogBase_New(void);
int BrbLogBase_Init(BrbLogBase *log_base, HardwareSerial *serial);
int BrbLogBase_PrintFmt(BrbLogBase *log_base, const char *format, ...);
// int BrbLogBase_Printf(BrbLogBase *log_base, const char *format, ...);
int BrbLogBase_Info(BrbLogBase *log_base);
void BrbLogBase_HexDump(BrbLogBase *log_base, char *data, int size);
/**********************************************************************************************************************/
#define LOG_INIT(l,s) BrbLogBase_Init(l,s)

#ifndef LOG_ENABLE
#define LOG_DISABLE_ALL 1
#endif

#ifndef LOG_DISABLE_INFO
#define LOG_ENABLE_INFO 1
#endif

#ifndef LOG_DISABLE_NOTICE
#define LOG_NOTICE_ENABLE 1
#endif

#ifndef LOG_DISABLE_WARN
#define LOG_WARN_ENABLE 1
#endif

#ifndef LOG_DISABLE_DEBUG
#define LOG_DEBUG_ENABLE 1
#endif

#ifdef LOG_DISABLE_ALL
#define LOG_PRINTF(log_base, format, ...)
#else
#define LOG_PRINTF(log_base, format, ...) BrbLogBase_PrintFmt(log_base, PSTR(format), ##__VA_ARGS__)
#endif

#ifdef LOG_INFO_ENABLE
#define LOG_INFO(log_base, format, ...) LOG_PRINTF(log_base, format, ##__VA_ARGS__)
#else
#define LOG_INFO(log_base, format, ...)
#endif

#ifdef LOG_NOTICE_ENABLE
#define LOG_NOTICE(log_base, format, ...) LOG_PRINTF(log_base, format, ##__VA_ARGS__)
#else
#define LOG_NOTICE(log_base, format, ...)
#endif

#ifdef LOG_WARN_ENABLE
#define LOG_WARN(log_base, format, ...) LOG_PRINTF(log_base, format, ##__VA_ARGS__)
#else
#define LOG_WARN(log_base, format, ...)
#endif

#ifdef LOG_DEBUG_ENABLE
#define LOG_DEBUG(log_base, format, ...) LOG_PRINTF(log_base, format, ##__VA_ARGS__)
#else
#define LOG_DEBUG(log_base, format, ...)
#endif

#define LOG_HEAP(log_base) BrbLogBase_Info(log_base)

/**********************************************************************************************************************/
#endif /* BRB_LOG_BASE_H_ */