/*
 * BrbCrypto.cpp
 *
 *  Created on: 2019-01-04
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

#define RC4_SWAP(S,a,b) do { int t = S[a]; S[a] = S[b]; S[b] = t; } while(0)

/**********************************************************************************************************************/
int BrbCryptoRC4(char *key, uint8_t key_sz, char *data, uint8_t data_sz, char *dst_buf, uint8_t dst_buf_sz)
{
   unsigned char crypto_buf[(data_sz / 2) + 1];
   uint8_t i, j, k      = 0;
   uint8_t crypto_sz    = sizeof(crypto_buf); 

    for (i = 0; i < crypto_sz; i++)
    {
         crypto_buf[i] = i;
         continue;
    }

    for (i = 0; i < crypto_sz ; i++)
    {
         j = ((j + crypto_buf[i] + key[i % key_sz]) % crypto_sz);    
         RC4_SWAP(crypto_buf, crypto_buf[i], crypto_buf[j]);
         continue;
    }

     i = j = 0;
     
     for (k = 0; k < data_sz; k++)
     {
         i = ((i + 1) % crypto_sz);
         j = ((j + crypto_buf[i]) % crypto_sz);
         RC4_SWAP(crypto_buf, crypto_buf[i], crypto_buf[j]);
         
         /* Write encrypted data into buffer */
         dst_buf[k] = data[k] ^ crypto_buf[(crypto_buf[i] + crypto_buf[j]) % crypto_sz];
         continue;
     }

     return 1;
}
/**********************************************************************************************************************/