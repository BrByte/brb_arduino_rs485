/*
 * BrbDLinkedList.h
 *
 *  Created on: 2019-02-11
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

#ifndef BRB_DLINKED_LIST_H_
#define BRB_DLINKED_LIST_H_

#include <BrbBase.h>
#include <assert.h>

#if defined(ESP32) || defined(ESP8266)
#ifndef MUTEX_INIT
#define MUTEX_INIT(mutex, mutex_name)			pthread_mutex_init( (pthread_mutex_t*)(&mutex), 0);
#define MUTEX_LOCK(mutex, mutex_name)			pthread_mutex_lock( (pthread_mutex_t*)(&mutex));
#define MUTEX_TRYLOCK(mutex, mutex_name, state)	state = pthread_mutex_trylock( (pthread_mutex_t*)(&mutex));
#define MUTEX_UNLOCK(mutex, mutex_name)			pthread_mutex_unlock( (pthread_mutex_t*)(&mutex));
#define MUTEX_DESTROY(mutex, mutex_name)		pthread_mutex_destroy((pthread_mutex_t*)(&mutex));
#endif
#endif


/**********************************************************************************************************************/
/* DEFINES */
/**********************************************************/
#define BRB_DLINKED_LIST_PTR_ISEMPTY(list) ((list)->head == NULL)
#define BRB_DLINKED_LIST_ISEMPTY(list) ((list).head == NULL)
#define BRB_DLINKED_LIST_HEAD(list) ((list).head->data)
#define BRB_DLINKED_LIST_ISORPHAN(list) ((list).next == NULL && (list).prev == NULL)
/**********************************************************************************************************************/
/* ENUMS */
/**********************************************************/
typedef enum
{
    BRB_DLINKEDLIST_SORT_DESCEND,
    BRB_DLINKEDLIST_SORT_ASCEND
} BrbDLinkedListSortCodes;
/**********************************************************************************************************************/
/* STRUCTS */
/**********************************************************/
typedef struct _BrbDLinkedListNode
{
    void *data;
    struct _BrbDLinkedListNode *prev;
    struct _BrbDLinkedListNode *next;
} BrbDLinkedListNode;

typedef struct _BrbDLinkedList
{
    BrbDLinkedListNode *head;
    BrbDLinkedListNode *tail;

    unsigned long size;
#if defined(ESP32) || defined(ESP8266)
    pthread_mutex_t mutex;
#endif
    struct
    {
        unsigned int thread_safe: 1;
    } flags;

} BrbDLinkedList;
/**********************************************************************************************************************/
/* PUBLIC FUNCTIONS */
/**********************************************************/

typedef int BrbDLinkedListCompareFunc(BrbDLinkedListNode *, BrbDLinkedListNode *);
typedef int BrbDLinkedListFilterNode(BrbDLinkedListNode *node, char *filter_key, char *filter_node);

void BrbDLinkedListInit(BrbDLinkedList *list, int thread_safe);
void BrbDLinkedListReset(BrbDLinkedList *list);
void BrbDLinkedListAddDebug(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data);
void BrbDLinkedListAdd(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data);
void BrbDLinkedListAddTailDebug(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data);
void BrbDLinkedListAddTail(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data);
void BrbDLinkedListDeleteDebug(BrbDLinkedList *list, BrbDLinkedListNode *node);
void BrbDLinkedListDelete(BrbDLinkedList *list, BrbDLinkedListNode *node);
void *BrbDLinkedListPopTail(BrbDLinkedList *list);
void *BrbDLinkedListPopHead(BrbDLinkedList *list);
void BrbDLinkedListDup(BrbDLinkedList *list, BrbDLinkedList *ret_list);
void BrbDLinkedListDupFilter(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListFilterNode *filter_func, char *filter_key, char *filter_value);
void BrbDLinkedListClean(BrbDLinkedList *list);
void BrbDLinkedListDestroyData(BrbDLinkedList *list);
void BrbDLinkedListSort(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);
void BrbDLinkedListSortFilter(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag, BrbDLinkedListFilterNode *filter_func, char *filter_key, char *filter_value);
void BrbDLinkedListBubbleSort(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);
void BrbDLinkedListMergeSort(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);

void BrbDLinkedListSortSimple(BrbDLinkedList *list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);
void BrbDLinkedListSortBubble(BrbDLinkedList *list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);
void BrbDLinkedListSortMerge(BrbDLinkedList *list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);
/**********************************************************************************************************************/
#endif /* BRB_DLINKED_LIST_H_ */