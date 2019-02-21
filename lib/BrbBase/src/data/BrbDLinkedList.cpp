/*
 * BrbDLinkedList.cpp
 *
 *  Created on: 2019-02-11
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
 *
 *
 * Copyright (c) 2019 BrByte Software (Oliveira Alves & Amorim LTDA)
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

#include "BrbDLinkedList.h"

static BrbDLinkedListNode *BrbDLinkedListMergeSortFunc(BrbDLinkedListNode *head, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag);

/**************************************************************************************************************************/
void BrbDLinkedListInit(BrbDLinkedList *list, int thread_safe)
{
	/* Clean up list data */
	memset(list, 0, sizeof(BrbDLinkedList));

	/* Set MT_FLAGS */
	list->flags.thread_safe = thread_safe;
#ifdef MUTEX_INIT
	list->mutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListReset(BrbDLinkedList *list)
{
	/* Sanity check */
	if (!list)
		return;

#ifdef MUTEX_INIT
	/* Running thread safe, destroy MUTEX */
	if (list->flags.thread_safe)
		MUTEX_DESTROY(list->mutex, "DLINKED_LIST");

	list->mutex = (pthread_mutex_t)PTHREAD_MUTEX_INITIALIZER;
#endif

	/* Reset LIST */
	list->head = NULL;
	list->tail = NULL;
	list->size = 0;

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListAddDebug(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data)
{
	BrbDLinkedListNode *aux_node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Walk the list to see if this node address is known */
	for (aux_node = list->head; aux_node; aux_node = aux_node->next)
		assert(aux_node != node);

	node->data = data;
	node->prev = NULL;
	node->next = list->head;

	if (list->head)
		list->head->prev = node;

	list->head = node;

	if (list->tail == NULL)
		list->tail = node;

	list->size++;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListAdd(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	node->data = data;
	node->prev = NULL;
	node->next = list->head;

	if (list->head)
		list->head->prev = node;

	list->head = node;

	if (list->tail == NULL)
		list->tail = node;

	list->size++;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListAddTailDebug(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data)
{
	BrbDLinkedListNode *aux_node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Walk the list to see if this node address is known */
	for (aux_node = list->head; aux_node; aux_node = aux_node->next)
		assert(aux_node != node);

	node->data = data;
	node->next = NULL;
	node->prev = list->tail;

	if (list->tail)
		list->tail->next = node;

	list->tail = node;

	if (list->head == NULL)
		list->head = node;

	list->size++;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListAddTail(BrbDLinkedList *list, BrbDLinkedListNode *node, void *data)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	node->data = data;
	node->next = NULL;
	node->prev = list->tail;

	if (list->tail)
		list->tail->next = node;

	list->tail = node;

	if (list->head == NULL)
		list->head = node;

	list->size++;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListDeleteDebug(BrbDLinkedList *list, BrbDLinkedListNode *node)
{
	BrbDLinkedListNode *aux_node;
	int node_found;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Walk the list to see if this node address is known */
	for (node_found = 0, aux_node = list->head; aux_node; aux_node = aux_node->next)
	{
		if (aux_node == node)
		{
			node_found = 1;
			break;
		}
		continue;
	}

	assert(node_found);

	/* This node is orphan */
	if ((list->head != node) && (NULL == node->next) && (NULL == node->prev))
	{
#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif
		return;
	}

	if (node->next)
		node->next->prev = node->prev;

	if (node->prev)
		node->prev->next = node->next;

	if (node == list->head)
		list->head = node->next;

	if (node == list->tail)
		list->tail = node->prev;

	list->size--;

	node->next = node->prev = NULL;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListDelete(BrbDLinkedList *list, BrbDLinkedListNode *node)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* This node is orphan */
	if ((list->head != node) && (NULL == node->next) && (NULL == node->prev))
	{
#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return;
	}

	if (node->next)
		node->next->prev = node->prev;

	if (node->prev)
		node->prev->next = node->next;

	if (node == list->head)
		list->head = node->next;

	if (node == list->tail)
		list->tail = node->prev;

	list->size--;

	/* Orphanize NODE */
	node->next = NULL;
	node->prev = NULL;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void *BrbDLinkedListPopTail(BrbDLinkedList *list)
{
	BrbDLinkedListNode *node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	if (list->tail == NULL)
	{
#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return NULL;
	}

	node = list->tail;

	/* Head and tail are the same, only item on list */
	if (node == list->head)
	{
		list->head = NULL;
		list->tail = NULL;
	}

	/* Unlink from previous node */
	if (node->prev)
		node->prev->next = NULL;

	/* Set new tail to previous node */
	list->tail = node->prev;

	/* Unlink local node */
	node->next = NULL;
	node->prev = NULL;

	list->size--;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return node->data;
}
/**************************************************************************************************************************/
void *BrbDLinkedListPopHead(BrbDLinkedList *list)
{
	BrbDLinkedListNode *node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	if (list->head == NULL)
	{
#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return NULL;
	}

	node = list->head;

	if (node->next)
		node->next->prev = node->prev;

	if (node->prev)
		node->prev->next = node->next;

	if (node == list->head)
		list->head = node->next;

	if (node == list->tail)
		list->tail = node->prev;

	node->next = node->prev = NULL;

	list->size--;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return node->data;
}
/**************************************************************************************************************************/
void BrbDLinkedListDestroyData(BrbDLinkedList *list)
{
	BrbDLinkedListNode *node;
	BrbDLinkedListNode *prev_node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	node = list->head;

	while (node)
	{
		/* Walking */
		prev_node = node;
		node = node->next;

		BrbDLinkedListDelete(list, prev_node);

		/* Destroy the data and node */
		free(prev_node->data);
		free(prev_node);

		continue;
	}

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListDup(BrbDLinkedList *list, BrbDLinkedList *ret_list)
{
	BrbDLinkedListNode *node;
	BrbDLinkedListNode *new_node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	for (node = list->head; node; node = node->next)
	{
		new_node = (BrbDLinkedListNode *)calloc(1, sizeof(BrbDLinkedListNode));
		new_node->data = node->data;
		BrbDLinkedListAdd(ret_list, new_node, new_node->data);

		continue;
	}

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListDupFilter(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListFilterNode *filter_func, char *filter_key, char *filter_value)
{
	BrbDLinkedListNode *node;
	BrbDLinkedListNode *new_node;
	int filter_node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	for (node = list->head; node; node = node->next)
	{
		if (!filter_func)
			continue;

		filter_node = filter_func(node, filter_key, filter_value);

		/* filter node */
		if (filter_node)
			continue;

		/* copy node */
		new_node = (BrbDLinkedListNode *)calloc(1, sizeof(BrbDLinkedListNode));
		new_node->data = node->data;
		BrbDLinkedListAdd(ret_list, new_node, new_node->data);
	}

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListClean(BrbDLinkedList *list)
{
	BrbDLinkedListNode *node;
	BrbDLinkedListNode *prev_node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	for (node = list->head; node; node = node->next)
	{
	loop_without_move:

		/* Sanity check */
		if (!node)
			break;

		/* Save reference pointers */
		prev_node = node;
		node = node->next;

		/* Remove from pending request list */
		BrbDLinkedListDelete(list, prev_node);

		/* Destroy the node */
		free(prev_node);

		goto loop_without_move;
	}

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListSort(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Return empty list */
	if (!list->head)
	{
#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return;
	}

	/* Duplicate into return list and calculate limit */
	BrbDLinkedListDup(list, ret_list);

	/* Sort Duplicated List */
	if (ret_list->size > 512)
		BrbDLinkedListSortMerge(ret_list, cmp_func, cmp_flag);
	else
		BrbDLinkedListSortSimple(ret_list, cmp_func, cmp_flag);

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListSortFilter(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag,
							  BrbDLinkedListFilterNode *filter_func, char *filter_key, char *filter_value)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Return empty list */
	if (!list->head)
	{
		/* Initialize empty list */
		BrbDLinkedListInit(ret_list, list->flags.thread_safe);

#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return;
	}

	/* Duplicate into return list and calculate limit */
	BrbDLinkedListDupFilter(list, ret_list, filter_func, filter_key, filter_value);

	/* Sort Duplicated List */
	if (ret_list->size > 512)
		BrbDLinkedListSortMerge(ret_list, cmp_func, cmp_flag);
	else
		BrbDLinkedListSortSimple(ret_list, cmp_func, cmp_flag);

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListBubbleSort(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Return empty list */
	if (!list->head)
	{
		/* Initialize empty list */
		BrbDLinkedListInit(ret_list, list->flags.thread_safe);

#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return;
	}

	/* Duplicate LIST */
	BrbDLinkedListDup(list, ret_list);

	/* Sort Duplicated List */
	BrbDLinkedListSortBubble(ret_list, cmp_func, cmp_flag);

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListMergeSort(BrbDLinkedList *list, BrbDLinkedList *ret_list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Return empty list */
	if (!list->head)
	{
		/* Initialize empty list */
		BrbDLinkedListInit(ret_list, list->flags.thread_safe);

#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return;
	}

	/* duplicate the list to return the list ordered without alter original list */
	BrbDLinkedListDup(list, ret_list);

	/* Sort Duplicated List */
	BrbDLinkedListSortMerge(ret_list, cmp_func, cmp_flag);

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

	return;
}
/**************************************************************************************************************************/
/* SORT FUNCTIONS */
/**************************************************************************************************************************/
void BrbDLinkedListSortSimple(BrbDLinkedList *list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
	BrbDLinkedListSortMerge(list, cmp_func, cmp_flag);

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListSortBubble(BrbDLinkedList *list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
	BrbDLinkedListSortMerge(list, cmp_func, cmp_flag);

	return;
}
/**************************************************************************************************************************/
void BrbDLinkedListSortMerge(BrbDLinkedList *list, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
	BrbDLinkedListNode *node;

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, LOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_LOCK(list->mutex, "DLINKED_LIST");
#endif

	/* Return empty list */
	if (!list->head)
	{
#ifdef MUTEX_INIT
		/* Running THREAD_SAFE, UNLOCK MUTEX */
		if (list->flags.thread_safe)
			MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif

		return;
	}

	/* check if the return list has more than one item */
	if (list->size > 1)
	{
		/* call the function to order */
		node = list->head;
		list->head = BrbDLinkedListMergeSortFunc(node, cmp_func, cmp_flag);

		node = list->head;

		while (node->next)
			node = node->next;

		list->tail = node;
	}

#ifdef MUTEX_INIT
	/* Running THREAD_SAFE, UNLOCK MUTEX */
	if (list->flags.thread_safe)
		MUTEX_UNLOCK(list->mutex, "DLINKED_LIST");
#endif
	return;
}
/**************************************************************************************************************************/
/**/
/**/
/**************************************************************************************************************************/
static BrbDLinkedListNode *BrbDLinkedListMergeSortFunc(BrbDLinkedListNode *head, BrbDLinkedListCompareFunc *cmp_func, BrbDLinkedListSortCodes cmp_flag)
{
	/* Private function with no LOCKING */

	/* Trivial case: length 0 or 1 */
	if (!head || !head->next)
		return head;

	BrbDLinkedListNode *right = head;
	BrbDLinkedListNode *temp = head;
	BrbDLinkedListNode *last = head;
	BrbDLinkedListNode *result = 0;
	BrbDLinkedListNode *next = 0;
	BrbDLinkedListNode *tail = 0;

	/* Find halfway through the list (by running two pointers, one at twice the speed of the other) */
	while (temp && temp->next)
	{
		last = right;
		right = right->next;
		temp = temp->next->next;

		continue;
	}

	/* Break the list in two - node->prev are broken here, but we fix it later */
	last->next = 0;

	/* Recurse on the two smaller lists */
	head = BrbDLinkedListMergeSortFunc(head, cmp_func, cmp_flag);
	right = BrbDLinkedListMergeSortFunc(right, cmp_func, cmp_flag);

	/* Merge lists */
	while (head || right)
	{
		/* Take from empty right list */
		if (!right)
		{
			next = head;
			head = head->next;
		}
		/* Take from empty head list */
		else if (!head)
		{
			next = right;
			right = right->next;
		}
		/* compare elements to ordering */
		else if (cmp_flag ? !cmp_func(head, right) : cmp_func(head, right))
		{
			next = head;
			head = head->next;
		}
		else
		{
			next = right;
			right = right->next;
		}

		if (!result)
			result = next;
		else
			tail->next = next;

		/* fixed previous pointer */
		next->prev = tail;
		tail = next;

		continue;
	}

	return result;
}
/**************************************************************************************************************************/
