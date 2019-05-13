/*
 * BrbTimer.cpp
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

/**********************************************************************************************************************/
BrbTimer *BrbTimerGrabByID(BrbBase *brb_base, int timer_id)
{
    BrbTimer *timer;

    /* Sanity check */
    if (timer_id > MAX_TIMER)
        return NULL;

    timer = &brb_base->timer.arr[timer_id];

    return timer;
}
/**********************************************************************************************************************/
int BrbTimerAdd(BrbBase *brb_base, long delay_ms, int persist, BrbGenericCBH *cb_func, void *cb_data)
{
    BrbTimer *timer;
    int i;

    timer = NULL;

    LOG_DEBUG(brb_base->log_base, "Will add timer with delay [%u] - Persist [%u]\r\n", delay_ms, persist);

    /* Select a free timer slot from array */
    for (i = 0; i < MAX_TIMER; i++)
    {
        if (!brb_base->timer.arr[i].flags.active)
        {
            /* Fill up timer data */
            timer = &brb_base->timer.arr[i];
            timer->timer_id = i;
            timer->cb_func = cb_func;
            timer->cb_data = cb_data;

            /* Calculate when */
            timer->ms.when = brb_base->ms.cur + delay_ms;
            timer->ms.delay = delay_ms;

            /* Set flags */
            timer->flags.persist = ((persist == 1) ? 1 : 0);
            timer->flags.active = 1;

            LOG_DEBUG(brb_base->log_base, "Added timer ID [%u] with delay [%u] - C/W [%u / %u]\r\n",
                      timer->timer_id, timer->ms.delay, brb_base->ms.cur, timer->ms.when);
            break;
        }
        continue;
    }

    /* No more timers */
    if (!timer)
    {
        LOG_WARN(brb_base->log_base, "Failed adding timer - Table exhausted\r\n");
        return -1;
    }

    return timer->timer_id;
}
/**********************************************************************************************************************/
void BrbTimerDispatch(BrbBase *brb_base)
{
    BrbTimer *timer;
    int i;

    timer = NULL;

    /* Walk all timers  */
    for (i = 0; i < MAX_TIMER; i++)
    {
        /* Ignore inactive timers */
        if (!brb_base->timer.arr[i].flags.active)
            continue;

        /* Grab timer */
        timer = &brb_base->timer.arr[i];

        // LOG_INFO(brb_base->log_base, "TID [%u] - W/C/D/P [%u %u %u %u]\r\n", timer->timer_id, timer->ms.when, brb_base->ms.cur, timer->ms.delay, timer->flags.persist);

        /* Timer need to be dispatched */
        if (timer->ms.when <= brb_base->ms.cur)
        {
            LOG_INFO(brb_base->log_base, "TIME [%ld] - TID [%u] - Dispatch\r\n", brb_base->ms.cur, timer->timer_id);

            /* Invoke timer CBH */
            if (timer->cb_func)
                timer->cb_func(timer, timer->cb_data);

            /* Readjust WHEN if timer is persistent */
            if (timer->flags.persist)
            {
                timer->ms.when = brb_base->ms.cur + timer->ms.delay;
                //    LOG_INFO(brb_base->log_base, "TID [%u] - NW [%u] - D [%u]\r\n", timer->timer_id, timer->ms.when, timer->ms.delay);
            }
            /* Disable volatile timer */
            else
            {
                timer->flags.active = 0;
            }
        }

        continue;
    }
}
/**********************************************************************************************************************/