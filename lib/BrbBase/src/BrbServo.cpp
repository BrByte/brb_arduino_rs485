/*
 * BrbServo.cpp
 *
 *  Created on: 2019-01-04
 *      Author: Luiz Fernando Souza Softov <softov@brbyte.com>
 *      Author: Guilherme Amorim de Oliveira Alves <guilherme@brbyte.com>
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

#include "BrbBase.h"

/**********************************************************************************************************************/
BrbServo *BrbServoGrabByID(BrbBase *brb_base, int servo_id)
{
    BrbServo *servo;

    /* Sanity check */
    if (servo_id > MAX_SERVO)
        return NULL;

    servo = &brb_base->servo.arr[servo_id];

    if (!servo->servo)
        servo->servo = new Servo();

    return servo;
}
/**********************************************************************************************************************/
BrbServo *BrbServoGrabFree(BrbBase *brb_base)
{
    BrbServo *servo;
    int i;

    servo = NULL;

    /* Select a free timer slot from array */
    for (i = 0; i < MAX_SERVO; i++)
    {
        if (brb_base->servo.arr[i].flags.active)
            continue;
            
        /* Fill up servo data */
        servo              = (BrbServo *)&brb_base->servo.arr[i];
        servo->servo_id   = i;
        break;
    }

    /* No more timers */
    if (!servo)
    {
        LOG_WARN(brb_base->log_base, "Failed Grab Servo\r\n");
        return NULL;
    }

    if (!servo->servo)
        servo->servo = new Servo();

    return servo;
}
/**********************************************************************************************************************/
BrbServo *BrbServoGrabByPin(BrbBase *brb_base, int pin)
{
    BrbServo *servo = NULL;
    int i;

    /* Select a free timer slot from array */
    for (i = 0; i < MAX_SERVO; i++)
    {            
        /* Fill up servo data */
        servo              = (BrbServo *)&brb_base->servo.arr[i];
        
        if (pin == servo->pin)
            return servo;

        continue;
    }

    LOG_WARN(brb_base->log_base, "Failed Grab Servo - PIN [%d]\r\n", pin);

    return NULL;
}
/**********************************************************************************************************************/
int BrbServoAttach(BrbBase *brb_base, BrbServo *servo, int pin)
{
    if (servo->flags.attached)
        return -1;

    servo->servo->attach(pin);
    servo->pin                  = pin;
    servo->flags.attached      = 1;

    servo->pos_cur              = servo->servo->read();

    LOG_DEBUG(brb_base->log_base, "Attach servo - Pin [%u] pos_cur [%u]\r\n", pin, servo->pos_cur);

    return servo->servo_id;
}
/**********************************************************************************************************************/
BrbServo *BrbServoSetPosByPin(BrbBase *brb_base, int pin, int pos_set)
{
    BrbServo *servo;

    servo           = BrbServoGrabByPin(brb_base, pin);

    if (!servo)
    {
        servo       = BrbServoGrabFree(brb_base);
        
        if (!servo)
            return NULL;
    }

    BrbServoAttach(brb_base, servo, pin);

    // servo->pos_cur      = servo->servo->read();

    LOG_DEBUG(brb_base->log_base, "Servo [%u] - Pos [%u]->[%u]\r\n", servo->servo_id, servo->pos_cur, pos_set);

    if (pos_set > servo->pos_cur)
        while (pos_set > servo->pos_cur)
            servo->servo->write(++servo->pos_cur);
    else if (pos_set < servo->pos_cur)
        while (pos_set < servo->pos_cur)
            servo->servo->write(--servo->pos_cur);

    return servo;
}
/**********************************************************************************************************************/
BrbServo *BrbServoSetPos(BrbBase *brb_base, BrbServo *servo, int pos_set)
{
    if (!servo->flags.attached)
        BrbServoAttach(brb_base, servo, servo->pin);

    servo->pos_cur      = servo->servo->read();

    LOG_DEBUG(brb_base->log_base, "Servo [%u] - Pos [%u]->[%u]\r\n", servo->servo_id, servo->pos_cur, pos_set);

    if (pos_set > servo->pos_cur)
        while (pos_set > servo->pos_cur)
            servo->servo->write(++servo->pos_cur);
    else if (pos_set < servo->pos_cur)
        while (pos_set < servo->pos_cur)
            servo->servo->write(--servo->pos_cur);

    servo->pos_cur      = servo->servo->read();

    LOG_DEBUG(brb_base->log_base, "Servo [%u] - Pos [%u]\r\n", servo->servo_id, servo->pos_cur);

    return servo;
}
/**********************************************************************************************************************/