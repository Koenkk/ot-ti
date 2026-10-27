/*
 *  Copyright (c) 2017, Texas Instruments Incorporated
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of the copyright holder nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <openthread/config.h>
#include OPENTHREAD_PROJECT_CORE_CONFIG_FILE

/* Standard Library Header files */
#include <assert.h>
#include <stddef.h>
#include <string.h>

/* POSIX Header files */
#include <mqueue.h>
#include <pthread.h>
#include <sched.h>

/* OpenThread public API Header files */
#include <openthread/coap.h>
#include <openthread/dataset.h>
#include <openthread/diag.h>
#include <openthread/joiner.h>
#include <openthread/platform/settings.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>

/* Example/Board Header files */
#include "ti_drivers_config.h"
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/dpl/ClockP.h>
#include "utils/code_utils.h"

#include "system.h"

#include <openthread/tasklet.h>

#include <stdio.h>

/**
 * @brief Size of the message queue for `OtStack_procQueue`
 *
 * Size determined by:
 *  7   main processing loop commands
 *  6   radio process requests
 *  2   UART process requests
 *  1   watchdog heartbeat
 * +1   buffer
 * -----------------------------------
 *  17  queue slots
 */
#define OTSTACK_PROC_QUEUE_MAX_MSG      (17)

/* How often the heartbeat asks the processing loop whether it still runs.
 * Keep it well under the watchdog's period (CONFIG_WATCHDOG0 in
 * openthread.syscfg, 2000 ms). Without a clear, the CC26x2 watchdog raises
 * an NMI at its first expiry and resets at the second; the CC26x4 one
 * resets at the first. A loop that stops answering is cleared for at most
 * one more heartbeat, so the device resets within about 4.5 s (CC26x2) or
 * 2.5 s (CC26x4); with interrupts left disabled, within two periods.
 */
#define OTSTACK_HEARTBEAT_PERIOD_MS     (500)

enum OtStack_procQueueCmd
{
    OtStack_procQueueCmd_alarm,
    OtStack_procQueueCmd_radio,
    OtStack_procQueueCmd_tasklets,
    OtStack_procQueueCmd_uart,
    OtStack_procQueueCmd_alarmu,
    OtStack_procQueueCmd_spi,
    OtStack_procQueueCmd_heartbeat,
};

struct OtStack_procQueueMsg {
    enum OtStack_procQueueCmd cmd;
    uintptr_t arg;
};

/* POSIX message queue for passing state to the processing loop */
const  char  OtStack_procQueueName[] = "/tiop_process";
static mqd_t OtStack_procQueueDesc;
static mqd_t OtStack_procQueueLoopDesc;

/* The watchdog is cleared only while the processing loop answers the
 * heartbeat: a hung RCP otherwise needs a power cycle, which the host can
 * not give it, while one that resets is brought up again by the host.
 */
static Watchdog_Handle    OtStack_watchdog;
static ClockP_Struct      OtStack_heartbeatClock;
static volatile bool      OtStack_heartbeatAnswered = true;
static bool               OtStack_heartbeatPosted;

static void heartbeatFxn(uintptr_t arg)
{
    struct OtStack_procQueueMsg msg;
    (void)arg;

    if (OtStack_heartbeatAnswered)
    {
        Watchdog_clear(OtStack_watchdog);

        OtStack_heartbeatAnswered = false;
        OtStack_heartbeatPosted   = false;
    }

    /* A full queue delays the heartbeat to the next tick; a loop that never
     * drains it never answers, and the watchdog resets the device.
     */
    if (!OtStack_heartbeatPosted)
    {
        msg.cmd                 = OtStack_procQueueCmd_heartbeat;
        OtStack_heartbeatPosted = (0 == mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0));
    }
}

static void watchdogInit(void)
{
    Watchdog_Params watchdogParams;
    ClockP_Params   clockParams;
    uint32_t        ticks = (OTSTACK_HEARTBEAT_PERIOD_MS * 1000) / ClockP_getSystemTickPeriod();

    if (OtStack_watchdog != NULL)
    {
        /* otSysInit runs again after a pseudo reset */
        return;
    }

    Watchdog_init();
    Watchdog_Params_init(&watchdogParams);
    watchdogParams.resetMode = Watchdog_RESET_ON;
    OtStack_watchdog         = Watchdog_open(CONFIG_WATCHDOG0, &watchdogParams);
    assert(OtStack_watchdog != NULL);

    ClockP_Params_init(&clockParams);
    clockParams.period    = ticks;
    clockParams.startFlag = true;
    ClockP_construct(&OtStack_heartbeatClock, heartbeatFxn, ticks, &clockParams);
}

void otSysInit(int argc, char *argv[])
{
    struct mq_attr attr;

    OT_UNUSED_VARIABLE(argc);
    OT_UNUSED_VARIABLE(argv);

    attr.mq_curmsgs = 0;
    attr.mq_flags   = 0;
    attr.mq_maxmsg  = OTSTACK_PROC_QUEUE_MAX_MSG;
    attr.mq_msgsize = sizeof(struct OtStack_procQueueMsg);

    /* Open The processing queue in non-blocking write mode for the notify
     * callback functions
     */
    OtStack_procQueueDesc = mq_open(OtStack_procQueueName,
                                    (O_WRONLY | O_NONBLOCK | O_CREAT),
                                    0, &attr);

    /* Open the processing queue in blocking read mode for the process loop */
    OtStack_procQueueLoopDesc = mq_open(OtStack_procQueueName, O_RDONLY, 0, NULL);

    platformAlarmInit();
    platformAlarmMicroInit();
    platformRandomInit();
    platformRadioInit();

    watchdogInit();
}

bool otSysPseudoResetWasRequested(void)
{
    return false;
}

/**
 * Callback from OpenThread stack to indicate tasklets are pending processing.
 *
 * Defined as weak to avoid conflict with exisgting application definitions.
 * The driver processing function will pend on the processing queue for stack
 * operations to process. This allows the RTOS task to be put in the pend queue
 * and for the SoC to be put into deep sleep. The message for tasklet
 * processing is lost due to the CLI and NCP definition of this function. This
 * is acceptable because the only way tasklets are added in those examples are
 * from other tasklets or driver functions. There are no other tasks to place
 * tasklets on the processing queue.
 */
OT_TOOL_WEAK void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
    struct OtStack_procQueueMsg msg;
    int                         ret;
    msg.cmd = OtStack_procQueueCmd_tasklets;
    ret = mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);

    (void) ret;
}

void platformAlarmSignal()
{
    struct OtStack_procQueueMsg msg;
    int                         ret;
    msg.cmd = OtStack_procQueueCmd_alarm;
    ret = mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);

    (void) ret;
}

void platformAlarmMicroSignal()
{
    struct OtStack_procQueueMsg msg;
    int                         ret;
    msg.cmd = OtStack_procQueueCmd_alarmu;
    ret = mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);

    (void) ret;
}

bool platformUartSignal(uintptr_t arg)
{
    struct OtStack_procQueueMsg msg;
    int                         ret;
    msg.cmd = OtStack_procQueueCmd_uart;
    msg.arg = arg;
    ret = mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);

    return (0 == ret);
}

void platformSpiSignal()
{
    struct OtStack_procQueueMsg msg;
    int                         ret;
    msg.cmd = OtStack_procQueueCmd_spi;
    ret = mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);

    (void) ret;
}

void platformRadioSignal(uintptr_t arg)
{
    struct OtStack_procQueueMsg msg;
    int                         ret;
    msg.cmd = OtStack_procQueueCmd_radio;
    msg.arg = arg;
    ret = mq_send(OtStack_procQueueDesc, (const char *)&msg, sizeof(msg), 0);
    assert(0 == ret);

    (void) ret;
}

/**
 * Main processing thread for OpenThread Stack.
 */
void otSysProcessDrivers(otInstance *aInstance)
{
    while (1)
    {
        struct OtStack_procQueueMsg msg;
        ssize_t ret;
#ifndef OT_TASKLET_SIGNALING
        if (otTaskletsArePending(aInstance))
        {
            // allow the caller to handle tasklets
            return;
        }
#endif
        ret = mq_receive(OtStack_procQueueLoopDesc, (char *)&msg, sizeof(msg), NULL);
        /* priorities are ignored */
        if (ret < 0 || ret != sizeof(msg))
        {
            /* there was an error on receive or we did not receive a full message */
            continue;
        }

        switch (msg.cmd)
        {
            case OtStack_procQueueCmd_alarm:
            {
                platformAlarmProcess(aInstance);
                break;
            }

            case OtStack_procQueueCmd_radio:
            {
                platformRadioProcess(aInstance, msg.arg);
                break;
            }

            case OtStack_procQueueCmd_tasklets:
            {
                otTaskletsProcess(aInstance);
                break;
            }

#if OPENTHREAD_CONFIG_NCP_HDLC_ENABLE || TIOP_ENABLE_UART
            case OtStack_procQueueCmd_uart:
            {
                platformUartProcess(msg.arg);
                break;
            }
#endif
            case OtStack_procQueueCmd_alarmu:
            {
                platformAlarmMicroProcess(aInstance);
                break;
            }

            case OtStack_procQueueCmd_spi:
            {
                platformSpiProcess();
                break;
            }

            case OtStack_procQueueCmd_heartbeat:
            {
                OtStack_heartbeatAnswered = true;
#if OPENTHREAD_CONFIG_NCP_HDLC_ENABLE || TIOP_ENABLE_UART
                /* catch up on uart work whose signal was lost */
                platformUartPoll();
#endif
                break;
            }

            default:
            {
                break;
            }
        }
    }
}


