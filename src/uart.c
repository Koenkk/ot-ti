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

#include <stddef.h>

#include <utils/code_utils.h>
#include <utils/uart.h>

#include <ti/drivers/dpl/SemaphoreP.h>

#include "ti_drivers_config.h"
#include <ti/drivers/UART2.h>
#include <ti/drivers/dpl/HwiP.h>

#include "system.h"
#include <stdbool.h>

/* Ensure all bytes are written in blocking mode before notifying the stack it
 * can send more data. Less efficient than callback mode. Necessary for certain
 * versions of the UART2 driver.
 */
#define TI_PLAT_UART_BLOCKING 1

#define PLATFORM_UART_EVENT_TX_DONE (1U << 0)
#define PLATFORM_UART_EVENT_RX_DONE (1U << 1)

#define PLATFORM_UART_RECV_BUF_LEN 256

/* Bytes received and not yet handed to the stack. A power of two, so the
 * free-running indices below wrap with it.
 */
#define PLATFORM_UART_RX_RING_LEN 1024

static uint8_t PlatformUart_receiveBuffer[PLATFORM_UART_RECV_BUF_LEN];

static UART2_Handle PlatformUart_uartHandle;

#if !TI_PLAT_UART_BLOCKING
static SemaphoreP_Struct PlatformUart_writeSem;
static SemaphoreP_Handle PlatformUart_writeSemHandle;
#endif

/* Received bytes pass from the read callback (Hwi) to the processing loop
 * through this ring: the callback alone advances the head, the loop alone
 * the tail. The callback signals the loop only when no signal is pending,
 * so the UART holds at most one RX slot of the processing queue, whose size
 * system.c budgets for, however many reads complete before the loop runs.
 */
static uint8_t           PlatformUart_rxRing[PLATFORM_UART_RX_RING_LEN];
static volatile uint32_t PlatformUart_rxHead;
static volatile uint32_t PlatformUart_rxTail;
static volatile bool     PlatformUart_rxSignalPending;

/* A send-done whose signal found the processing queue full. */
static bool PlatformUart_txDoneLost;

static void uartReadCallback(UART2_Handle aHandle, void *aBuf, size_t aLen, void *userArg, int_fast16_t status)
{
    (void)aHandle;
    (void)aBuf;
    (void)userArg;
    (void)status;
    uint32_t head  = PlatformUart_rxHead;
    uint32_t space = PLATFORM_UART_RX_RING_LEN - (head - PlatformUart_rxTail);
    size_t   i;

    /* Bytes beyond the ring's space are dropped: the frame they belong to
     * fails its HDLC check, and the host, missing its response, treats the
     * RCP as failed and recovers it.
     */
    if (aLen > space)
    {
        aLen = space;
    }

    for (i = 0; i < aLen; i++)
    {
        PlatformUart_rxRing[(head + i) % PLATFORM_UART_RX_RING_LEN] = PlatformUart_receiveBuffer[i];
    }
    PlatformUart_rxHead = head + aLen;

    if (!PlatformUart_rxSignalPending)
    {
        PlatformUart_rxSignalPending = true;
        if (!platformUartSignal(PLATFORM_UART_EVENT_RX_DONE))
        {
            /* not queued: the next read signals again */
            PlatformUart_rxSignalPending = false;
        }
    }

    UART2_read(PlatformUart_uartHandle, PlatformUart_receiveBuffer, sizeof(PlatformUart_receiveBuffer), NULL);
}

#if !TI_PLAT_UART_BLOCKING
static void uartWriteCallback(UART2_Handle aHandle, void *aBuf, size_t aLen, void *userArg, int_fast16_t status)
{
    (void)aHandle;
    (void)aBuf;
    (void)aLen;
    (void)userArg;

    SemaphoreP_post(PlatformUart_writeSemHandle);
    platformUartSignal(PLATFORM_UART_EVENT_TX_DONE);
}
#endif /* !TI_PLAT_UART_BLOCKING */

otError otPlatUartEnable(void)
{
    UART2_Params params;

#if !TI_PLAT_UART_BLOCKING
    PlatformUart_writeSemHandle = SemaphoreP_constructBinary(&PlatformUart_writeSem, 1U);
#endif

    UART2_Params_init(&params);

    params.readMode       = UART2_Mode_CALLBACK;
    params.readCallback   = uartReadCallback;
    params.readReturnMode = UART2_ReadReturnMode_PARTIAL;
    params.eventMask      = UART2_EVENT_TX_FINISHED;
    params.baudRate       = 460800;
    params.dataLength     = UART2_DataLen_8;
    params.stopBits       = UART2_StopBits_1;
    params.parityType     = UART2_Parity_NONE;

#if TI_PLAT_UART_BLOCKING
    params.writeMode     = UART2_Mode_BLOCKING;
    params.writeCallback = NULL;
#else
    params.writeMode     = UART2_Mode_CALLBACK;
    params.writeCallback = uartWriteCallback;
#endif

    PlatformUart_uartHandle = UART2_open(CONFIG_UART2_0, &params);

    UART2_read(PlatformUart_uartHandle, PlatformUart_receiveBuffer, sizeof(PlatformUart_receiveBuffer), NULL);

    return OT_ERROR_NONE;
}

otError otPlatUartDisable(void)
{
    UART2_close(PlatformUart_uartHandle);

    return OT_ERROR_NONE;
}

otError otPlatUartSend(const uint8_t *aBuf, uint16_t aBufLength)
{
    int_fast16_t ret;

    /* Block any incoming Tx requests if one is already in progress */
#if !TI_PLAT_UART_BLOCKING
    SemaphoreP_pend(PlatformUart_writeSemHandle, UINT32_MAX);
#endif
    ret = UART2_write(PlatformUart_uartHandle, aBuf, aBufLength, NULL);

#if TI_PLAT_UART_BLOCKING
    if (!platformUartSignal(PLATFORM_UART_EVENT_TX_DONE))
    {
        PlatformUart_txDoneLost = true;
    }
#endif

    return OT_ERROR_NONE;
}

void platformUartProcess(uintptr_t arg)
{
    if (arg & PLATFORM_UART_EVENT_TX_DONE)
    {
        otPlatUartSendDone();
    }

    if (arg & PLATFORM_UART_EVENT_RX_DONE)
    {
        /* Cleared before draining: a read completing from here on signals
         * again, and one completing before has its bytes drained below.
         */
        PlatformUart_rxSignalPending = false;

        while (PlatformUart_rxTail != PlatformUart_rxHead)
        {
            uint32_t tail  = PlatformUart_rxTail;
            uint32_t start = tail % PLATFORM_UART_RX_RING_LEN;
            uint32_t len   = PlatformUart_rxHead - tail;

            if (len > PLATFORM_UART_RX_RING_LEN - start)
            {
                len = PLATFORM_UART_RX_RING_LEN - start;
            }

            otPlatUartReceived(&PlatformUart_rxRing[start], (uint16_t)len);
            PlatformUart_rxTail = tail + len;
        }
    }
}

void platformUartPoll(void)
{
    uintptr_t events = 0;

    /* With a signal pending, its own message drains the ring. */
    if (!PlatformUart_rxSignalPending && PlatformUart_rxTail != PlatformUart_rxHead)
    {
        events |= PLATFORM_UART_EVENT_RX_DONE;
    }

    if (PlatformUart_txDoneLost)
    {
        PlatformUart_txDoneLost = false;
        events |= PLATFORM_UART_EVENT_TX_DONE;
    }

    if (events != 0)
    {
        platformUartProcess(events);
    }
}

otError otPlatUartFlush(void)
{
    return OT_ERROR_NOT_IMPLEMENTED;
}
