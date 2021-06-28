/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_mcan.h"
#include "board.h"
#include "stdlib.h"

#include "pin_mux.h"
#include <stdbool.h>
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define USE_CANFD (1U)
/*
 *    CAN_DATASIZE   DLC    BYTES_IN_MB
 *    8              8      kMCAN_8ByteDatafield
 *    12             9      kMCAN_12ByteDatafield
 *    16             10     kMCAN_16ByteDatafield
 *    20             11     kMCAN_20ByteDatafield
 *    24             12     kMCAN_24ByteDatafield
 *    32             13     kMCAN_32ByteDatafield
 *    48             14     kMCAN_48ByteDatafield
 *    64             15     kMCAN_64ByteDatafield
 *
 *  CAN data size (pay load size), DLC and Bytes in Message buffer must align.
 *
 */
#define DLC         (15)
#define BYTES_IN_MB kMCAN_64ByteDatafield
/* If not define USE_CANFD or define it 0, CAN_DATASIZE should be 8. */
#define CAN_DATASIZE (64U)
/* If user need to auto execute the improved timming configuration. */
#define USE_IMPROVED_TIMING_CONFIG (1U)
#define EXAMPLE_MCAN_IRQHandler    CAN0_IRQ0_IRQHandler
#define EXAMPLE_MCAN_IRQn          CAN0_IRQ0_IRQn
#define EXAMPLE_MCAN               CAN0
#define MCAN_CLK_FREQ              CLOCK_GetMCanClkFreq(0U)
#define STDID_OFFSET               (18U)
#ifndef MSG_RAM_BASE
#define MSG_RAM_BASE 0x20010000U
#endif
#define STD_FILTER_OFS 0x0
#define RX_FIFO0_OFS   0x10U
#define TX_BUFFER_OFS  0x20U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool txComplete = false;
volatile bool rxComplete = false;
mcan_tx_buffer_frame_t txFrame;
mcan_rx_buffer_frame_t rxFrame;
uint8_t tx_data[CAN_DATASIZE];
uint8_t rx_data[CAN_DATASIZE];
mcan_handle_t mcanHandle;
mcan_buffer_transfer_t txXfer;
mcan_fifo_transfer_t rxXfer;
uint32_t txIdentifier;
uint32_t rxIdentifier;


    volatile unsigned int end_time[30] = {0};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief MCAN Call Back function
 */
static void mcan_callback(CAN_Type *base, mcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {
        case kStatus_MCAN_RxFifo0Idle:
        {
            rxComplete = true;
        }
        break;

        case kStatus_MCAN_TxIdle:
        {
            txComplete = true;
        }
        break;

        default:
            break;
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    unsigned int start_time, stop_time;

    mcan_config_t mcanConfig;
    mcan_frame_filter_config_t rxFilter;
    mcan_std_filter_element_config_t stdFilter;
    mcan_ext_filter_element_config_t extFilter;
    mcan_rx_fifo_config_t rxFifo0;
    mcan_rx_fifo_config_t rxFifo1;
    mcan_tx_buffer_config_t txBuffer;
    mcan_rx_buffer_config_t rxBuffer;
    mcan_tx_fifo_config_t txFifo0;
    mcan_tx_buffer_frame_t pTxFrame;
    mcan_rx_buffer_frame_t pRxFrame;
    mcan_handle_t handle;
    uint8_t node_type;
    uint8_t numMessage = 0;
    uint8_t cnt        = 0;

    /* Initialize board hardware. */
    /* attach 12 MHz clock to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    /* Set MCAN clock 180/6=30MHz. */
    CLOCK_SetClkDiv(kCLOCK_DivCan0Clk, 6U, true);

    BOARD_InitPins();
    BOARD_BootClockPLL180M();
    BOARD_InitDebugConsole();
    SysTick->CTRL = 0;//adfdsf    

    /* Select mailbox ID. */
        txIdentifier = 0x321U;
        rxIdentifier = 0x123U;

    MCAN_GetDefaultConfig(&mcanConfig);
#if (defined(USE_CANFD) && USE_CANFD)
    mcanConfig.enableCanfdNormal = true;
#endif

#if (defined(USE_IMPROVED_TIMING_CONFIG) && USE_IMPROVED_TIMING_CONFIG)
    mcan_timing_config_t timing_config;
    memset(&timing_config, 0, sizeof(timing_config));
#if (defined(USE_CANFD) && USE_CANFD)
    if (MCAN_FDCalculateImprovedTimingValues(mcanConfig.baudRateA, mcanConfig.baudRateD, MCAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(mcanConfig.timingConfig), &timing_config, sizeof(mcan_timing_config_t));
    }
    else
    {
        PRINTF("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#else
    if (MCAN_CalculateImprovedTimingValues(mcanConfig.baudRateA, MCAN_CLK_FREQ, &timing_config))
    {
        /* Update the improved timing configuration*/
        memcpy(&(mcanConfig.timingConfig), &timing_config, sizeof(mcan_timing_config_t));
    }
    else
    {
        PRINTF("No found Improved Timing Configuration. Just used default configuration\r\n\r\n");
    }
#endif
#endif

    MCAN_Init(EXAMPLE_MCAN, &mcanConfig, MCAN_CLK_FREQ);

    /* Create MCAN handle structure and set call back function. */
    MCAN_TransferCreateHandle(EXAMPLE_MCAN, &mcanHandle, mcan_callback, NULL);

    /* Set Message RAM base address and clear to avoid BEU/BEC error. */
    MCAN_SetMsgRAMBase(EXAMPLE_MCAN, MSG_RAM_BASE);
    uint32_t *p = (uint32_t *)(MSG_RAM_BASE);
    memset(p, 0, (8U + CAN_DATASIZE) * sizeof(uint8_t));

    /* STD filter config. */
    rxFilter.address  = STD_FILTER_OFS;
    rxFilter.idFormat = kMCAN_FrameIDStandard;
    rxFilter.listSize = 1U;
    rxFilter.nmFrame  = kMCAN_reject0;
    rxFilter.remFrame = kMCAN_rejectFrame;
    MCAN_SetFilterConfig(EXAMPLE_MCAN, &rxFilter);

    stdFilter.sfec = kMCAN_storeinFifo0;
    /* Classic filter mode, only filter matching ID. */
    stdFilter.sft   = kMCAN_classic;
    stdFilter.sfid1 = rxIdentifier;
    stdFilter.sfid2 = 0x7FFU;
    MCAN_SetSTDFilterElement(EXAMPLE_MCAN, &rxFilter, &stdFilter, 0);

    /* RX fifo0 config. */
    rxFifo0.address       = RX_FIFO0_OFS;
    rxFifo0.elementSize   = 1U;
    rxFifo0.watermark     = 0;
    rxFifo0.opmode        = kMCAN_FifoBlocking;
    rxFifo0.datafieldSize = kMCAN_8ByteDatafield;
#if (defined(USE_CANFD) && USE_CANFD)
    rxFifo0.datafieldSize = BYTES_IN_MB;
#endif
    MCAN_SetRxFifo0Config(EXAMPLE_MCAN, &rxFifo0);

    /* TX buffer config. */
    memset(&txBuffer, 0, sizeof(txBuffer));
    txBuffer.address       = TX_BUFFER_OFS;
    txBuffer.dedicatedSize = 1U;
    txBuffer.fqSize        = 0;
    txBuffer.datafieldSize = kMCAN_8ByteDatafield;
#if (defined(USE_CANFD) && USE_CANFD)
    txBuffer.datafieldSize = BYTES_IN_MB;
#endif
    MCAN_SetTxBufferConfig(EXAMPLE_MCAN, &txBuffer);

    /* Finish software initialization and enter normal mode, synchronizes to
       CAN bus, ready for communication */
    MCAN_EnterNormalMode(EXAMPLE_MCAN);

    if ((node_type == 'A') || (node_type == 'a'))
    {
        PRINTF("Press any key to trigger one-shot transmission\r\n\r\n");
    }
    else
    {
        PRINTF("Start to Wait data from Node A\r\n\r\n");
    }
    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_GetDefaultConfig(&mcanConfig);
    stop_time = SysTick->VAL;
    end_time[0] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_FDCalculateImprovedTimingValues(mcanConfig.baudRateA, mcanConfig.baudRateD, MCAN_CLK_FREQ, &timing_config);
    stop_time = SysTick->VAL;
    end_time[1] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_Init(EXAMPLE_MCAN, &mcanConfig, MCAN_CLK_FREQ);
    stop_time = SysTick->VAL;
    end_time[2] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_TransferCreateHandle(EXAMPLE_MCAN, &mcanHandle, mcan_callback, NULL);

    stop_time = SysTick->VAL;
    end_time[3] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;
    MCAN_SetMsgRAMBase(EXAMPLE_MCAN, MSG_RAM_BASE);
    stop_time = SysTick->VAL;
    end_time[4] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetFilterConfig(EXAMPLE_MCAN, &rxFilter);
    stop_time = SysTick->VAL;
    end_time[5] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetFilterConfig(EXAMPLE_MCAN, &rxFilter);
    stop_time = SysTick->VAL;
    end_time[6] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetSTDFilterElement(EXAMPLE_MCAN, &rxFilter, &stdFilter, 0);
    stop_time = SysTick->VAL;
    end_time[7] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetRxFifo0Config(EXAMPLE_MCAN, &rxFifo0);
    stop_time = SysTick->VAL;
    end_time[8] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetTxBufferConfig(EXAMPLE_MCAN, &txBuffer);
    stop_time = SysTick->VAL;
    end_time[9] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetTxBufferConfig(EXAMPLE_MCAN, &txBuffer);
    stop_time = SysTick->VAL;
    end_time[10] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_EnterNormalMode(EXAMPLE_MCAN);
    stop_time = SysTick->VAL;
    end_time[11] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_TransferSendNonBlocking(EXAMPLE_MCAN, &mcanHandle, &txXfer);
    stop_time = SysTick->VAL;
    end_time[12] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_TransferReceiveFifoNonBlocking(EXAMPLE_MCAN, 0, &mcanHandle, &rxXfer);
    stop_time = SysTick->VAL;
    end_time[13] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_Deinit(EXAMPLE_MCAN);

    stop_time = SysTick->VAL;
    end_time[14] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetArbitrationTimingConfig(EXAMPLE_MCAN, &timing_config);
    stop_time = SysTick->VAL;
    end_time[15] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetDataTimingConfig(EXAMPLE_MCAN, &timing_config);
    stop_time = SysTick->VAL;
    end_time[16] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetRxFifo1Config(EXAMPLE_MCAN, &rxFifo1);
    stop_time = SysTick->VAL;
    end_time[17] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetRxBufferConfig(EXAMPLE_MCAN, &rxBuffer);
    stop_time = SysTick->VAL;
    end_time[18] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetTxEventFifoConfig(EXAMPLE_MCAN, &txFifo0);
    stop_time = SysTick->VAL;
    end_time[19] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_SetEXTFilterElement(EXAMPLE_MCAN, &rxFilter, &extFilter, 0);
    stop_time = SysTick->VAL;
    end_time[20] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_IsTransmitRequestPending(EXAMPLE_MCAN, 0);
    stop_time = SysTick->VAL;
    end_time[21] = start_time - stop_time;


    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_IsTransmitOccurred(EXAMPLE_MCAN, 0);
    stop_time = SysTick->VAL;
    end_time[22] = start_time - stop_time;

    
    uint8_t *pdata, value_data;
    value_data= 0xA;
    pdata = &value_data;
    pTxFrame.id = 0x5a;
    pTxFrame.rtr = 0;
    pTxFrame.data = pdata;
    pTxFrame.size = 1;
    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_WriteTxBuffer(EXAMPLE_MCAN, 0, &pTxFrame);
    stop_time = SysTick->VAL;
    end_time[23] = start_time - stop_time;



    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_ReadRxBuffer(EXAMPLE_MCAN, 0, &pRxFrame);
    stop_time = SysTick->VAL;
    end_time[24] = start_time - stop_time;



    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_ReadRxFifo(EXAMPLE_MCAN, 0, &pRxFrame);
    stop_time = SysTick->VAL;
    end_time[25] = start_time - stop_time;



    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_TransferAbortSend(EXAMPLE_MCAN, &handle, 0);
    stop_time = SysTick->VAL;
    end_time[26] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_TransferAbortReceiveFifo(EXAMPLE_MCAN, 0, &handle);
    stop_time = SysTick->VAL;
    end_time[27] = start_time - stop_time;

    SysTick->CTRL = 0;//adfdsf
    SysTick->LOAD = 0xFFFFFFFF;
    SysTick->VAL = 0;
    while(SysTick->VAL != 0);
    start_time = SysTick->VAL;    
    MCAN_TransferHandleIRQ(EXAMPLE_MCAN, &handle);
    stop_time = SysTick->VAL;
    end_time[28] = start_time - stop_time;

}
