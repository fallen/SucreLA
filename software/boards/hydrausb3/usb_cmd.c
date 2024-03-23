/********************************** (C) COPYRIGHT *******************************
* File Name          : usb_cmd.c
* Author             : bvernoux
* Version            : V1.0
* Date               : 2022/08/20
* Description 		 :
* Copyright (c) 2022 Benjamin VERNOUX
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "CH56x_common.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"

#include "CH56x_debug_log.h"
#include "usb_cmd.h"


/*******************************************************************************
 * @fn     usb_cmd_rx
 *
 * @brief  Callback called by USB2 & USB3 endpoint 1
 *         - For USB3 this usb_cmd_rx() is called from IRQ(USBHS_IRQHandler)
 *           with rx_usb_dma_buff containing 4096 bytes (DEF_ENDP1_MAX_SIZE)
 *         - For USB2 this usb_cmd_rx() is called from IRQ USB30_IRQHandler->EP1_OUT_Callback)
 *           with rx_usb_dma_buff containing 4096 bytes (DEF_ENDP1_MAX_SIZE)
 *
 * @param  usb_type: USB Type (USB2 HS or USB3 SS)
 * @param  rx_usb_dma_buff: USB RX DMA buffer containing 4096 bytes of data
 *                          Data received from USB
 * @param  tx_usb_dma_buff: USB TX DMA buffer containing 4096 bytes of data
 *                          Data to be transmitted over USB
 *
 * @return size of data to be sent to USB host
 */


#define RISCV_FENCE(p, s) \
	__asm__ __volatile__ ("fence " #p "," #s : : : "memory")

/* These barriers need to enforce ordering on both devices or memory. */
#define mb()		RISCV_FENCE(iorw,iorw)
#define rmb()		RISCV_FENCE(ir,ir)
#define wmb()		RISCV_FENCE(ow,ow)

extern volatile int usb_cmd_rxed;
extern volatile uint8_t usb_cmd[11];
extern volatile uint8_t usb_cmd_len;
extern volatile int ep1_out_nack;

// return 1 for busy, 0 for OK
int usb_cmd_rx(e_usb_type usb_type, uint8_t* rx_usb_dma_buff, uint8_t* tx_usb_dma_buff)
{
	if (usb_cmd_rxed) { // already processing...
	    ep1_out_nack++;
	    return 1;
	}
	switch (rx_usb_dma_buff[0]) {
	    case 'U': {
	        if (rx_usb_dma_buff[1] == 'w') {
                usb_cmd_len = rx_usb_dma_buff[2];
                memcpy(usb_cmd, &rx_usb_dma_buff[1], usb_cmd_len + 2);
                mb();
                usb_cmd_rxed = 1;
            }
            if (rx_usb_dma_buff[1] == 'r') {
                usb_cmd_len = rx_usb_dma_buff[2];
                usb_cmd[0] = 'r';
                mb();
                usb_cmd_rxed = 1;
            }
            if (rx_usb_dma_buff[1] != 'r' && rx_usb_dma_buff[1] != 'w')
                printf("Big PROTO error!\n");
            break;
	    }
	    default: {
	        printf("Big error!\n");
	    }
	}
	return 0;
}
