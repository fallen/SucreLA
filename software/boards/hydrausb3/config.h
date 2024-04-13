#ifndef _CONFIG_H_
#define _CONFIG_H_

/* UARTs settings */
#define UART1_BAUD 115200
#define UART3_BAUD 115200

/* HSPI Data width : 8 bits */
#define HSPI_DATA_WIDTH 0

#define HSPI_RX_DMA_LENGTH   4096

extern __attribute__((aligned(16))) volatile uint8_t HSPI_RX_Addr0[HSPI_RX_DMA_LENGTH] __attribute__((section(".DMADATA")));
extern __attribute__((aligned(16))) volatile uint8_t HSPI_RX_Addr1[HSPI_RX_DMA_LENGTH] __attribute__((section(".DMADATA")));

#endif