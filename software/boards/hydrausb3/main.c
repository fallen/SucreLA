#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* System clock and HSPI freq : 120 MHz */
#define FREQ_SYS (120000000)

#include "core_riscv.h"
#include "CH56x_bsp.h"
#include "CH56x_uart.h"
#include "CH56x_common.h"

#include "uartbone.h"
#include "csr.h" // for FPGA SoC CSR register addresses

/* UARTs settings */
#define UART1_BAUD 115200
#define UART3_BAUD 115200

/* HSPI Data width : 8 bits */
#define HSPI_DATA_WIDTH 0

#define RX_DMA_Addr0   0x20020000
#define RX_DMA_LENGTH   4096
#define RX_DMA_Addr1   RX_DMA_Addr0 + RX_DMA_LENGTH

/* Shared variables */
volatile int HSPI_TX_End_Flag; // Send completion flag
volatile int HSPI_RX_End_Flag; // Receive completion flag
volatile int HSPI_RX_End_Err; // 0=No Error else >0 Error code

void uartbone_ch56x_init(struct uartbone_ctx *ctx, int uart_num, int baudrate, int addr_width);

int main(void) {
	struct uartbone_ctx ctx;
	char ident_str[256];
	int i = 0;
	char c;
	uint32_t *dma_rx = (uint32_t *)RX_DMA_Addr0;
	uint8_t *hspi_rxed_bytes = (uint8_t *)RX_DMA_Addr0;

	bsp_gpio_init();
	bsp_init(FREQ_SYS);

	UART1_init(UART1_BAUD, FREQ_SYS);
	UART3_init(UART3_BAUD, FREQ_SYS);

	printf("\n\r");
	printf("###########################\n\r");
	printf("# SucreLA fw starting up! #\n\r");
	printf("###########################\n\r\n\r");

	printf("board: HydraUSB3\n\r");
	uartbone_ch56x_init(&ctx, 3, 115200, 4);
	printf("uartbone: initialized on UART3\n\r");

	printf("Identifying FPGA SoC...\n\r");
	memset(ident_str, '\0', sizeof(ident_str));
	do {
		c = uartbone_read(&ctx, CSR_IDENTIFIER_MEM_BASE+i*4);
		ident_str[i++] = c;
	} while (c);
	printf("FPGA SoC ident: %s\n\r", ident_str);

    printf("Preparing HSPI RX interface...\n\r");
    memset(dma_rx, '\0', RX_DMA_LENGTH*2);
    HSPI_RX_End_Flag = 0;  // Receive completion flag
    HSPI_RX_End_Err = 0; // 0=No Error else >0 Error code
    HSPI_DoubleDMA_Init(HSPI_DEVICE, HSPI_DATA_WIDTH, RX_DMA_Addr0, RX_DMA_Addr1, 0);

    printf("Starting a capture...\n\r");
    uartbone_write(&ctx, CSR_LA_TRIGGER_MEM_MASK_ADDR, 0);
    uartbone_write(&ctx, CSR_LA_TRIGGER_MEM_VALUE_ADDR, 0);
    uartbone_write(&ctx, CSR_LA_TRIGGER_MEM_WRITE_ADDR, 1);
    uartbone_write(&ctx, CSR_LA_STORAGE_OFFSET_ADDR, 0);
    uartbone_write(&ctx, CSR_LA_STORAGE_LENGTH_ADDR, 512);
    uartbone_write(&ctx, CSR_LA_STORAGE_ENABLE_ADDR, 1);
    uartbone_write(&ctx, CSR_LA_TRIGGER_ENABLE_ADDR, 1);

    printf("Waiting for HSPI RX data...\n\r");
    while(HSPI_RX_End_Flag == 0)
        ;
    printf("HSPI RX done!\n\r");

    for (i = 0; i < 4096; i++) {
        if (!(i % 50))
            printf("\n\r");
        printf("%02x ", hspi_rxed_bytes[i]);
    }

	return 0;
}

void HSPI_IRQHandler_ReInitRX(void)
{
	R32_HSPI_RX_ADDR0 = RX_DMA_Addr0;
	R32_HSPI_RX_ADDR1 = RX_DMA_Addr1;
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void HSPI_IRQHandler(void)
{
    if (R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE) // Single packet reception completed
    {
        R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  // Clear Interrupt
        HSPI_IRQHandler_ReInitRX();
        HSPI_RX_End_Flag = 1;
    }

    // Determine whether the CRC is correct
    if(R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR)
    {
        // CRC check err
        //printf("CRC err\n\r");
        HSPI_IRQHandler_ReInitRX();
        HSPI_RX_End_Err |= 1;
    }
}