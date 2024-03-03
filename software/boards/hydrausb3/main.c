#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "core_riscv.h"
#include "CH56x_bsp.h"
#include "CH56x_uart.h"

#include "uartbone.h"

/* System clock and HSPI freq : 15 MHz */
#define FREQ_SYS (15000000)

/* UARTs settings */
#define UART1_BAUD 115200
#define UART3_BAUD 115200

/* HSPI Data width : 8 bits */
#define HSPI_DATA_WIDTH 0

//DMA_Addr0
#define TX_DMA_Addr0   0x20020000
#define RX_DMA_Addr0   0x20020000

//DMA_Addr1
#define TX_DMA_Addr1   0x20020000 + DMA_Tx_Len0
#define RX_DMA_Addr1   0x20020000 + DMA_Tx_Len1

void uartbone_ch56x_init(struct uartbone_ctx *ctx, int uart_num, int baudrate, int addr_width);

int main(void) {
	struct uartbone_ctx ctx;
	char ident_str[256];
	int i = 0;
	char c;
	uint32_t ident_addr = 0x00002000; /* we should find a way to embed this at build time from the soc.csv */

	bsp_gpio_init();

	UART1_init(UART1_BAUD, FREQ_SYS);
	UART3_init(UART3_BAUD, FREQ_SYS);

	printf("\n\r");
	printf("###########################\n\r");
	printf("# SucreLA fw starting up! #\n\r");
	printf("###########################\n\r\n\r");

	printf("board: hydrausb3\n\r");
	uartbone_ch56x_init(&ctx, 3, 115200, 4);
	printf("uartbone: initialized on UART3\n\r");

	printf("Identifying FPGA SoC...\n\r");
	memset(ident_str, '\0', sizeof(ident_str));
	do {
		c = uartbone_read(&ctx, ident_addr+i*4);
		ident_str[i++] = c;
	} while (c);
	printf("FPGA SoC ident: %s\n", ident_str);

	return 0;
}
