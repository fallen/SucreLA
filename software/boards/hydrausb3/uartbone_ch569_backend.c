#include <stdio.h>
#include <stdlib.h>

#include "uartbone.h"
#include "CH56x_uart.h"

static void ch56x_uart_writeb(struct uartbone_ctx *ctx, uint8_t data) {
	//printf("sending byte 0x%02x to uart %d\n\r", data, ctx->fd);
	switch (ctx->fd) {
		case 0: {
			UART0_SendByte(data);
			break;
		}
		case 1: {
			UART1_SendByte(data);
			break;
		}
		case 2: {
			UART2_SendByte(data);
			break;
		}
		case 3: {
			UART3_SendByte(data);
			break;
		}
		default: {
			printf("Only valid UARTs are 0/1/2/3\n");
			abort();
		}
	}
}

static int ch56x_uart_readb(struct uartbone_ctx *ctx, uint8_t *data) {
	switch (ctx->fd) {
		case 0: {
			while (!(R8_UART0_LSR & RB_LSR_DATA_RDY)); // wait for FIFO not empty
			*data = UART0_RecvByte();
			break;
		}
		case 1: {
			while (!(R8_UART1_LSR & RB_LSR_DATA_RDY)); // wait for FIFO not empty
			*data = UART1_RecvByte();
			break;
		}
		case 2: {
			while (!(R8_UART2_LSR & RB_LSR_DATA_RDY)); // wait for FIFO not empty
			*data = UART2_RecvByte();
			break;
		}
		case 3: {
			while (!(R8_UART3_LSR & RB_LSR_DATA_RDY)); // wait for FIFO not empty
			*data = UART3_RecvByte();
			break;
		}
		default: {
			printf("Only valid UARTs are 0/1/2/3\n");
			abort();
		}
	}
	return 0;
}

struct uart_backend ch56x_uart_backend = {
	.type = CH569_UART,
	.readb = ch56x_uart_readb,
	.writeb = ch56x_uart_writeb
};

void uartbone_ch56x_init(struct uartbone_ctx *ctx, int uart_num, int baudrate, int addr_width) {
	ctx->addr_width = addr_width;
	ctx->baudrate = baudrate;
	ctx->fd = uart_num;
	ctx->open = true;
	ctx->error = 0;
	ctx->uart = &ch56x_uart_backend;
}
