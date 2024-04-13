#include <stdio.h>
#include <stdlib.h>

#include "uartbone.h"
#include "CH56x_uart.h"

static void ch56x_uart_write(struct uartbone_ctx *ctx, uint8_t *data, size_t len) {
	switch (ctx->fd) {
		case 0: {
			UART0_SendString(data, len);
			break;
		}
		case 1: {
			UART1_SendString(data, len);
			break;
		}
		case 2: {
			UART2_SendString(data, len);
			break;
		}
		case 3: {
			UART3_SendString(data, len);
			break;
		}
		default: {
			printf("Only valid UARTs are 0/1/2/3\n\r");
			abort();
		}
	}
}

static int ch56x_uart_read(struct uartbone_ctx *ctx, uint8_t *data, size_t len) {
	switch (ctx->fd) {
		case 0: {
			UART0_Recv(data, len);
			break;
		}
		case 1: {
			UART1_Recv(data, len);
			break;
		}
		case 2: {
			UART2_Recv(data, len);
			break;
		}
		case 3: {
			UART3_Recv(data, len);
			break;
		}
		default: {
			printf("Only valid UARTs are 0/1/2/3\n\r");
			abort();
		}
	}
	return 0;
}

struct uart_backend ch56x_uart_backend = {
	.type = CH569_UART,
	.read = ch56x_uart_read,
	.write = ch56x_uart_write
};

void uartbone_ch56x_init(struct uartbone_ctx *ctx, int uart_num, int baudrate, int addr_width) {
	ctx->addr_width = addr_width;
	ctx->baudrate = baudrate;
	ctx->fd = uart_num;
	ctx->open = true;
	ctx->error = 0;
	ctx->uart = &ch56x_uart_backend;
}
