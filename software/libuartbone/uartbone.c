#include <unistd.h> // write(), read(), close()
#include <stdio.h>
#include <string.h>

#if defined(__linux__)
#  include <endian.h>
#elif defined(__FreeBSD__) || defined(__NetBSD__)
#  include <sys/endian.h>
#elif defined(__OpenBSD__)
#  include <sys/types.h>
#  define be16toh(x) betoh16(x)
#  define be32toh(x) betoh32(x)
#  define be64toh(x) betoh64(x)
#elif defined(__riscv) // workaround for xpack newlib nano based toolchain
	#include <machine/endian.h>
	#define htobe16(_x) __htons(_x)
	#define htobe32(_x) __htonl(_x)
	#define htobe64(_x) ({ printf("unsupported 64 bit endianness conversion\n"); abort(); 0; })
	#define be32toh(_x) __ntohl(_x)
#endif

#include "uartbone.h"

#define MAX_CMD_LENGTH          10 // max for 64 bit addresses + cmd code + burst length
#define CMD_WRITE_BURST_INCR    1
#define CMD_READ_BURST_INCR     2
#define CMD_WRITE_BURST_FIXED   3
#define CMD_READ_BURST_FIXED    4

union wb_val_32 {
    uint32_t val;
    uint8_t buff[4];
};

union wb_val_64 {
    uint64_t val;
    uint8_t buff[8];
};

void uart_send(struct uartbone_ctx *ctx, uint8_t *buffer, size_t length) {
    size_t i;

    for (i = 0; i < length; i++, buffer++)
        ctx->uart->writeb(ctx, *buffer);
}

void uart_recv(struct uartbone_ctx *ctx, uint8_t *buffer, size_t length) {
    size_t i;
    int ret;

    for (i = 0; i < length; i++, buffer++) {
        ret = ctx->uart->readb(ctx, buffer);
        if (ret < 0)
            break;
    }
}

void uartbone_flush(struct uartbone_ctx *ctx) {
    //ctx->uart->flush();
}

uint64_t to_bigendian(struct uartbone_ctx *ctx, uint64_t val) {
    uint64_t newval;

    switch(ctx->addr_width) {
        case 1:
            newval = val;
            break;

        case 2:
            newval = htobe16(val);
            break;

        case 4:
            newval = htobe32(val);
            break;

        case 8:
            newval = htobe64(val);
    }

    return newval;
}

uint32_t uartbone_read(struct uartbone_ctx *ctx, uint64_t addr) {
    size_t cmd_length = ctx->addr_width + 2;
    unsigned char buffer[MAX_CMD_LENGTH];
    union wb_val_32 wbval;
    union wb_val_64 wbaddr;

    buffer[0] = CMD_READ_BURST_FIXED;
    buffer[1] = 1;

    uartbone_flush(ctx);
    wbaddr.val = to_bigendian(ctx, addr >> 2);
    memcpy(&buffer[2], wbaddr.buff, ctx->addr_width);
    uart_send(ctx, buffer, cmd_length);
    uart_recv(ctx, wbval.buff, 4);
    wbval.val = be32toh(wbval.val);

    return wbval.val;
}

void uartbone_write(struct uartbone_ctx *ctx, uint64_t addr, uint32_t val) {
    size_t cmd_length = ctx->addr_width + 2;
    unsigned char buffer[MAX_CMD_LENGTH];
    union wb_val_32 wbval;
    union wb_val_64 wbaddr;

    buffer[0] = CMD_WRITE_BURST_FIXED;
    buffer[1] = 1;

    uartbone_flush(ctx);
    wbval.val = to_bigendian(ctx, val);;
    wbaddr.val = to_bigendian(ctx, addr >> 2); // address must be shifted by 2 bits
    memcpy(&buffer[2], wbaddr.buff, ctx->addr_width);
    uart_send(ctx, buffer, cmd_length);
    uart_send(ctx, wbval.buff, 4);
}
