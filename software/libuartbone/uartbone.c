#include <arpa/inet.h> // for byte order related functions
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
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

static speed_t baudrate_to_speed(unsigned int baudrate) {
    switch (baudrate) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        return B0;
    }
}

void uartbone_unix_init(struct uartbone_ctx *ctx, char *file, unsigned int baudrate, unsigned int addr_width) {
    struct termios tty;
    int fd;

    ctx->error = 0;
    ctx->open = false;
    ctx->addr_width = addr_width;

    fd = open(file, O_RDWR);

    if (fd == -1) {
        printf("Error %d could not open uart %s: %s\n", errno, file, strerror(errno));
        ctx->error = errno;
        return;
    }

    ctx->fd = fd;

    if (tcgetattr(fd, &tty) != 0) {
      printf("Error %d from tcgetattr: %s\n", errno, strerror(errno));
      ctx->error = errno;
      close(fd);
      return;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate
    cfsetispeed(&tty, baudrate_to_speed(baudrate));
    cfsetospeed(&tty, baudrate_to_speed(baudrate));

    // Save tty settings, also checking for error
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        ctx->error = errno;
        close(fd);
        return;
    }

    ctx->open = true;
}