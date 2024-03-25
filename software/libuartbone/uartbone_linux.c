#include <errno.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>

#include <libusb.h>
#if !defined(LIBUSB_API_VERSION) || (LIBUSB_API_VERSION < 0x0100010A)
#define libusb_init_context(a, b, c) libusb_init(a)
#endif

#include "uartbone.h"

int get_reg_addr(FILE *csv, char *reg_name, uint32_t *res) {
    char *line = NULL;
    char *tok;
    size_t len = 0;
    bool csr_base_found = false;
    bool csr_reg_found = false;
    uint32_t csr_base = 0;
    uint32_t reg_addr = 0;
    ssize_t nread;
    size_t reg_name_len = strlen(reg_name);

    if (!reg_name)
        return -1;

    while ((nread = getline(&line, &len, csv)) != -1) {

        // Find the csd_base address
        if (strncmp(line, "csr_base", strlen("csr_base")) == 0) {
            strtok(line, ",");
            tok = strtok(NULL, ",");
            if (tok && (strncmp(tok, reg_name, reg_name_len) == 0)) {
                if (tok[reg_name_len] != '\0')
                    continue;

                tok = strtok(NULL, ",");
                if (tok) {
                    csr_base = strtol(tok, NULL, 0);
                    csr_base_found = true;
                }
            }
        }

        // Find the register address
        if (strncmp(line, "csr_register", strlen("csr_register")) == 0) {
            strtok(line, ",");
            tok = strtok(NULL, ",");

            if (tok && reg_name && (strncmp(tok, reg_name, reg_name_len) == 0)) {
                if (tok[reg_name_len] != '\0')
                    continue;

                tok = strtok(NULL, ",");
                if (tok) {
                    reg_addr = strtol(tok, NULL, 0);
                    csr_reg_found = true;
                }
            }
        }
    }

    if (csr_reg_found)
        *res = reg_addr;
    else if (csr_base_found)
        *res = csr_base;
    else
        return -1;

    return 0;
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

void uart_write(struct uartbone_ctx *ctx, uint8_t *data, size_t len) {
    ssize_t ret;
    size_t cur_pos = 0;

    while (cur_pos != len) {
        ret = write(ctx->fd, data + cur_pos, len - cur_pos);
        if (ret == -1) {
            perror("write");
            return;
        }
        cur_pos += ret;
    }
}

int uart_read(struct uartbone_ctx *ctx, uint8_t *res, size_t len) {
    ssize_t ret;
    size_t cur_pos = 0;

    while (cur_pos != len) {
        ret = read(ctx->fd, res + cur_pos, len - cur_pos);
        if (ret <= 0) {
            perror("read");
            return ret;
        }
        cur_pos += ret;
    }
    return ret;
}

void usb_write(struct uartbone_ctx *ctx, uint8_t *data, size_t len) {
    uint8_t *usb_buffer;
    int ret;
    int transferred;
    size_t usb_tx_len = len + 3;
    int i;

#ifdef DEBUG
    printf("doing write to endpoint %d of size %zu: \n", ctx->endpoint, len);
    for (i = 0; i < len; i++)
        printf("%02x ", data[i]);
    printf("\n");
#endif

    usb_buffer = malloc(usb_tx_len);
    usb_buffer[0] = 'U'; // uartbone code
    usb_buffer[1] = 'w'; // uart write cmd
    usb_buffer[2] = len; // uart payload total size
    memcpy(&usb_buffer[3], data, len);

    ret = libusb_bulk_transfer((libusb_device_handle *)ctx->usb_handle, ctx->endpoint, usb_buffer, usb_tx_len, &transferred, 5000);
    free(usb_buffer);
    if (ret)
        printf("libusb_bulk_transfer error %d: %s\n", ret, libusb_error_name(ret));

    if (transferred != usb_tx_len)
        printf("error, libusb_bulk_transfer transferred %d instead of %zu\n", transferred, usb_tx_len);
 }

int usb_read(struct uartbone_ctx *ctx, uint8_t *res, size_t len) {
    uint8_t usb_buffer[1024];
    int ret;
    int transferred;
    uint8_t tmpbuf[3] = {'U', 'r', len};

#ifdef DEBUG
    printf("doing read to endpoint %d of size %zu\n", ctx->endpoint, len);
    printf("sending Read command\n");
#endif

    ret = libusb_bulk_transfer((libusb_device_handle *)ctx->usb_handle, ctx->endpoint, tmpbuf, sizeof(tmpbuf), &transferred, 5000);
    if (ret) {
        printf("libusb_bulk_transfer error %d: %s\n", ret, libusb_error_name(ret));
        return -1;
    }

    if (transferred != sizeof(tmpbuf)) {
        printf("error, libusb_bulk_transfer transferred %d instead of %zu\n", transferred, sizeof(tmpbuf));
        return -1;
    }

#ifdef DEBUG
    printf("Reading EP1 IN buffer\n");
#endif

    ret = libusb_bulk_transfer((libusb_device_handle *)ctx->usb_handle, ctx->endpoint | LIBUSB_ENDPOINT_IN, usb_buffer, 1024, &transferred, 5000);
    if (ret) {
        printf("libusb_bulk_transfer error %d: %s\n", ret, libusb_error_name(ret));
        return -1;
    }

    if (transferred != len) {
        printf("error, libusb_bulk_transfer transferred %d instead of %zu\n", transferred, len);
        return -1;
    }

    memcpy(res, usb_buffer, len);

    return 0;
}

struct uart_backend serial_backend = {
    .type = UNIX_UART,
    .read = uart_read,
    .write = uart_write
};

struct uart_backend usb_backend = {
    .type = UNIX_USB,
    .read = usb_read,
    .write = usb_write
};

/*
 * parse USB scheme
 * must be of the form usb://vendor_id:product_id[/endpoint]
 * vendor_id and product_id can be expressed in hex but must
 * have a 0x prefix.
 *
 * The /endpoint at the end is optional. Of omitted endpoint is
 * considered to be OUT 0.
 * return -1 upon parsing error, 0 if SUCCESS
 */
int parse_usb_scheme(char *scheme, uint16_t *vid, uint16_t *pid, uint8_t *endpoint) {
    long res;
    char *endptr;
    char *tmp;

    /* minimum size usb device scheme: usb://x:y => 9 */
    if ((strlen(scheme) < strlen("usb://x:y")) || (strncmp(scheme, "usb://", strlen("usb://")) != 0))
        return -1;

    scheme += strlen("usb://");
    errno = 0;
    res = strtol(scheme, &endptr, 0);
    if ((errno != 0) || (res > UINT16_MAX) || (scheme == endptr) || (*endptr == '\0'))
        return -1;

    *vid = res;
    endptr++; // skip the ':' char
    tmp = endptr;
    errno = 0;
    res = strtol(endptr, &endptr, 0);
    if ((errno != 0) || (res > UINT16_MAX) || (tmp == endptr))
        return -1;

    *pid = res;
    if (*endptr != '/') {
        *endpoint = 0;
        return 0;
    }

    endptr++; // skip the '/' char
    if (*endptr == '\0') {
        *endpoint = 0;
        return 0;
    }
    tmp = endptr;
    errno = 0;
    res = strtol(endptr, &endptr, 0);
    if ((errno != 0) || (res > UINT8_MAX) || (tmp == endptr))
        return -1;
    *endpoint = res;

    return 0;
}

void uartbone_unix_init(struct uartbone_ctx *ctx, char *file, unsigned int baudrate, unsigned int addr_width) {
    struct termios tty;
    int fd;
    bool use_usb = false;

    ctx->error = 0;
    ctx->open = false;
    ctx->addr_width = addr_width;

    /* minimum size usb device scheme: usb://x:y => 9 */
    if (!parse_usb_scheme(file, &ctx->vendor_id, &ctx->product_id, &ctx->endpoint)) {
        int ret;
        libusb_device_handle *handle;

        printf("Using USB port: %s\n", file);
        printf("vid: %04x pid: %04x endpoint: %02x\n", ctx->vendor_id, ctx->product_id, ctx->endpoint);
        use_usb = true;
        ctx->uart = &usb_backend;
        ret = libusb_init_context(NULL, NULL, 0);
        if (ret) {
            ctx->error = ret;
            printf("libusb_init_context error %d: %s\n", ret, libusb_error_name(ret));
            return;
        }
        handle = libusb_open_device_with_vid_pid(NULL, ctx->vendor_id, ctx->product_id);
        if (!handle) {
            ctx->error = -1;
            printf("libusb_open_device_with_vid_pid error: Device not found!\n");
            return;
        }
        ctx->usb_handle = handle;

        ret = libusb_set_auto_detach_kernel_driver(handle, 1);
        if (ret) {
            ctx->error = -1;
            printf("libusb_set_auto_detach_kernel_driver error %d: %s\n", ret, libusb_error_name(ret));
            return;
        }
        if (libusb_kernel_driver_active(handle, 0) == 1) {
            printf("Kernel driver active!\n");
        }
        ret = libusb_claim_interface(handle, 0);
        if (ret) {
            ctx->error = ret;
            printf("libusb_claim_interface error %d: %s\n", ret, libusb_error_name(ret));
            return;
        }
        ctx->open = true;
        return;
    }

    ctx->uart = &serial_backend;
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
