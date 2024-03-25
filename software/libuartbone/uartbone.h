#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*
 * There can exist only 16 OUT endpoints and 16 IN endpoints
 * Bit 7 indicates IN vs OUT (1 = IN)
 * bits 0..3 indicate endpoint number
 * bits 4..6 should be 0, therefor -1 can be used to signify
 * "no endpoint"
 */
#define UARTBONE_NO_ENDPOINT (-1)

enum uart_backend_type {
    UNIX_UART,
    CH569_UART,
    UNIX_USB
};

struct uartbone_ctx;

struct uart_backend {
    enum uart_backend_type type;
    int (*read)(struct uartbone_ctx *ctx, uint8_t *data, size_t len);
    void (*write)(struct uartbone_ctx *ctx, uint8_t *data, size_t len);
    void (*flush)(void);
};

struct uartbone_ctx {
    unsigned int addr_width;
    unsigned int baudrate;
    int fd;
    struct uart_backend *uart;
    bool open;
    int error;
    uint16_t vendor_id;
    uint16_t product_id;
    unsigned char endpoint;
    void *usb_handle;
};

int get_reg_addr(FILE *csv, char *reg_name, uint32_t *res);
uint32_t uartbone_read(struct uartbone_ctx *ctx, uint64_t addr);
void uartbone_unix_init(struct uartbone_ctx *ctx, char *file, unsigned int baudrate, unsigned int addr_width);
void uartbone_write(struct uartbone_ctx *ctx, uint64_t addr, uint32_t val);

static inline bool ctx_uses_usb(struct uartbone_ctx *ctx) {
    return ctx && ctx->uart && ctx->uart->type == UNIX_USB;
}