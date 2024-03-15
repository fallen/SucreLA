#include <stdbool.h>
#include <stdint.h>

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
};

uint32_t uartbone_read(struct uartbone_ctx *ctx, uint64_t addr);
void uartbone_unix_init(struct uartbone_ctx *ctx, char *file, unsigned int baudrate, unsigned int addr_width);
void uartbone_write(struct uartbone_ctx *ctx, uint64_t addr, uint32_t val);