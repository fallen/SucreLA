#include <stdbool.h>
#include <stdint.h>

enum uart_backend_type {
    UNIX_UART,
    CH569_UART
};

struct uartbone_ctx;

struct uart_backend {
    enum uart_backend_type type;
    int (*readb)(struct uartbone_ctx *ctx, uint8_t *data);
    void (*writeb)(struct uartbone_ctx *ctx, uint8_t data);
    void (*flush)(void);
};

struct uartbone_ctx {
    unsigned int addr_width;
    unsigned int baudrate;
    int fd;
    struct uart_backend *uart;
    bool open;
    int error;
};

uint32_t uartbone_read(struct uartbone_ctx *ctx, uint64_t addr);
void uartbone_unix_init(struct uartbone_ctx *ctx, char *file, unsigned int baudrate, unsigned int addr_width);
void uartbone_write(struct uartbone_ctx *ctx, uint64_t addr, uint32_t val);