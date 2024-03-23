uartbone_cli: LDFLAGS=$(shell pkg-config --libs libusb-1.0)
uartbone_cli: linux_cli.o uartbone.o uartbone_linux.o
	$(CC) -o $@ $^ $(LDFLAGS)

linux_cli.o: linux_cli.c

uartbone.o: uartbone.c

uartbone_linux.o: CFLAGS=$(shell pkg-config --cflags libusb-1.0)
uartbone_linux.o: uartbone_linux.c

clean:
	rm -f linux_cli.o uartbone.o uartbone_linux.o uartbone_cli
