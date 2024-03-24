CFLAGS=-fPIC

all: uartbone_cli libuartbone.so

uartbone_cli: LDLIBS=$(shell pkg-config --libs libusb-1.0)
uartbone_cli: linux_cli.o uartbone.o uartbone_linux.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

linux_cli.o: linux_cli.c

uartbone.o: uartbone.c

uartbone_linux.o: CFLAGS+=$(shell pkg-config --cflags libusb-1.0)
uartbone_linux.o: uartbone_linux.c

libuartbone.so: LDLIBS=$(shell pkg-config --libs libusb-1.0)
libuartbone.so: LDFLAGS=-shared -fPIC
libuartbone.so: uartbone.o uartbone_linux.o
	$(CC) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean:
	rm -f linux_cli.o uartbone.o uartbone_linux.o uartbone_cli libuartbone.so

.PHONY: clean all
