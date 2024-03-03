uartbone_cli: linux_cli.o uartbone.o uartbone_linux.o
	$(CC) $(LDFLAGS) -o $@ $^

linux_cli.o: linux_cli.c

uartbone.o: uartbone.c

uartbone_linux.o: uartbone_linux.c

clean:
	rm -f linux_cli.o uartbone.o uartbone_linux.o uartbone_cli
