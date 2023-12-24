uartbone_cli: linux_cli.o uartbone.o
	$(CC) $(LDFLAGS) -o $@ $^

linux_cli.o: linux_cli.c

uartbone.o: uartbone.c

clean:
	rm -f linux_cli.o uartbone.o uartbone_cli
