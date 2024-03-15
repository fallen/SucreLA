#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
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

void print_usage(char *prog_name) {
	printf("usage: %s [-u uart_port] [-V] [-b baudrate] [-r addr|reg_name] [-w addr|reg_name -v value] [-i] [-a addr_width]\n", prog_name);
}

int main(int argc, char **argv) {
    struct uartbone_ctx ctx;
    char ident_str[256];
    char c;
    int i = 0;
    int opt;
    bool do_read = false;
    bool do_write = false;
    bool do_print_ident = false;
    bool verbose = false;
    unsigned int baudrate = 115200;
    unsigned int addr_width = 4;
    uint32_t addr;
    char *addr_string = NULL;
    uint32_t res, val;
    char *prog_name = argv[0];
    char *uart_port = NULL;
    char *csv_file = NULL;
    FILE *csv;
    bool use_usb = false;

    while ((opt = getopt(argc, argv, ":w:r:b:a:v:c:u:Vih")) != -1) {
        switch (opt) {
            case 'r':
                char *endptr;

                do_read = true;
                addr = strtol(optarg, &endptr, 0);
                if (errno != 0 || endptr == optarg)
                    addr_string = optarg;
                break;
            case 'w':

                do_write = true;
                addr = strtol(optarg, &endptr, 0);
                if (errno != 0 || endptr == optarg)
                    addr_string = optarg;
                break;
            case 'i':
                do_print_ident = true;
                break;
            case 'b':
                baudrate = strtol(optarg, &endptr, 0);
                if (errno != 0 || endptr == optarg)
                    goto err_print_usage;
                break;
            case 'a':
                addr_width = strtol(optarg, &endptr, 0);
                if (errno != 0 || endptr == optarg)
                    goto err_print_usage;
                break;
            case 'c':
                csv_file = optarg;
                break;
            case 'u':
                uart_port = optarg;
                break;
            case 'V':
                verbose = true;
                break;
            case 'v':
                val = strtol(optarg, &endptr, 0);
                if (errno != 0 || endptr == optarg)
                    goto err_print_usage;
                break;
            case 'h':
		print_usage(prog_name);
		return 0;
                break;
            case '?':
            default:
            err_print_usage:
		print_usage(prog_name);
                return -1;
        }
    }

    if (!uart_port) {
        printf("You must specify an uart or USB port using -u port\n");
        printf("Syntax: -u /dev/ttyUSBxxx\n");
        printf("Syntax: -u usb://vid:pid/EP\n");
        print_usage(prog_name);
        return -1;
    }

    uartbone_unix_init(&ctx, uart_port, baudrate, addr_width);

    if (!ctx.open) {
        printf("opening device %s failed!\n", uart_port);
        return ctx.error;
    }

    if (verbose) {
        if (do_print_ident)
            printf("printing ident string\n");
        if (do_read)
            printf("issuing Read\n");
        if (do_write)
            printf("issuing Write\n");
        printf("- uart: %s\n", uart_port);
        printf("- baudrate: %d\n", baudrate);
        printf("- address width: %d\n", addr_width);
        if (csv_file)
            printf("- csv: %s\n", csv_file);
        printf("\n");
    }

    // Let's open the CSV file
    if (do_print_ident || ((do_read || do_write) && addr_string)) {
        if (!csv_file) {
            printf("You must pass -c <csv_file> argument\n");
            return -1;
        }
        csv = fopen(csv_file, "r");
        if (!csv) {
            perror("fopen");
            return errno;
        }
    }

    if (do_print_ident) {
        int ret;
        uint32_t ident_addr;

        ret = get_reg_addr(csv, "identifier_mem", &ident_addr);
        if (ret) {
            printf("Could not find 'identifier_mem' in CSV file\n");
            return -1;
        }
        memset(ident_str, '\0', sizeof(ident_str));
        do {
            c = uartbone_read(&ctx, ident_addr+i*4);
            ident_str[i++] = c;
        } while (c);
        printf("ident: %s\n", ident_str);
    } else if (do_read) {
        int ret;

        if (addr_string) {
            ret = get_reg_addr(csv, addr_string, &addr);
            if (ret) {
                printf("Could not find register '%s'\n", addr_string);
                return -1;
            }
        }
        val = uartbone_read(&ctx, addr);
        printf("Read 0x%08x @ 0x%08x\n", val, addr);
    } else if (do_write) {
        int ret;

        if (addr_string) {
            ret = get_reg_addr(csv, addr_string, &addr);
            if (ret) {
                printf("Could not find register '%s'\n", addr_string);
                return -1;
            }
        }
        uartbone_write(&ctx, addr, val);
        printf("Written 0x%08x @ 0x%08x\n", val, addr);
    }

    return 0;
}
