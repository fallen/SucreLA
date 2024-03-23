#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* System clock and HSPI freq : 120 MHz */
#define FREQ_SYS (120000000)

#include "core_riscv.h"
#include "CH56x_bsp.h"
#include "CH56x_uart.h"
#include "CH56x_common.h"

#include "uartbone.h"
#include "csr.h" // for FPGA SoC CSR register addresses

#include "CH56x_usb30_devbulk.h"
#include "CH56x_usb20_devbulk.h"
#include "CH56x_usb30_devbulk_LIB.h"
#include "CH56x_usb_devbulk_desc_cmd.h"

#include "hydrausb3_usb_devbulk_vid_pid.h"

#define RISCV_FENCE(p, s) \
	__asm__ __volatile__ ("fence " #p "," #s : : : "memory")

/* These barriers need to enforce ordering on both devices or memory. */
#define mb()		RISCV_FENCE(iorw,iorw)
#define rmb()		RISCV_FENCE(ir,ir)
#define wmb()		RISCV_FENCE(ow,ow)

/* UARTs settings */
#define UART1_BAUD 115200
#define UART3_BAUD 115200

/* HSPI Data width : 8 bits */
#define HSPI_DATA_WIDTH 0

#define HSPI_RX_DMA_LENGTH   4096

__attribute__((aligned(16))) uint8_t HSPI_RX_Addr0[HSPI_RX_DMA_LENGTH] __attribute__((section(".DMADATA")));
__attribute__((aligned(16))) uint8_t HSPI_RX_Addr1[HSPI_RX_DMA_LENGTH] __attribute__((section(".DMADATA")));

/* Shared variables */
volatile int HSPI_TX_End_Flag; // Send completion flag
volatile int HSPI_RX_End_Flag; // Receive completion flag
volatile int HSPI_RX_End_Err; // 0=No Error else >0 Error code

/* FLASH_ROMA Read Unique ID (8bytes/64bits) */
#define FLASH_ROMA_UID_ADDR (0x77fe4)
usb_descriptor_serial_number_t unique_id;

/* USB VID PID */
usb_descriptor_usb_vid_pid_t vid_pid =
{
	.vid = USB_VID,
	.pid = USB_PID
};

void uartbone_ch56x_init(struct uartbone_ctx *ctx, int uart_num, int baudrate, int addr_width);

volatile int usb_cmd_rxed = 0;
volatile uint8_t usb_cmd[11];
volatile uint8_t usb_cmd_len;
extern volatile int EP1_to_be_sent;
volatile int ep1_out_nack = 0;

static void USB_init(void) {
	R32_USB_CONTROL = 0;
	PFIC_EnableIRQ(USBSS_IRQn);
	PFIC_EnableIRQ(LINK_IRQn);
	PFIC_EnableIRQ(TMR0_IRQn);
	R8_TMR0_INTER_EN = RB_TMR_IE_CYC_END;
	TMR0_TimerInit(67000000); // USB3.0 connection failure timeout about 0.56 seconds

	/* USB Descriptor set String Serial Number with CH569 Unique ID */
	usb_descriptor_set_string_serial_number(&unique_id);

	/* USB Descriptor set USB VID/PID */
	usb_descriptor_set_usb_vid_pid(&vid_pid);

	/* USB3.0 initialization, make sure that the two USB3.0 interrupts are enabled before initialization */
	USB30D_init(ENABLE);
}

int main(void) {
	struct uartbone_ctx ctx;
	char ident_str[256];
	int i = 0;
	char c;
	uint32_t *dma_rx = (uint32_t *)HSPI_RX_Addr0;
	uint8_t *hspi_rxed_bytes = (uint8_t *)HSPI_RX_Addr0;

	bsp_gpio_init();
	bsp_init(FREQ_SYS);
	memset(&unique_id, 0, sizeof(unique_id));
	FLASH_ROMA_READ(FLASH_ROMA_UID_ADDR, (uint32_t*)&unique_id, sizeof(unique_id));
	UART1_init(UART1_BAUD, FREQ_SYS);
	UART3_init(UART3_BAUD, FREQ_SYS);

	printf("\n\r");
	printf("###########################\n\r");
	printf("# SucreLA fw starting up! #\n\r");
	printf("###########################\n\r\n\r");

	printf("board: HydraUSB3\n\r");
	printf("USB init...");
	USB_init();
	printf("done\n\r");
	uartbone_ch56x_init(&ctx, 3, 115200, 4);
	printf("uartbone: initialized on UART3\n\r");

	printf("Identifying FPGA SoC...\n\r");
	memset(ident_str, '\0', sizeof(ident_str));
	do {
		c = uartbone_read(&ctx, CSR_IDENTIFIER_MEM_BASE+i*4);
		ident_str[i++] = c;
	} while (c);
	printf("FPGA SoC ident: %s\n\r", ident_str);

	printf("Preparing HSPI RX interface...\n\r");
	memset(HSPI_RX_Addr0, '\0', HSPI_RX_DMA_LENGTH);
	memset(HSPI_RX_Addr1, '\0', HSPI_RX_DMA_LENGTH);
	HSPI_RX_End_Flag = 0;  // Receive completion flag
	HSPI_RX_End_Err = 0; // 0=No Error else >0 Error code
	HSPI_DoubleDMA_Init(HSPI_DEVICE, HSPI_DATA_WIDTH, (unsigned long int)HSPI_RX_Addr0, (unsigned long int)HSPI_RX_Addr1, 0);

	printf("Starting a capture...\n\r");
	uartbone_write(&ctx, CSR_LA_TRIGGER_MEM_MASK_ADDR, 0);
	uartbone_write(&ctx, CSR_LA_TRIGGER_MEM_VALUE_ADDR, 0);
	uartbone_write(&ctx, CSR_LA_TRIGGER_MEM_WRITE_ADDR, 1);
	uartbone_write(&ctx, CSR_LA_STORAGE_OFFSET_ADDR, 0);
	uartbone_write(&ctx, CSR_LA_STORAGE_LENGTH_ADDR, 512);
	uartbone_write(&ctx, CSR_LA_STORAGE_ENABLE_ADDR, 1);
	uartbone_write(&ctx, CSR_LA_TRIGGER_ENABLE_ADDR, 1);

	printf("Waiting for HSPI RX data...\n\r");
	while(HSPI_RX_End_Flag == 0)
		;
	printf("HSPI RX done!\n\r");

	for (i = 0; i < 4096; i++) {
		if (!(i % 50))
			printf("\n\r");
		printf("%02x ", hspi_rxed_bytes[i]);
	}
	printf("\n\r");

	if (!HSPI_RX_End_Err)
		printf("CRC OK!\n\r");

	while (1) { // infinite loop to handle uartbone over USB protocole
		if (usb_cmd_rxed) {
			mb();
#ifdef DEBUG
			printf("usb rx!\n\r");
#endif
			switch (usb_cmd[0]) {
				case 'w': {
#ifdef DEBUG
					printf("uart sending: \n\r");
					for (i = 2; i < usb_cmd_len+2; i++)
						printf("%02x ", usb_cmd[i]);
					printf("\n\r");
#endif
					UART3_SendString(&usb_cmd[2], usb_cmd_len);
					usb_cmd_len = 0;
					break;
				}
				case 'r': {
					UART3_Recv(usb_cmd, usb_cmd_len);
#ifdef DEBUG
					printf("uart rx'ed: \n\r");
					for (i = 0; i < usb_cmd_len; i++)
						printf("%02x ", usb_cmd[i]);
					printf("\n\r");
#endif
					memcpy(endp1Tbuff, usb_cmd, usb_cmd_len);
					mb();
					EP1_to_be_sent = usb_cmd_len;
					USB30_IN_set(ENDP_1, ENABLE, ACK, 1, EP1_to_be_sent); // Set the endpoint to be able to send 1 packet
					USB30_send_ERDY(ENDP_1 | IN, 1); // Notify the host we are ready to receive 1 IN packet
					usb_cmd_len = 0;
					break;
				}
				default: {
					printf("buggy usb cmd: \n\r");
					for (i = 0; i < usb_cmd_len + 1; i++)
						printf("%02x ", usb_cmd[i]);
					printf("\n\r");
				}
			}
			usb_cmd_rxed = 0; // command completed, let's prepare for the next one.
			USB30_OUT_set(ENDP_1, ACK, 0); // Set EP1 to ACK next OUT packet, we can now receive a new command
			USB30_send_ERDY(ENDP_1 | OUT, 1); // Notify the host we are ready to receive 1 OUT packet
		}
#ifdef DEBUG
		if (ep1_out_nack) {
			printf("EP1 out nack! This should not happen!\n\r");
			ep1_out_nack--;
		}
#endif
	}

	return 0;
}

void HSPI_IRQHandler_ReInitRX(void)
{
	R32_HSPI_RX_ADDR0 = (unsigned long int)HSPI_RX_Addr0;
	R32_HSPI_RX_ADDR1 = (unsigned long int)HSPI_RX_Addr1;
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void HSPI_IRQHandler(void)
{
	if (R8_HSPI_INT_FLAG & RB_HSPI_IF_R_DONE) // Single packet reception completed
	{
		R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  // Clear Interrupt
		HSPI_IRQHandler_ReInitRX();
		HSPI_RX_End_Flag = 1;
	}

	// Determine whether the CRC is correct
	if(R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR)
	{
		// CRC check err
		//printf("CRC err\n\r");
		HSPI_IRQHandler_ReInitRX();
		HSPI_RX_End_Err |= 1;
	}
}

__attribute__((interrupt("WCH-Interrupt-fast"))) void HardFault_Handler(void)
{
	printf("HardFault\n");
	printf(" SP=0x%08X\n", __get_SP());
	printf(" MIE=0x%08X\n", __get_MIE());
	printf(" MSTATUS=0x%08X\n", __get_MSTATUS());
	printf(" MCAUSE=0x%08X\n", __get_MCAUSE());
	bsp_wait_ms_delay(1);
}