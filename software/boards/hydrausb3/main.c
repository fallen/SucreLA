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

#include "config.h"
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

__attribute__((aligned(16))) volatile uint8_t HSPI_RX_Addr0[HSPI_RX_DMA_LENGTH] __attribute__((section(".DMADATA")));
__attribute__((aligned(16))) volatile uint8_t HSPI_RX_Addr1[HSPI_RX_DMA_LENGTH] __attribute__((section(".DMADATA")));

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
volatile unsigned int hspi_rx_packets = 0;
volatile unsigned int hspi_first_error_packet = -1;

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

#undef DEBUG

int main(void) {
	struct uartbone_ctx ctx;
	int i = 0;
	char c;
	volatile uint8_t *hspi_rxed_bytes = (volatile uint8_t *)HSPI_RX_Addr0;
	bool print_done = false;

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

	printf("Preparing HSPI RX interface...\n\r");
	memset(HSPI_RX_Addr0, '\0', HSPI_RX_DMA_LENGTH);
	memset(HSPI_RX_Addr1, '\0', HSPI_RX_DMA_LENGTH);
	HSPI_RX_End_Flag = 0;  // Receive completion flag
	HSPI_RX_End_Err = 0; // 0=No Error else >0 Error code
	HSPI_DoubleDMA_Init(HSPI_DEVICE, HSPI_DATA_WIDTH, (unsigned long int)HSPI_RX_Addr0, (unsigned long int)HSPI_RX_Addr1, 0);
    R8_HSPI_RX_SC = 0;

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
					USB30_IN_set(ENDP_1, ENABLE, ACK, 1, usb_cmd_len); // Set the endpoint to be able to send 1 packet
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

        if (HSPI_RX_End_Flag) {
            if (!(hspi_rx_packets % 100))
              printf("HSPI received %d packets!\n\r", hspi_rx_packets);
            HSPI_RX_End_Flag = 0;
        }

        if (HSPI_RX_End_Err && (!print_done)) {
            int start = 0;
            bool error_printed = false;
            uint8_t prev;
            printf("HSPI received %d packets!\n\r", hspi_rx_packets);
            if (HSPI_RX_End_Err) {
                if (HSPI_RX_End_Err & RB_HSPI_CRC_ERR)
                    printf("HSPI CRC error\n\r");
                printf("first error packet was num #%d\n\r", hspi_first_error_packet);
            }
            bsp_wait_us_delay(1);
            printf("Packet 0: \n\r");
            while (hspi_rxed_bytes[start] == 0) {
                printf("%02x ", hspi_rxed_bytes[start]);
                start++;
            }
            prev = hspi_rxed_bytes[start] - 1;
            for (i = start; i < 4096 - start; i++) {
                if (!(i % 50))
                    printf("\n\r");
                printf("%02x ", hspi_rxed_bytes[i]);
                prev = hspi_rxed_bytes[i];
	        }
	        printf("\n\r");
            printf("RX0 LEN = %08x RX1 LEN = %08x\n\r", R16_HSPI_RX_LEN0, R16_HSPI_RX_LEN1);
	        printf("UDF0: %08x UDF1: %08x\n\r", R32_HSPI_UDF0, R32_HSPI_UDF1);
	        printf("RX NUM seq %02x\n\r", R8_HSPI_RX_SC);
            printf("Packet 1: \n\r");
            hspi_rxed_bytes = HSPI_RX_Addr1;
            start = 0;
            while (hspi_rxed_bytes[start] == 0) {
                printf("%02x ", hspi_rxed_bytes[start]);
                start++;
            }
            prev = hspi_rxed_bytes[start] - 1;
            for (i = start; i < 4096 - start; i++) {
                if (!(i % 50))
                    printf("\n\r");
                printf("%02x ", hspi_rxed_bytes[i]);
                prev = hspi_rxed_bytes[i];
	        }
            printf("\n\r");
            printf("RX0 LEN = %08x RX1 LEN = %08x\n\r", R16_HSPI_RX_LEN0, R16_HSPI_RX_LEN1);
	        printf("UDF0: %08x UDF1: %08x\n\r", R32_HSPI_UDF0, R32_HSPI_UDF1);
	        printf("RX NUM seq %02x\n\r", R8_HSPI_RX_SC);
            HSPI_RX_End_Err = 0;
            HSPI_RX_End_Flag = 0;
            print_done = true;
        }
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
	    bool error = false;
		R8_HSPI_INT_FLAG = RB_HSPI_IF_R_DONE;  // Clear Interrupt
        hspi_rx_packets++;
        if (R8_HSPI_RX_SC & 0xf)
                R32_HSPI_RX_ADDR0 = (unsigned long int)HSPI_RX_Addr0;
        else
                R32_HSPI_RX_ADDR1 = (unsigned long int)HSPI_RX_Addr1;

		HSPI_RX_End_Flag = 1;
        if(R8_HSPI_RTX_STATUS & RB_HSPI_CRC_ERR) {
            HSPI_RX_End_Err |= RB_HSPI_CRC_ERR;
            error = true;
        }
        if(R8_HSPI_INT_FLAG & RB_HSPI_IF_FIFO_OV) {
            error = true;
            printf("HSPI FIFO overflow!\n\r");
            R8_HSPI_INT_FLAG = RB_HSPI_IF_FIFO_OV;
        }
        if (R8_HSPI_RTX_STATUS & RB_HSPI_NUM_MIS) {
            error = true;
            printf("HSPI seq num mismatch\n\r");
            HSPI_RX_End_Err |= RB_HSPI_NUM_MIS;
        }
        if (error) {
            printf("ERROR!!!\n\r");
            if (hspi_first_error_packet == -1)
                hspi_first_error_packet = hspi_rx_packets;
            return;
        }
        USB30_IN_set(ENDP_2, ENABLE, ACK, DEF_ENDP2_IN_BURST_LEVEL, 1024); // Able to send 4x 1024-bytes packets
        USB30_send_ERDY(ENDP_2 | IN, DEF_ENDP2_IN_BURST_LEVEL); // Tell Host we can accept EP2 IN packets
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