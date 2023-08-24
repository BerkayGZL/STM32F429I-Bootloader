#ifndef INC_BOOTLOADER_COMMAND_APP_H_
#define INC_BOOTLOADER_COMMAND_APP_H_

#include "main.h"
#include "stdio.h"
#include "string.h"

#define BL_VER	0x10		//	1.0

#define CRC_FAIL		1
#define CRC_SUCCESS		0

#define BL_ACK_VALUE    0xA5
#define BL_NACK_VALUE	0x7F

#define ADDR_VALID			0x00
#define ADDR_INVALID		0x01

#define SRAM1_SIZE			112*1024 	// 112 kBayte
#define SRAM1_END			(SRAM1_BASE + SRAM1_SIZE)
#define SRAM2_SIZE			16*1024 	// 16 kBayte
#define SRAM2_END			(SRAM2_BASE + SRAM2_SIZE)
#define BKPSRAM_SIZE		4*1024 		// 4 kBayte
#define BKPSRAM_END			(BKPSRAM_BASE + BKPSRAM_SIZE)

#define INVALID_SECTOR 	0x04

void bootloader_get_ver_cmd(uint8_t *bl_rx_data);
void bootloader_get_help_cmd(uint8_t *bl_rx_data);
void bootloader_go_to_addr_cmd(uint8_t *bl_rx_data);
void bootloader_flash_erase_cmd(uint8_t *bl_rx_data);
void bootloader_mem_write_cmd(uint8_t *pBuffer);


/*CRC Verify Function*/

uint8_t bootloader_verify_crc(uint8_t *Buffer, uint32_t len, uint32_t crcHost);

void bootloader_send_ack(uint8_t followLength);
void bootloader_send_nack();

uint8_t bootloader_get_version(void);

void bootloader_uart_write_data(uint8_t *Buffer, uint32_t len);

uint8_t bootloader_verify_address(uint32_t goAddress);

uint8_t execute_flash_erase(uint8_t sectorNumber, uint8_t numberOfSector);

uint8_t execute_memory_write(uint8_t *Buffer, uint32_t memAddress, uint32_t len);

#endif /* INC_BOOTLOADER_COMMAND_APP_H_ */
