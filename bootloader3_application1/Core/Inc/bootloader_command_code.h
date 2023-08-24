#ifndef INC_BOOTLOADER_COMMAND_CODE_H_
#define INC_BOOTLOADER_COMMAND_CODE_H_

#include "main.h"

#define BL_GET_VER							0x51
#define BL_GET_HELP							0x52
#define BL_GO_TO_ADDR						0x55
#define BL_FLASH_ERASE						0x56
#define BL_MEM_WRITE						0x57

uint8_t supported_commands[] =
{
	BL_GET_VER,
	BL_GET_HELP,
	BL_GO_TO_ADDR,
	BL_FLASH_ERASE,
	BL_MEM_WRITE,
};


#endif /* INC_BOOTLOADER_COMMAND_CODE_H_ */
