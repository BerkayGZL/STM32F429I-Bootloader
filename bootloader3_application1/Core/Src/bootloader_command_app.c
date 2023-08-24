#include "bootloader_command_app.h"

extern uint8_t supported_commands[];

void bootloader_get_ver_cmd(uint8_t *bl_rx_data)
{
	uint8_t bl_Version = 0; // versiyon bilgisini tutmak için

	printMessage("Bootloader: Bootloaer_Get_Ver_Cmd\n");

	uint32_t command_packet_length = bl_rx_data[0] + 1; // paket uzunlugu bulundu.(length to follow +1)

	// Toplam paket uzunluğundan 4 çıkarılarak CRC başlangıc add bulundu
	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_length - 4));

	// crc control yapıldı. crc=0 doğru, crc=1 yanlış,
	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_length - 4, host_crc))
	{
		printMessage("Bootloader : Checksum success\n");
		bootloader_send_ack(1);
		bl_Version = bootloader_get_version();
		printMessage("Bootloader : BL_VER : %d %#x  \n", bl_Version, bl_Version); // Serial ekrana yazdırıldı.
		bootloader_uart_write_data(&bl_Version, 1);
	}
	else
	{
		printMessage("Bootloader : Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_get_help_cmd (uint8_t *bl_rx_data)
{

	printMessage("Bootloader: Bootloaer_Get_Help_Cmd\n");

	uint32_t command_packet_length = bl_rx_data[0] + 1; // paket uzunlugu bulundu.(length to follow +1)

	// Toplam paket uzunluğundan 4 çıkarılarak CRC başlangıc add bulundu
	uint32_t host_crc = *((uint32_t*) (bl_rx_data + command_packet_length -4));

	// crc control yapıldı. crc=0 doğru, crc=1 yanlış,
	if (!bootloader_verify_crc(&bl_rx_data[0], command_packet_length-4, host_crc))
	{
		printMessage("Bootloader: Checksum succes\n");

		// ack değerini gönder ve 2. olarak da gelecek verinin uzunluğu gönderildi.
		bootloader_send_ack(strlen(supported_commands));

		// supported command ve size kadar gönder.
		bootloader_uart_write_data(supported_commands, strlen(supported_commands));
	}
	else
	{
		printMessage("Bootloader: Checksum fail\n");
		bootloader_send_nack();
	}
}

void bootloader_go_to_addr_cmd(uint8_t *bl_rx_data)
{
	uint32_t go_to_address = 0;
	uint8_t addr_valid = ADDR_VALID; 		// address geçerli
	uint8_t addr_invalid = ADDR_INVALID; 	// address geçersiz

	printMessage("Bootloader : bootlodaer_go_to_addr_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1; // paket uzunlugu bulundu.(length to follow +1)

	// Toplam paket uzunluğundan 4 çıkarılarak CRC başlangıc add. bulundu
	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_len - 4));

	// crc control yapıldı. crc=0 doğru, crc=1 yanlış,
	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("Bootloader : Checksum succes \n");
		bootloader_send_ack(1);

		// gelen memory address
		go_to_address = *((uint32_t*)&bl_rx_data[2]);
		printMessage("Bootloader : GO Addr: %#x \n", go_to_address);

		//gelen address doğruysa yazmaya başla
		if(bootloader_verify_address(go_to_address) == ADDR_VALID)
		{
			bootloader_uart_write_data(&addr_valid, 1);

			go_to_address += 1; 		// T Bit = 1, 0x08008000 sondaki 0 => 1 yapıldı.

			//atlama fonksiyonu
			void (*lets_go_to_address)(void) = (void*) go_to_address;

			printMessage("Bootloader : Going to Address \n");

			lets_go_to_address();
		}
		else
		{
			printMessage("Bootloader : Go Address Invalid \n");
			bootloader_uart_write_data(&addr_invalid, 1);
		}
	}
	else
	{
		printMessage("Bootloader : Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_flash_erase_cmd(uint8_t *bl_rx_data)
{
	uint8_t eraseStatus = 0;

	printMessage("Bootloader : bootloader_flash_erase_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1; // paket uzunlugu bulundu.(length to follow +1)

	// Toplam paket uzunluğundan 4 çıkarılarak CRC başlangıc add. bulundu
	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_len - 4));

	// crc control yapıldı. crc=0 doğru, crc=1 yanlış,
	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("Bootloader : Checksum success \n");
		bootloader_send_ack(1);
		printMessage("Bootloader : Initial Sector: %d Nubmer Of Secotrs: %d \n", bl_rx_data[2], bl_rx_data[3]);

		// execute_flash_erase de silinmesi gereken sektör ve sektör numarası verilir.
		eraseStatus = execute_flash_erase(bl_rx_data[2], bl_rx_data[3]);

		printMessage("Bootloader : Flash Erase Status : %d \n", eraseStatus);
		bootloader_uart_write_data(&eraseStatus, 1);
	}
	else
	{
		printMessage("Bootloader : Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_mem_write_cmd(uint8_t* bl_rx_data)
{
	uint8_t addrValid = ADDR_VALID; // doğru adrese yazılıyor mu?
	uint8_t writeStatus = 0x00; // Yazma durumu
	uint8_t checkSum = 0;
	uint8_t length = 0;

	length = bl_rx_data[0]; // lengt to follow değeri atandı.

	uint8_t payloadLength = bl_rx_data[6]; // 6. index yük uzunlugu olacak

	uint32_t memAddress = *((uint32_t *) (&bl_rx_data[2])); // gelen datanın 2. adresten başlayacak.

	checkSum = bl_rx_data[length]; // gelen data uzunlugu kadar olacak

	printMessage("Bootloader : bootloader_mem_write_cmd \n");

	uint32_t command_packet_len = bl_rx_data[0] + 1; // paket uzunlugu bulundu.(length to follow +1)

	// Toplam paket uzunluğundan 4 çıkarılarak CRC başlangıc add. bulundu
	uint32_t host_crc = *((uint32_t*)(bl_rx_data + command_packet_len - 4));

	// crc control yapıldı. crc=0 doğru, crc=1 yanlış,
	if(!bootloader_verify_crc(&bl_rx_data[0], command_packet_len - 4, host_crc))
	{
		printMessage("Bootloader : Checksum success \n");
		bootloader_send_ack(1);

		printMessage("Bootloader : Memory Write Address: %#x \n", memAddress);

		// Memory address doğrulaması
		if(bootloader_verify_address(memAddress) == ADDR_VALID)
		{
			printMessage("Bootloader : Valid Memory Write Address \n");

			writeStatus = execute_memory_write(&bl_rx_data[7], memAddress, payloadLength);

			bootloader_uart_write_data(&writeStatus, 1);
		}
		else
		{
			printMessage("Bootloader : Invalid Memory Write Address \n");
			writeStatus = ADDR_INVALID;
			bootloader_uart_write_data(&writeStatus, 1);
		}
	}
	else
	{
		printMessage("Bootloader : Checksum fail \n");
		bootloader_send_nack();
	}
}

void bootloader_uart_write_data(uint8_t *Buffer, uint32_t len)
{
	HAL_UART_Transmit(&huart3, Buffer, len, HAL_MAX_DELAY);
}

//CRC_SUCCESS = 0, CRC_FAIL=1
uint8_t bootloader_verify_crc(uint8_t *Buffer, uint32_t len, uint32_t crcHost)
{
	uint32_t crcValue = 0xFF; 	//crc degerleri atanması için
	uint32_t data = 0;			//Buffer daki elemanları içine alınacak

	for(uint32_t i = 0; i < len; i++)
	{
			data = Buffer[i];

			// data da bulunan degerler crcValue ye tek tek eklenir
			crcValue = HAL_CRC_Accumulate(&hcrc, &data, 1);
	}

	__HAL_CRC_DR_RESET(&hcrc);

	if(crcValue == crcHost) // crcHost gelen crc ile benim doldurduğum crc karşılaştırıldı.
	{
		return CRC_SUCCESS;
	}

	return CRC_FAIL;
}


//crc bilgisi doğru ise ack degerini pc ye gönder. ack bilgisi 2 byte gönderilir.
void bootloader_send_ack(uint8_t followLength)
{
	uint8_t ackBuffer[2];
	ackBuffer[0] = BL_ACK_VALUE; //  0xA5 tanımlanıp gönderildi. Başarılı.
	ackBuffer[1] = followLength;

	HAL_UART_Transmit(&huart3, ackBuffer, 2, HAL_MAX_DELAY);
}

//crc bilgisi yanlış ise nack gönder. nack bilgisi 1 byte gönderilir.
void bootloader_send_nack()
{
	uint8_t nackValue = BL_NACK_VALUE; //0x7F tanımlanıp gönderildi. Başarısız.
	HAL_UART_Transmit(&huart3, &nackValue, 1, HAL_MAX_DELAY);
}

uint8_t bootloader_get_version(void)
{
	return BL_VER;
}


uint8_t bootloader_verify_address(uint32_t goAddress)
{
	// sector 0 ile 11 arasında kontrolü yapıldı.
	if(goAddress >= FLASH_BASE && goAddress <= FLASH_END)
		return ADDR_VALID;
	else if(goAddress >= SRAM1_BASE && goAddress <= SRAM1_END)
		return ADDR_VALID;
	else if(goAddress >= SRAM2_BASE && goAddress <= SRAM2_END)
		return ADDR_VALID;
	else if(goAddress >= BKPSRAM_BASE && goAddress <= BKPSRAM_END)
		return ADDR_VALID;
	else
		return ADDR_INVALID;
}

// execute_flash_erase de silinmesi gereken sektör ve sektör numarası verilir.
uint8_t execute_flash_erase(uint8_t sectorNumber, uint8_t numberOfSector)
{
	FLASH_EraseInitTypeDef FlashEraseInitStruct = {0}; // içerisinde belli değerler vardır. ilk 0 verildi.
	uint32_t SectorError = 0;
	HAL_StatusTypeDef status = {0}; // status içinde ok-error-busy-timeout vardır. ilk 0 verdik.

	// toplam 11 sector old için fazlası kabul edilmez
	if(numberOfSector > 11)
		return INVALID_SECTOR; //0x04 olarak dönülür.

	if((sectorNumber <= 11) || (sectorNumber == 0xFF))
	{
		if(sectorNumber == 0xFF) //0xFF girilirse komple sil
		{
			FlashEraseInitStruct.TypeErase = FLASH_TYPEERASE_MASSERASE;
		}
		else
		{
			// ornegin sector number 8 ise 11-8 = 3, 3 adet sektor silinmeli
			uint8_t remainingSector = 11 - sectorNumber;

			if(sectorNumber > remainingSector)
				sectorNumber = remainingSector; // kac adet sektör silinmesi gerekiyor o hesaplandı.

		FlashEraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; // sector sector silme
		FlashEraseInitStruct.Sector = sectorNumber; //sectorNumber sectorleri silinecek
		FlashEraseInitStruct.NbSectors = numberOfSector;// number of sector kadar silinecek
		}
		FlashEraseInitStruct.Banks = FLASH_BANK_1;//bank seçimi yapıldı.

		HAL_FLASH_Unlock(); // flash kilidi açıldı
		FlashEraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3; // voltaj aralığı 2.6-3.7 alındı.
		status = (uint8_t) HAL_FLASHEx_Erase(&FlashEraseInitStruct, &SectorError);
		HAL_FLASH_Lock(); // flash kilitlendi.

		return status;
	}

	return INVALID_SECTOR;
}

// buffer içeriğini hafızaya yazar
uint8_t execute_memory_write(uint8_t *Buffer, uint32_t memAddress, uint32_t len) // 255 byte taşıyabilir.
{
	uint8_t status = HAL_OK;

	HAL_FLASH_Unlock();		// flash kilidi ac


	//buffer da bulunan değerleri uzunluk kadar yazmak için kullanıldı.
	for(uint32_t i = 0 ; i <len ; i++)
	{
		//Gelen veri byte byte status yazıldı. 8 bit olarak yazıldı. memAddress+i 0x08008000+1
		// HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, memAddress+i, Buffer[i]) memory yazma tamamlandı
		status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, memAddress+i, Buffer[i]);
	}

	HAL_FLASH_Lock(); // flash kilitle

	return status;
}

