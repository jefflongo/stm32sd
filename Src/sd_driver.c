#include "sd_driver.h"

extern SPI_HandleTypeDef hspi1;
extern volatile u8 timer;
static volatile DSTATUS stat;
static u8 type = 0;

/* SPI Functions */

static u8 SPI_ReadByte(void) {
	u8 dummy = 0xFF;
	u8 data = 0;
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_TransmitReceive(&hspi1, &dummy, &data, 1, 1000);
	return data;
}

static void SPI_WriteByte(u8 data) {
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
}

static void SD_Select(void) {
	HAL_GPIO_WritePin(SD_SPI_PORT, SD_SPI_SS_PIN, GPIO_PIN_RESET);
}

static void SD_Deselect(void) {
	HAL_GPIO_WritePin(SD_SPI_PORT, SD_SPI_SS_PIN, GPIO_PIN_SET);
}

static int SD_CheckReady(){
	u8 prevtime = timer;
	timer = 50;
	u8 r = SPI_ReadByte();
	while ((r != 0xFF) && timer) {
		r = SPI_ReadByte();
	}
	timer = prevtime;
	return r;
}

static u8 SD_SendCommand(u8 cmd, u32 arg) {
	u8 r;
	u8 crc = 0;
	// Wait for SD card to be ready
	if (SD_CheckReady() != 0xFF) return 0xFF;
	// Send command->argument->CRC
	SPI_WriteByte(cmd);
	SPI_WriteByte((u8)(arg >> 24));
	SPI_WriteByte((u8)(arg >> 16));
	SPI_WriteByte((u8)(arg >> 8));
	SPI_WriteByte((u8)(arg));
	if (cmd == CMD0) crc = 0x95;
	if (cmd == CMD8) crc = 0x87;
	SPI_WriteByte(crc);
	// Wait for a valid response
	for (int i = 0; i < 10; i++) {
		r = SPI_ReadByte();
		if ((r & 0x80) == 0) break;
	}
	return r;
}

static void SD_ReadBlock(BYTE* buff, UINT* count) {
	// Receive count number of 512 byte packets
	do {
		u8 token;
		// Ensure the next response is the valid 0xFE token
		for (int i = 0; i < 128; i++) {
			token = SPI_ReadByte();
			if (token != 0xFF) break;
		}
		if (token != 0xFE) return;
		// Receive the packet
		for (int i = 0; i < 512; i++) {
			*buff = SPI_ReadByte();
			buff++;
		}
		// Throw away CRC
		SPI_ReadByte();
		SPI_ReadByte();
	} while (count-- > 0);
}

/* Driver functions */
DSTATUS SD_initialize (BYTE pdrv) {
	u8 r, ocr[4];
	// Reduce SCLK to ~400kHz
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	HAL_SPI_Init(&hspi1);
	// Single drive hardware should only have drive 0
	if (pdrv != 0) return STA_NOINIT;
	// Ensure there is a drive plugged in
	if(stat & STA_NODISK) return stat;

	/* Power on sequence */
	// Delay 1ms, Set DI and SS high, apply 74 or more clock pulses to SCLK
	HAL_Delay(1);
	SD_Deselect();
	for (int i = 0; i < 10; i++) SPI_WriteByte(0xFF);
	// Go_Idle_State command
	SD_Select();
	r = SD_SendCommand(CMD0, 0);
	// Add extra waiting period on startup
	for (int i = 0; i < 0x1FFF; i++) {
		if (r == 0x01) {
			printf("CMD0 success\n");
			break;
		}
		r = SPI_ReadByte();
	}
	SD_Deselect();
	if (r != 0x01) return STA_NOINIT;
	// Set DI back to high
	SPI_WriteByte(0xFF);

	/* Card type detection */
	timer = 100;
	SD_Select();
	r = SD_SendCommand(CMD8, 0x1AA);
	// CMD8 accepted
	if (r == 0x01) {
		printf("CMD8 success\n");
		// Read 4 byte data
		for (int i = 0; i < 4; i++) ocr[i] = SPI_ReadByte();
		// Arg matched
		if ((ocr[2] << 2) | (ocr[3] == 0x1AA)) {
			printf("0x1AA matched\n");
			SD_Deselect();
			SPI_WriteByte(0xFF);
			SD_Select();
			// Attempt to initialize until success
			while (timer) {
				if (SD_SendCommand(CMD55, 0) <= 0x01 && SD_SendCommand(CMD41, 1UL << 30) == 0) {
					printf("CMD41/51 success\n");
					SD_Deselect();
					SPI_WriteByte(0xFF);
					break;
				}
				SD_Deselect();
				SPI_WriteByte(0xFF);
				SD_Select();
			}
			// Read 4 byte data from OCR
			SD_Select();
			if (SD_SendCommand(CMD58, 0) == 0) {
				printf("CMD58 success\n");
				for (int i = 0; i < 4; i++) ocr[i] = SPI_ReadByte();
				// Check CCS bit
				type = (ocr[0] & 0x40) ? 6 : 2;
			}
		}
	}
	// CMD8 rejected
	else {
		type = (SD_SendCommand(CMD55, 0) <= 1 && SD_SendCommand(CMD41, 0) <= 1) ? 2 : 1;
		// Attempt to initialize until success
		while (timer) {
			r = SD_SendCommand(CMD55, 0);
			// If CMD55 succeeds send CMD41, otherwise CMD1
			if (r <= 0x01 && SD_SendCommand(CMD41, 0) == 0) break;
			else if (r == 0x05 && SD_SendCommand(CMD1, 0) == 0) break;
		}
		// Set block length to 512 bytes
	    if (SD_SendCommand(CMD16, 512) != 0) type = 0;
	}
	SD_Deselect();
	SPI_ReadByte();
	if (type != 0) stat &= ~STA_NOINIT;
	// Increase SCLK back to ~25MHz
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi1);
	return stat;
}

DSTATUS SD_status (BYTE pdrv) {
	return (pdrv == 0) ? stat : STA_NOINIT;
}

DRESULT SD_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
	// Invalid parameters
	if (pdrv != 0 || count == 0) return RES_PARERR;
	// Not initialized
	if (stat & STA_NOINIT) return RES_NOTRDY;

	SD_Select();
	// Read single block
	if (count == 1) {
		// Send receive command
		if (SD_SendCommand(CMD17, sector) == 0) {
			SD_ReadBlock(buff, &count);
		}
	}
	// Read multi block
	else {
		if (SD_SendCommand(CMD18, sector) == 0) {
			SD_ReadBlock(buff, &count);
			SD_SendCommand(CMD12, 0);
		}
	}
	SD_Deselect();
	SPI_ReadByte();
	return (count == 0) ? RES_OK : RES_ERROR;
}

DRESULT SD_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
	// TODO: Implement
	return RES_OK;
}

DRESULT SD_ioctl (BYTE pdrv, BYTE cmd, void* buff) {
	// TODO: Implement
	return RES_OK;
}
