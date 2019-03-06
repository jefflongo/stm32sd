#ifndef SD_DRIVER_H_
#define SD_DRIVER_H

#include "stm32f4xx_hal.h"
#include "diskio.h"

#define SD_SPI_PORT   GPIOA
#define SD_SPI_SS_PIN GPIO_PIN_4

typedef uint8_t u8;
typedef uint32_t u32;

// SD commands are in the form 0-1-(6 bit command index)
#define CMD0     (0x40+0)     /* GO_IDLE_STATE */
#define CMD1     (0x40+1)     /* SEND_OP_COND */
#define CMD8     (0x40+8)     /* SEND_IF_COND */
#define CMD9     (0x40+9)     /* SEND_CSD */
#define CMD10    (0x40+10)    /* SEND_CID */
#define CMD12    (0x40+12)    /* STOP_TRANSMISSION */
#define CMD16    (0x40+16)    /* SET_BLOCKLEN */
#define CMD17    (0x40+17)    /* READ_SINGLE_BLOCK */
#define CMD18    (0x40+18)    /* READ_MULTIPLE_BLOCK */
#define CMD23    (0x40+23)    /* SET_BLOCK_COUNT */
#define CMD24    (0x40+24)    /* WRITE_BLOCK */
#define CMD25    (0x40+25)    /* WRITE_MULTIPLE_BLOCK */
#define CMD41    (0x40+41)    /* SEND_OP_COND (ACMD) */
#define CMD55    (0x40+55)    /* APP_CMD */
#define CMD58    (0x40+58)    /* READ_OCR */

DSTATUS SD_initialize (BYTE pdrv);
DSTATUS SD_status (BYTE pdrv);
DRESULT SD_read (BYTE pdrv, BYTE* buff, DWORD sector, UINT count);
DRESULT SD_write (BYTE pdrv, const BYTE* buff, DWORD sector, UINT count);
DRESULT SD_ioctl (BYTE pdrv, BYTE cmd, void* buff);

#endif
