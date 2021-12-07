/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "SD.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		1	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		0	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	switch (pdrv) {
	case DEV_RAM :
		return RES_OK;

	case DEV_MMC :
		return RES_OK;

	case DEV_USB :
		return RES_OK;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	SD_Error ret;

	switch (pdrv) {
	case DEV_RAM :
		return STA_NOINIT;

	case DEV_MMC :
		ret = SD_ResetAllCards();
		if (ret != SD_OK) {
			return STA_NOINIT;
		}
		
		return RES_OK;

	case DEV_USB :
		return STA_NOINIT;
	}
	
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	int retryCount = 10;
	SD_Error ret;

	switch (pdrv) {
	case DEV_RAM :
		return RES_ERROR;

	case DEV_MMC :
		do {
			ret = SD_ReadBlocks(buff, sector, count);
			if (ret == SD_OK)
				break;
			retryCount--;
		} while (retryCount);
		
		if (retryCount == 0)
			return RES_ERROR;

		return RES_OK;

	case DEV_USB :
		return RES_ERROR;
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	int retryCount = 10;
	SD_Error ret;

	switch (pdrv) {
	case DEV_RAM :
		return RES_ERROR;

	case DEV_MMC :
		do {
			ret = SD_WriteBlocks(buff, sector, count);
			if (ret == SD_OK)
				break;
			retryCount--;
		} while (retryCount);
		
		if (retryCount == 0)
			return RES_ERROR;

		return RES_OK;

	case DEV_USB :
		return RES_ERROR;
	}

	return RES_PARERR;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	SD_Error ret;
	static SD_CardInfo cardInfo;
	
	switch (pdrv) {
	case DEV_RAM :
		return RES_ERROR;

	case DEV_MMC :
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				return RES_OK; 

		    case GET_SECTOR_SIZE:
				*(DWORD*)buff = SD_BLOCK_SIZE; // ff中的扇区就是SD卡中的块
		        return RES_OK;
	 
		    case GET_BLOCK_SIZE:	// ff中扇区可以有多个块
				*(WORD*)buff = 4;
		        return RES_OK;
			
		    case GET_SECTOR_COUNT:
				ret = SD_GetCardInfo(&cardInfo);
				if (ret != SD_OK)
					return RES_ERROR;
				
		        *(DWORD*)buff = cardInfo.BlocksNum;
		        return RES_OK;
				
		    default:
		        return RES_PARERR;
	    }

	case DEV_USB :
		return RES_ERROR;
	}

	return RES_PARERR;
}

