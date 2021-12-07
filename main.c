
#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "Serial.h"
#include "SD.h"
#include "LED.h"
#include "Delay.h"
#include "ff.h"


#define BUF_SIZE	2048

static FATFS g_fs;
static FILINFO g_fno;
static DIR g_dir;
static FIL g_fil;

static char g_buf[BUF_SIZE + 1];


__NO_RETURN int main()
{
    FRESULT fres;
	UINT br;
	
    SER_Init();
    LED_Init();
    SD_Init();

    while (1) {
        printf("Input any key to start test...\r\n");
        getchar();

        printf("Mount:\r\n");

        fres = f_mount(&g_fs, "0:", 1);
        if (fres != FR_OK) {
            printf("Failed to mount SD: %d\r\n", fres);
            continue;
        }

		printf("Files in the root directory:\r\n");
		
		fres = f_findfirst(&g_dir, &g_fno, "/", "*.*");
        if (fres != FR_OK) {
            printf("f_findfirst failed: %d\r\n", fres);
            continue;
        }
		
		while (fres == FR_OK && g_fno.fname[0]) {
			printf("%s\r\n", g_fno.fname);
			
			if (f_open(&g_fil, g_fno.fname, FA_READ) == FR_OK) {
				if (f_read(&g_fil, g_buf, BUF_SIZE, &br) == FR_OK) {
					g_buf[br] = 0;
					printf("%s\r\n", g_buf);
				}
				f_close(&g_fil);
			}
			
			fres = f_findnext(&g_dir, &g_fno);
		}
		
		f_closedir(&g_dir);
    }
}
