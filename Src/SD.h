#ifndef _SD_H_
#define _SD_H_

#include "main.h"
#include "ff.h"

typedef struct {
  FIL f;
  uint8_t filename[50];
  uint8_t flags; //read / write permissions etc.
  uint8_t error; //0  == no unhandled error has occured, 1, error hasnt been handled
} SD_FIL;

extern SPI_HandleTypeDef hspi1;
extern char SDPath[4];

#define SD_CS       GPIO_PIN_6
#define SD_CS_GPIO  GPIOA

#define SD_PWR       GPIO_PIN_7
#define SD_PWR_GPIO  GPIOA
#define SD_PWR_ON    GPIO_PIN_SET
#define SD_PWR_OFF    GPIO_PIN_RESET
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO, SD_CS, GPIO_PIN_SET);
#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO, SD_CS, GPIO_PIN_RESET);


// removed SD_openSafe etc. because they are not safe with more than one open file
//    which is a likely scenario in which these functions may be called

//int SD_openSafe (SD_FIL* fp, const TCHAR* path, BYTE mode, uint32_t timeout);				/* Open or create a file */
//int SD_closeSafe (SD_FIL* fp, uint32_t timeout);											/* Close an open file object */
//int SD_readSafe (SD_FIL* fp, void* buff, UINT btr, UINT* br, uint32_t timeout);			/* Read data from a file */
//int SD_writeSafe (SD_FIL* fp, const void* buff, UINT btw, UINT* bw, uint32_t timeout);	
//int SD_lseekSafe (SD_FIL* fp, DWORD ofs, uint32_t timeout);	

void SD_test_write();

int SD_link();
int SD_mount();
int SD_unmount();
void SD_powerCycle();
void SD_test();
uint32_t SD_toughTest();
#endif //_SD_H_