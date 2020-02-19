//Functions to use required by SD library

#include "main.h"
#include "stm32_adafruit_sd.h"
#include "SD.h"
#include "sd_diskio.h"
#include "SPI.h"

#define SD_DUMMY_BYTE 0xff

uint32_t writeTime = 0;

SPIConfig_t SDSpiConfig = {SPI_BAUDRATEPRESCALER_2, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE};

FATFS SDFatFs;  /* File system object for SD disk logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD disk logical drive path */ 

/*****************************************
        SD FATFS safe file ops

   These are function wrappers of the fatfs file op functions. once file has been opened with SD_openSafe the other fatfs
   functions can be used with SD_FIL.f

   Each function will reset the SD and reopen the file if an SD error occurs. f_sync is called as the end of each function
    
******************************************/

int SD_openSafe (SD_FIL* fp, const TCHAR* path, BYTE mode, uint32_t timeout) {			/* Open or create a file */
//  strcpy(fp->filename, path);
//  fp->flags = mode;
//  fp->error = 0;
  int er = -1;
//  uint32_t startTime = HAL_GetTick();
//  
//  uint8_t success = 0;
//
//  do {
//    if(fp->error == 0) {
      er = f_open(&fp->f, path, mode);
//    }
//    else {
//      er = -1;
//    }
//    
//    if(( er != FR_OK) && (
//          ( er != FR_NO_FILE ) ||
//          ( er != FR_NO_PATH )
//          ) ) {
//      #ifdef DEBUG_PRINT
//      debug_printf("f_open failed: %i\n", er);
//      #endif
//      #ifdef UART_DEBUG
//      DEBUG_print(&debugInstance, "f_open failed: %i\r\n", er);
//      #endif
//      if(er == FR_NO_FILE ||
//                er == FR_NO_PATH ||
//                er == FR_INVALID_NAME ||
//                er == FR_DENIED ||
//                er == FR_EXIST) {
//        // This is not a disk error, do not need to reset disk
//        break;
//      }
//      else {
//        // Disk error, reset SD
//        SD_unmount();
//        SD_link();
//        if(SD_mount() != FR_OK) {
//          //reset failed go around and try again
//          fp->error = 1;
//        }
//      }
//    }
//    else {
//      success = 1;
//    }
//
//    if( (HAL_GetTick() - startTime) >  timeout) {
//      er = -2;
//      break;
//    }
//  } while(success == 0);

  return er;
}

int SD_closeSafe (SD_FIL* fp, uint32_t timeout) {					/* Close an open file object */
  //as read write safe stuff is safe, it doesnt matter if this fails
  f_close(&fp->f);
  return 0;
}

int SD_readSafe (SD_FIL* fp, void* buff, UINT btr, UINT* br, uint32_t timeout) {			/* Read data from a file */
//  uint32_t startTime = HAL_GetTick();
//  uint32_t rwPointer = f_tell(&fp->f);
  int er;
//
//  do {
//    //check timeout
//    if( (HAL_GetTick() - startTime) >  timeout) {
//      er = -2;
//      break;
//    }
// 
//    //do read
//    if(fp->error == 0) {
      er = f_read(&fp->f, buff, btr, br);
//    }
//    else {
//      er = -1;
//    }
//
//    //handle errors
//    if(er != FR_OK) {
//      #ifdef UART_DEBUG
//      DEBUG_print(&debugInstance, "f_read failed: %i\r\n", er);
//      #endif
//      SD_unmount();
//      SD_link();
//      if(SD_mount() != FR_OK) {
//        //reset failed go around and try again
//        fp->error = 1;
//        er = -1;
//      }
//      else {
//        if( f_open(&fp->f, fp->filename, fp->flags) != FR_OK ) {
//          er = -3;
//          fp->error = 1;
//          continue;
//        } 
//        if(f_lseek(&fp->f, rwPointer) != FR_OK) {
//          er = -4;
//          fp->error = 1;
//          continue;
//        }
//        fp->error = 0; //error was handled succesfully
//      }
//    }
//  } while( er != FR_OK );
//
//  writeTime = HAL_GetTick() - startTime;

  return er;
}

int SD_writeSafe (SD_FIL* fp, const void* buff, UINT btw, UINT* bw, uint32_t timeout) {
//  uint32_t startTime = HAL_GetTick();
//  uint32_t rwPointer = f_tell(&fp->f);
  int er;
//
//  do {
//    //check timeout
//    if( (HAL_GetTick() - startTime) >  timeout) {
//      er = -2;
//      break;
//    }
// 
//    //do read
//    if(fp->error == 0) {
      er = f_write(&fp->f, buff, btw, bw);
//      if(er == FR_OK) {
//        er = f_sync(&fp->f);
//      }
//    }
//    else {
//      er = -1;
//    }
//
//    //handle errors
//    if(er != FR_OK) {
//      #ifdef UART_DEBUG
//      DEBUG_print(&debugInstance, "f_write failed: %i\r\n", er);
//      #endif
//      #ifdef DEBUG_PRINT
//      debug_printf("SD: handle write error!\n");
//      #endif
//      SD_unmount();
//      SD_link();
//      if(SD_mount() != FR_OK) {
//        //reset failed go around and try again
//        fp->error = 1;
//        er = -1;
//      }
//      else {
//        if( f_open(&fp->f, fp->filename, fp->flags) != FR_OK ) {
//          er = -3;
//          fp->error = 1;
//          continue;
//        } 
//        if(f_lseek(&fp->f, rwPointer) != FR_OK) {
//          er = -4;
//          fp->error = 1;
//          continue;
//        }
//        fp->error = 0; //error was handled succesfully
//      }
//    }
//  } while( er != FR_OK );

  return er;
}

int SD_lseekSafe (SD_FIL* fp, DWORD ofs, uint32_t timeout) {
//  uint32_t startTime = HAL_GetTick();
  int er;
//
//  do {
//    //check timeout
//    if( (HAL_GetTick() - startTime) >  timeout) {
//      er = -2;
//      break;
//    }
// 
//    //do read
//    if(fp->error == 0) {
      er = f_lseek(&fp->f, ofs);
//    }
//    else {
//      er = -1;
//    }
//
//    //handle errors
//    if(er != FR_OK) {
//      #ifdef UART_DEBUG
//      DEBUG_print(&debugInstance, "f_lseek failed: %i\r\n", er);
//      #endif
//      #ifdef DEBUG_PRINT
//      debug_printf("SD: handle write error!\n");
//      #endif
//      SD_unmount();
//      SD_link();
//      if(SD_mount() != FR_OK) {
//        //reset failed go around and try again
//        fp->error = 1;
//        er = -1;
//      }
//      else {
//        if( f_open(&fp->f, fp->filename, fp->flags) != FR_OK ) {
//          er = -3;
//          fp->error = 1;
//          continue;
//        }
//        fp->error = 0; //error was handled succesfully
//      }
//    }
//  } while( er != FR_OK );

  return er;
}



/********************************
    app functions
*******************************/


int SD_link() {
  return (int) FATFS_LinkDriver(&SD_Driver, SDPath);
}

int SD_mount() {
  return (int) f_mount(&SDFatFs, (TCHAR const*)SDPath, 1);
}

int SD_unmount() {
  f_mount(0, "", 0);
  FATFS_UnLinkDriver(SDPath);
  HAL_GPIO_WritePin(SD_PWR_GPIO, SD_PWR, SD_PWR_OFF);
}

/*
  perform power cycle

  as described in the SD specification: p92, http://users.ece.utexas.edu/~valvano/EE345M/SD_Physical_Layer_Spec.pdf

  VDD must be disconnected for at least 1ms, all other pins must be driven low during this time
*/
void SD_powerCycle() {
  SD_CS_LOW();
  spi1_deInit();
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /* Configure SPI SCK */
  GPIO_InitStruct.Pin = SPI1_SCK_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(SPI1_SCK_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SPI1_SCK_GPIO_PORT, SPI1_SCK_PIN, 0);

  /* Configure SPI MISO and MOSI */ 
  GPIO_InitStruct.Pin = SPI1_MISO_PIN;
  HAL_GPIO_Init(SPI1_MISO_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SPI1_SCK_GPIO_PORT, SPI1_MISO_PIN, 0);


  GPIO_InitStruct.Pin = SPI1_MOSI_PIN;
  HAL_GPIO_Init(SPI1_MOSI_GPIO_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(SPI1_SCK_GPIO_PORT, SPI1_MOSI_PIN, 0);

  HAL_GPIO_WritePin(SD_PWR_GPIO, SD_PWR, SD_PWR_OFF);

  HAL_Delay(40);//measurement showed that 25 was correct

  HAL_GPIO_WritePin(SD_PWR_GPIO, SD_PWR, SD_PWR_ON);
  SD_CS_HIGH();
  spi1SetConfig(&SDSpiConfig);

//  HAL_Delay(300);
}

//test slow power up of SD
void SD_test() {
      GPIO_InitTypeDef  GPIO_InitStruct;

      SD_FIL file, rfile;
      uint8_t name[] = "test.txt";
      uint8_t tbuf[100], rbuf[500];
      uint32_t wlen;
      uint32_t time;
      int error, count = 0;;

      SD_link();
      SD_mount();

      //create/open a file
      
      error = SD_openSafe(&file, name, FA_OPEN_ALWAYS | FA_WRITE, 2000);
      if(error != FR_OK) {
        #ifdef DEBUG_PRINT
        debug_printf("fopen error!\n");
        #endif
        while(1);
      }

      //delete old file data
      if(f_truncate(&file.f) != FR_OK) {
        #ifdef DEBUG_PRINT
        debug_printf("f_truncate error!\n");
        #endif
        while(1);
      }


      while(1) {
        sprintf(tbuf, "here is a test string: %i\r\n", count++);
        error = SD_writeSafe(&file, tbuf, strlen(tbuf), &wlen, 2000);
        if( (error != FR_OK) || (wlen != strlen(tbuf)) ) {
          #ifdef DEBUG_PRINT
          debug_printf("f_write error!\n");
          #endif
          while(1);
        }

        if(count == 5) {
          HAL_GPIO_WritePin(SD_PWR_GPIO, SD_PWR, SD_PWR_OFF); //turn off
        }

        if(count > 10) {
          break;
        }
      }

      if(SD_closeSafe(&file, 2000) != FR_OK) {
        #ifdef DEBUG_PRINT
        debug_printf("f_close error!\n");
        #endif
        while(1);
      }

      error = SD_openSafe(&file, name,FA_READ, 2000);
      if(error != FR_OK) {
        #ifdef DEBUG_PRINT
        debug_printf("fopen error!\n");
        #endif
        while(1);
      }

      error = SD_readSafe(&file, rbuf, sizeof(rbuf), &wlen, 2000);
      if(error != FR_OK) {
        #ifdef DEBUG_PRINT
        debug_printf("fread error!\n");
        #endif
        while(1);
      }
      rbuf[wlen] = 0;
      #ifdef DEBUG_PRINT
      debug_printf("Read success!\n%s\n", rbuf);
      #endif


      if(SD_closeSafe(&file, 2000) != FR_OK) {
        #ifdef DEBUG_PRINT
        debug_printf("f_close error!\n");
        #endif
        while(1);
      }

      

      SD_unmount();

      #ifdef DEBUG_PRINT
      debug_printf("DONE\n");
      #endif
      while(1);

}
/*
#define STOP_BUTTON_TIME 3000
#define START_BUTTON_TIME 1000
uint32_t SD_toughTest() {
  uint8_t testString[512];
  uint32_t testLen =512;
  memset(testString, 'A', testLen);
  SD_FIL file;
  int fName = 0, mainLoopCount = 0, stop = 1, writtenSoFar = 0, error = 0, percent = 0;
  uint32_t wlen;
  uint8_t nameStr[20];
  while(1) {
    if(stop) {
      LED_OFF(LED1 | LED2 | LED3);
      mainLoopCount = 0;
      fName = 0;
      if(button_hasBeenPressedForMs(START_BUTTON_TIME)) {
        stop  = 0;
      }
    }
    else {
      LED_ON(LED1 | LED2 | LED3);
      for(fName = 0; fName < 800 && stop == 0; fName++) {
        sprintf(nameStr, "%i.txt", fName);

        #ifdef UART_DEBUG
        DEBUG_print(&debugInstance, "Test num: %i/%s\r\nWrite:\r\n", mainLoopCount, nameStr);
        #endif

        SD_link();
        SD_mount();

        //create/open a file
      
        error = SD_openSafe(&file, nameStr, FA_OPEN_ALWAYS | FA_WRITE, 2000);
        if(error != FR_OK) {
          #ifdef DEBUG_PRINT
          debug_printf("fopen error!\n");
          #endif
          while(1);
        }

        //delete old file data
        if(f_truncate(&file.f) != FR_OK) {
          #ifdef DEBUG_PRINT
          debug_printf("f_truncate error!\n");
          #endif
          while(1);
        }

        writtenSoFar = 0;
        percent = 10;
        while(writtenSoFar < 39000000) {
          error = SD_writeSafe(&file, testString, testLen, &wlen, 2000);
          if(error != FR_OK) {
            #ifdef UART_DEBUG
            DEBUG_print(&debugInstance, "WriteSafe failed! %i\r\n", error);
            #endif
            while(1);
          }
          writtenSoFar += wlen;
          if( writtenSoFar > ( percent * (39000000 / 100) ) ){
            #ifdef UART_DEBUG
            DEBUG_print(&debugInstance, "%i%%\r\n", percent);
            #endif
            percent += 10;
          }
          if(button_hasBeenPressedForMs(STOP_BUTTON_TIME)) {
            stop = 1;
            break;
          }
        }

        error = SD_lseekSafe(&file, 0, 2000); //move file pointer
        if(error != FR_OK) {
          #ifdef UART_DEBUG
          DEBUG_print(&debugInstance, "lseekSafe failed! %i\r\n", error);
          #endif
          while(1);
        }
        error = SD_writeSafe(&file, testString, 44, &wlen, 2000);
        if(error != FR_OK) {
          #ifdef UART_DEBUG
          DEBUG_print(&debugInstance, "WriteSafe header failed! %i\r\n", error);
          #endif
          while(1);
        }



        #ifdef UART_DEBUG
        DEBUG_print(&debugInstance, "Read and verify:\r\n", mainLoopCount, nameStr);
        #endif
        error = SD_closeSafe(&file, 2000);
        if(error != FR_OK) {
          #ifdef UART_DEBUG
          DEBUG_print(&debugInstance, "closeSafe failed! %i\r\n", error);
          #endif
          while(1);
        }

        error = SD_openSafe(&file, nameStr, FA_READ, 2000);
        if(error != FR_OK) {
          #ifdef DEBUG_PRINT
          debug_printf("fopen error!\n");
          #endif
          while(1);
        }

        int readlen = 512;
        uint8_t rBuf[512];
        percent = 0;
        int errorCount = 0;
        writtenSoFar = 0;
        
        do {
          error = SD_readSafe(&file, rBuf, readlen, &wlen, 2000);
          if(error != FR_OK) {
            #ifdef UART_DEBUG
            DEBUG_print(&debugInstance, "readsafe failed! %i\r\n", error);
            #endif
            while(1);
          }
          writtenSoFar += wlen;
          if( writtenSoFar > ( percent * (39000000 / 100) ) ){
            #ifdef UART_DEBUG
            DEBUG_print(&debugInstance, "%i%%\r\n", percent);
            #endif
            percent += 10;
          }
          if(button_hasBeenPressedForMs(STOP_BUTTON_TIME)) {
            stop = 1;
            break;
          }
          for(int i = 0; i < wlen; i++) {
            if(rBuf[i] != 'A') {
              errorCount++;
            }
          }
        } while(wlen == readlen);
        if(errorCount != 0) {
          #ifdef UART_DEBUG
          DEBUG_print(&debugInstance, "Verify failed! %i incorrect bytes\r\n", errorCount);
          #endif          
        }

        error = SD_closeSafe(&file, 2000);
        if(error != FR_OK) {
          #ifdef UART_DEBUG
          DEBUG_print(&debugInstance, "closeSafe failed! %i\r\n", error);
          #endif
          while(1);
        }     

        SD_unmount();
        HAL_Delay(25);
      }
      if(stop) {
        #ifdef UART_DEBUG
        DEBUG_print(&debugInstance, "STOP\r\n");
        #endif
        while(button_hasBeenPressedForMs(STOP_BUTTON_TIME));
      }
      mainLoopCount++;
    }
  }
}

*/
/***************************************
        adafruit sd test functions
****************************************/

//#define SD_TEST_MAX_SECTOR ( (uint32_t) 62309375 )
#define SD_TEST_MAX_SECTOR ( (uint32_t) 25 )
#define SD_TEST_SECTOR_DIFFERENCE ( (uint32_t) 25 )
//#define SD_TEST_SECTOR_DIFFERENCE ( (uint32_t) 500000 )
#define SD_TEST_SECTOR_SIZE 512

void SD_test_write() {
  BSP_SD_Init();

  uint32_t currentSector = SD_TEST_MAX_SECTOR;
  uint8_t data[SD_TEST_SECTOR_SIZE], rBuf[SD_TEST_SECTOR_SIZE];
  memset(data, 'A', SD_TEST_SECTOR_SIZE);

  #ifdef DEBUG_PRINT
  debug_printf("data: %c\n", data[0]);
  #endif

  int er;

  // write blocks
  while(currentSector >= SD_TEST_SECTOR_DIFFERENCE) {
    #ifdef DEBUG_PRINT
    debug_printf("test sector: %u\n\twriting...\n", currentSector);
    #endif

    BSP_SD_Erase(currentSector, currentSector);

    er = BSP_SD_WriteBlocks((uint32_t *)data, currentSector, 1, 5000);
    if(er != BSP_SD_OK) {
      #ifdef DEBUG_PRINT
      debug_printf("\tSD write failed: %i\n", er);
      #endif
    }

    #ifdef DEBUG_PRINT
    debug_printf("\tcomplete\n");
    #endif

    currentSector -= SD_TEST_SECTOR_DIFFERENCE;
  }

  //read blocks
  currentSector = SD_TEST_MAX_SECTOR - 1;
  while(currentSector >= SD_TEST_SECTOR_DIFFERENCE) {
    memset(rBuf, 0, SD_TEST_SECTOR_SIZE);
    #ifdef DEBUG_PRINT
    debug_printf("test sector: %u\n\verifying...\n", currentSector);
    #endif

    er = BSP_SD_ReadBlocks((uint32_t *)rBuf, currentSector, 1, 5000);
    if(er != BSP_SD_OK) {
      #ifdef DEBUG_PRINT
      debug_printf("\tSD read failed: %i\n", er);
      #endif
    }

    if(memcmp(data, rBuf, SD_TEST_SECTOR_SIZE) != 0) {
      #ifdef DEBUG_PRINT
      debug_printf("\tVerify failed\n");
      #endif
    }
    else {
      #ifdef DEBUG_PRINT
      debug_printf("\tSuccess!\n");
      #endif
    }

    currentSector -= SD_TEST_SECTOR_DIFFERENCE;
  }

  #ifdef DEBUG_PRINT
  debug_printf("Test complete\n");
  #endif

  while(1);
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SD ************************************/
/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_IO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;
  uint8_t counter;

  /* SD_CS_GPIO Periph clock enable */
__HAL_RCC_GPIOA_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
   GPIO_InitStruct.Pin = SD_CS;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_PULLUP;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   HAL_GPIO_Init(SD_CS_GPIO, &GPIO_InitStruct);

   GPIO_InitStruct.Pin = SD_PWR;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
   HAL_GPIO_Init(SD_PWR_GPIO, &GPIO_InitStruct);
   HAL_GPIO_WritePin(SD_PWR_GPIO, SD_PWR, SD_PWR_ON); //turn SD on


  spi1_init(&SDSpiConfig);

  SD_powerCycle();

  /* SD chip select high */
  
  SD_CS_HIGH();
  SD_CS_LOW();
  SD_CS_HIGH();
  SD_CS_LOW();
  SD_CS_HIGH();
  
  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 3; counter++)
  {
    /* Send dummy byte 0xFF */
    //SD_IO_WriteByte(0xCC);
  }
}

/**
  * @brief  Set the SD_CS pin.
  * @param  pin value.
  * @retval None
  */
void SD_IO_CSState(uint8_t val)
{
  if(val == 1) 
  {
    SD_CS_HIGH();
  }
  else
  {
    //spi1SetConfig(&SDSpiConfig);
    SD_CS_LOW();
  }
}

/**
  * @brief  Write a byte on the SD.
  * @param  Data: byte to send.
  * @retval None
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = spi1ReadWrite((uint8_t*) DataIn, DataOut, DataLength, 100);
    
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    #ifdef DEBUG
    uasrt_logging_printf ("SD SPI FAILED: %i\n", status);
    #endif
  }
}
//
///**
//  * @brief  Writes a byte on the SD.
//  * @param  Data: byte to send.
//  * @retval None
//  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;
  /* Send the byte */
  SD_IO_WriteReadData(&Data,&tmp,1);
  return tmp;
}


