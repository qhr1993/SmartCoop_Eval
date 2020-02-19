/*
  functions for sending and receiving data
*/

#include "SPI.h"

//SPI_HandleTypeDef  hspi1;


int spi1_init(SPIConfig_t * config) {
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_RESET  ||
     hspi1.Init.BaudRatePrescaler != config->baudRatePrescaler ||
     hspi1.Init.CLKPolarity != config->polarity || 
     hspi1.Init.CLKPhase != config->phase)
  {
    

    HAL_SPI_DeInit(&hspi1);
    /* SPI Config */
    hspi1.Instance = SPI1;
      /* SPI baudrate is set to 13,5 MHz maximum (APB2/SPI_BaudRatePrescaler = 108/8 = 13,5 MHz) 
       to verify these constraints:
          - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint 
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK2 max frequency is 108 MHz 
       */ 
    hspi1.Init.BaudRatePrescaler = config->baudRatePrescaler;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.CLKPhase = config->phase;
    hspi1.Init.CLKPolarity = config->polarity;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    //MSP re-implemented and included in HAL_SPI_Init()
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  }
}

int spi1_deInit() {
  HAL_SPI_DeInit(&hspi1);
  return 0;
}

HAL_StatusTypeDef spi1SetConfig( SPIConfig_t * config ) {
  //update spi settings if they dont match requirements
  spi1_init(config);
  
  return HAL_OK;
}


HAL_StatusTypeDef spiReadWrite(SPI_HandleTypeDef * handle, uint8_t * txBuf, uint8_t * rxBuf, uint32_t len, uint32_t timeout) {

  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(handle, txBuf, rxBuf, len, timeout);
  return  status;
}

int spi1ReadWrite(uint8_t * txBuf, uint8_t * rxBuf, uint32_t len, uint32_t timeout) {
  return spiReadWrite(&hspi1, txBuf, rxBuf, len, timeout);
}

//external: read one byte, write one byte SPI1
uint8_t SPI1_ReceiveData8() {
  uint8_t pBuff = 0xaa;
  HAL_StatusTypeDef retval = spiReadWrite(&hspi1, &pBuff, &pBuff, 1, 500);
  if(retval != HAL_OK) {
    #ifdef DEBUG
    uasrt_logging_printf ("spi1 receive 8 failed\n");
    #endif
  }
  return pBuff;
}

void SPI1_SendData8(uint8_t data){
  uint8_t pBuff = 0xaa;
  HAL_StatusTypeDef retval = spiReadWrite(&hspi1, &data, &pBuff, 1, 500);
  if(retval != HAL_OK) {
    #ifdef DEBUG
    uasrt_logging_printf ("spi1 send 8 failed\n");
    #endif
  }
}
