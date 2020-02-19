#ifndef _SPI_H_
#define _SPI_H_

#include "main.h"

/*
  each device requires a SPIConfig_t to allow SPI to change settings when transmitting
    Configure baud prescaler, phase, pol
*/
typedef struct {
  uint32_t baudRatePrescaler;
  uint32_t polarity;
  uint32_t phase;
} SPIConfig_t;

extern SPI_HandleTypeDef  hspi1;

#define SPI1_MISO_PIN       GPIO_PIN_4
#define SPI1_MISO_GPIO_PORT  GPIOB
#define SPI1_MOSI_PIN       GPIO_PIN_5
#define SPI1_MOSI_GPIO_PORT  GPIOB
#define SPI1_SCK_PIN        GPIO_PIN_3
#define SPI1_SCK_GPIO_PORT   GPIOB

int spi1_init(SPIConfig_t * config);
int spi1_deInit();
HAL_StatusTypeDef spi1SetConfig( SPIConfig_t * config );
HAL_StatusTypeDef spiReadWrite(SPI_HandleTypeDef * handle, uint8_t * txBuf, uint8_t * rxBuf, uint32_t len, uint32_t timeout);
int spi1ReadWrite(uint8_t * txBuf, uint8_t * rxBuf, uint32_t len, uint32_t timeout);
uint8_t SPI1_ReceiveData8();
void SPI1_SendData8(uint8_t data);

#endif //_SPI_H_