#ifndef _ADC_AUDIO_H_
#define _ADC_AUDIO_H_

#ifdef DEBUG
#include <__cross_studio_io.h>
#endif
#include "stm32f7xx_hal.h"
#include "main.h"
//#include "stm32f7xx_nucleo_144.h"


#define ADC_DMAbufsize 1024//512 * 11 // 2 pages of 1 byte data
#define SAI_DMAbufsize 8*ADC_DMAbufsize

extern SAI_HandleTypeDef hsai_BlockA1;
extern DMA_HandleTypeDef hdma_sai1_a;




typedef struct {
  uint16_t array[ADC_DMAbufsize/2];
  uint8_t ready;
  void * next;
} listMember_t;

typedef enum {WAIT, START, RECORDING, FINISH, AUDIO_ERROR} SAI_DMA_statusEnum;


HAL_StatusTypeDef SAI_DMA_init();
HAL_StatusTypeDef SAI_DMAstart();
uint16_t SAI_readDMA();
void HAL_SAI_RxConvHalfCpltCallback(SAI_HandleTypeDef* hsai);
void HAL_SAI_RxConvCpltCallback(SAI_HandleTypeDef* hsai);
SAI_DMA_statusEnum SAI_getstatus();
void SAI_setstatus(SAI_DMA_statusEnum status);
HAL_StatusTypeDef SAI_DMAstop();



//length of data = ADC_DMAbufsize/2
uint16_t * SAI_addToList();
void SAI_markLastReady();
uint16_t * SAI_readNextArray();
void SAI_free();
uint32_t SAI_getListFill();
#endif