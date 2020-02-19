// Set up ADC to record audio with DMA

#include "stm32f7xx_hal.h"
#include "ADC_audio.h" 
#include "main.h"
//#include "ADC.h"
//#include "static_test_data.c"
#include "pdm2pcm_glo.h"

SAI_DMA_statusEnum SAI_DMA_status = WAIT;
int missedSamples = 0;
uint8_t ite = 0;

uint16_t test_addr_offset = 0;

//original code designed to create a DMA buffer with 1024 samples using adc
//when using SAI with oversampling = 128, 1 sample <=> 128 / 16 = 8 uint16_t, henece a total depth of 8 * 1024 = needed

#define AUDIO_STORAGE_SIZE 120   // (120 * 512) / 22500 = 2.73s, therefor buffer can store 2.73 seconds of audio
struct {
  uint32_t oldest;
  uint32_t fill;
  uint32_t newestInUse;
  listMember_t buf[AUDIO_STORAGE_SIZE];  
} audioStorage = {0};


//volatile uint32_t uhADCxConvertedValue[ADC_DMAbufsize] = {0};
//volatile uint32_t * adata = &(uhADCxConvertedValue[0]);
volatile uint16_t u_SAI_PDM_value[SAI_DMAbufsize] = {0};
volatile uint16_t * adata = &(u_SAI_PDM_value[0]);

PDM_Filter_Handler_t PDM1_filter_handler;
PDM_Filter_Config_t PDM1_filter_config;

HAL_StatusTypeDef SAI_DMA_Init() {
 MX_DMA_Init();
 MX_SAI1_Init();

 __HAL_RCC_CRC_CLK_ENABLE();
 CRC->CR = CRC_CR_RESET;

 PDM1_filter_handler.bit_order = PDM_FILTER_BIT_ORDER_LSB;
 PDM1_filter_handler.endianness = PDM_FILTER_ENDIANNESS_BE;
 PDM1_filter_handler.high_pass_tap = 2122358088;// ;
 PDM1_filter_handler.out_ptr_channels = 1;
 PDM1_filter_handler.in_ptr_channels = 1;
 PDM_Filter_Init((PDM_Filter_Handler_t *)(&PDM1_filter_handler));
 PDM1_filter_config.output_samples_number = ADC_DMAbufsize/2;
 PDM1_filter_config.mic_gain = 32;
 PDM1_filter_config.decimation_factor = PDM_FILTER_DEC_FACTOR_128;
 PDM_Filter_setConfig((PDM_Filter_Handler_t *)&PDM1_filter_handler, &PDM1_filter_config);
}

HAL_StatusTypeDef SAI_DMAstart() {
  /*##-3- Start the conversion process #######################################*/
  HAL_NVIC_EnableIRQ (DMA2_Stream0_IRQn);
  if( HAL_SAI_Receive_DMA( &hsai_BlockA1, (uint8_t *) adata, SAI_DMAbufsize)!= HAL_OK)
  {
    /* Start Conversation Error */
    #ifdef DEBUG
    uasrt_logging_printf ("DMA start Fail\n");
    #endif
    return HAL_ERROR;
  }
  else
    return HAL_OK;

  //HAL_ADC_Start(&Adc1Handle);
}

HAL_StatusTypeDef SAI_DMAstop() {
   HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
  if(HAL_SAI_DMAStop (&hsai_BlockA1) != HAL_OK) {
    #ifdef DEBUG
    uasrt_logging_printf ("DMA stop Fail\n");
    #endif
    return HAL_ERROR;
  }
  else
    return HAL_OK;
}


uint16_t SAI_readDMA() {
  return u_SAI_PDM_value[0];
}


/**
  * @brief  Regular conversion complete callback in non blocking mode 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
  if(hsai->Instance == hsai_BlockA1.Instance) {  
    uint16_t * wbuf = SAI_addToList();
      uint16_t * rbuf = &(u_SAI_PDM_value[SAI_DMAbufsize/2]);
      PDM_Filter(rbuf, wbuf, &PDM1_filter_handler);
      #ifdef SD_IO_TEST
      uint16_t tt = 0xEE00;
      for(int j = 0; j < ADC_DMAbufsize/2; j++) {
       wbuf[j] = tt;
        tt += 1;
      }
      #endif
      //for(int j = 0; j < ADC_DMAbufsize/2; j++) {
      // wbuf[j] = ((*(rbuf + 1))<<4) + ((*(rbuf))>>4);
      //  rbuf += 2;
      //}
    #ifdef STATIC_TESTING
      uint8_t * rbuf = (uint8_t *)&fTestData[test_addr_offset];
      for(int j = 0; j < ADC_DMAbufsize/2; j++) {
       wbuf[j] = * rbuf;
        rbuf += 1;
      }
      test_addr_offset  +=  ADC_DMAbufsize/2;
      if ( ( test_addr_offset + ADC_DMAbufsize/2 ) > DATASIZE)
        test_addr_offset = 0;
    #endif
    
  //  memset(wbuf, ite++, 512);
    SAI_markLastReady();
  //  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
  }
  else {
    //ADC_cnvComplete(hadc);

  }
}

/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode 
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
      uint16_t * wbuf = SAI_addToList();
      uint16_t * rbuf = &(u_SAI_PDM_value[0]);
      PDM_Filter(rbuf, wbuf, &PDM1_filter_handler);
      #ifdef SD_IO_TEST
      uint16_t tt = 0xAA00;
      for(int j = 0; j < ADC_DMAbufsize/2; j++) {
       wbuf[j] = tt;
        tt += 1;
      }
      #endif
    #ifdef STATIC_TESTING
      uint8_t * rbuf = (uint8_t *)&fTestData[test_addr_offset];
      for(int j = 0; j < ADC_DMAbufsize/2; j++) {
       wbuf[j] = * rbuf;
        rbuf += 1;
      }
      test_addr_offset  +=  ADC_DMAbufsize/2;
      if ( ( test_addr_offset + ADC_DMAbufsize/2 ) > DATASIZE)
        test_addr_offset = 0;
    #endif
//  memset(wbuf, ite++, 512);
  SAI_markLastReady();
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

SAI_DMA_statusEnum SAI_getstatus() {
  return SAI_DMA_status;
}

void ADC_setstatus(SAI_DMA_statusEnum status) {
  SAI_DMA_status = status;
}

//length of data = ADC_DMAbufsize/2
uint16_t * SAI_addToList() {
  if(audioStorage.newestInUse == 1) {
    return NULL; //already got one out in the field
  }
  if(audioStorage.fill == AUDIO_STORAGE_SIZE) {
    //overflow delete the oldest entry
    audioStorage.oldest = MOD(audioStorage.oldest + 1, AUDIO_STORAGE_SIZE);
    audioStorage.fill--;
  }
  uint16_t * retval = audioStorage.buf[MOD(audioStorage.oldest + audioStorage.fill, AUDIO_STORAGE_SIZE)].array;
  audioStorage.newestInUse = 1;
  return retval;
//  memcpy(lp->array, data, (ADC_DMAbufsize/2));
}

//set last list member as ready
void SAI_markLastReady() {
  if(audioStorage.newestInUse  == 0) {
    return;
  }
  audioStorage.newestInUse = 0;
  audioStorage.fill++;
}

uint16_t * SAI_readNextArray() {
  if(audioStorage.fill == 0) {
    return NULL;
  }
  uint16_t * retval = audioStorage.buf[audioStorage.oldest].array;
  return retval;
}

void SAI_free() {
  audioStorage.oldest = MOD(audioStorage.oldest + 1, AUDIO_STORAGE_SIZE);
  audioStorage.fill--;
}

uint32_t SAI_getListFill() {
  return audioStorage.fill;
}
