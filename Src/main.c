
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#define AUDIO_PROCESSING_MS 5000
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include "TSL2591.h"
#include "SD.h"
#include "ADC_audio.h"
#include "audio_record.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
static __IO uint32_t TimingDelay;
uint8_t logging_buffer[100];

I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi1;
SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai1_a;


GPIO_InitTypeDef gpioPB;

USART_HandleTypeDef husart3;

FIL sensorFile;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

typedef enum {
  STATE_IDLE,
  STATE_RECORDING
} states_t;

static volatile states_t current_state = STATE_IDLE;
static volatile uint8_t state_change_flag = 0;
uint16_t seq = 0;

uint16_t audioListFill = 0, peakListFill = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_SAI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_Init(void);
static void MX_GPIOPB_Init(void);
void uasrt_logging_printf( const char * format, ... );
void HAL_SYSTICK_Callback(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Delay(__IO uint32_t nTime);

//ASM funcs
static void state_machine(void);
static void state_IDLE_entry(void);
static void state_IDLE_main(void);
static void state_IDLE_exit(void);
static void state_RECORDING_entry(void);
static void state_RECORDING_main(void);
static void state_RECORDING_exit(void);


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_Init();
  MX_GPIOPB_Init();
  MX_DMA_Init();
  MX_SAI1_Init();
  SAI_DMA_Init();
  MX_I2C2_Init();
  //MX_SPI1_Init(); //the init implemented by SPI.c
  /* USER CODE BEGIN 2 */
  uasrt_logging_printf ("Program started!!!\r\n");
  //uint16_t porta[256];
  //HAL_StatusTypeDef error = HAL_SAI_Receive_DMA( &hsai_BlockA1,porta,128);
 // uasrt_logging_printf ("%d\r\n",error);
    /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  //TSL2591_begin(&hi2c2);

  while (1)
  {

  /* USER CODE END WHILE */
 state_machine();
  
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 113;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 4;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLI2S;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00303D5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  uasrt_logging_printf ("I2C Bus: is si ready? 0x%04X\r\n",  HAL_I2C_IsDeviceReady (&hi2c2, 0x0040 << 1,1,100));
  uasrt_logging_printf ("I2C Bus: is sht ready? 0x%04X\r\n",  HAL_I2C_IsDeviceReady (&hi2c2, 0x0044 << 1,1,100));
  uasrt_logging_printf ("I2C Bus: is tsl ready? 0x%04X\r\n",  HAL_I2C_IsDeviceReady (&hi2c2, TSL2591_ADDR << 1,1,100));
  HAL_Delay(1000);
}

/* SAI1 init function */
void MX_SAI1_Init(void)
{

  hsai_BlockA1.Instance = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_MCKDIV;
  hsai_BlockA1.Init.Mckdiv = 5;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.FrameInit.FrameLength = 128;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 8;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_SPI1_DeInit() {
  HAL_SPI_DeInit(&hspi1);
}

/* USART3 init function */
static void MX_USART3_Init(void)
{

  husart3.Instance = USART3;
  husart3.Init.BaudRate = 115200;
  husart3.Init.WordLength = USART_WORDLENGTH_8B;
  husart3.Init.StopBits = USART_STOPBITS_1;
  husart3.Init.Parity = USART_PARITY_NONE;
  husart3.Init.Mode = USART_MODE_TX_RX;
  husart3.Init.CLKPolarity = USART_POLARITY_HIGH;
  husart3.Init.CLKPhase = USART_PHASE_1EDGE;
  husart3.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static void MX_GPIOPB_Init(void)
{
  gpioPB.Mode = GPIO_MODE_IT_RISING_FALLING;
  gpioPB.Pull = GPIO_NOPULL;
  gpioPB.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC,&gpioPB);
  
  HAL_NVIC_SetPriority(EXTI15_10_IRQn,0,0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpioPB.Mode = GPIO_MODE_OUTPUT_PP;
  gpioPB.Pull = GPIO_PULLDOWN;
  gpioPB.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB,&gpioPB);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void uasrt_logging_printf( const char * format, ... )
{
  va_list args;
 va_start (args, format);

  vsprintf (logging_buffer, format, args);
  HAL_USART_Transmit(&husart3, logging_buffer, strlen(logging_buffer),1000);
  #ifdef DEBUG
    vprintf(format,args);
  #endif
  va_end (args);
}

void HAL_SYSTICK_Callback(void)
{
if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;
  
  while(TimingDelay != 0);
}

static void state_machine(void)
{
  if (state_change_flag ==1)
  {
    HAL_Delay(200);
    state_change_flag = 0;
    if (current_state == STATE_IDLE)
        {
          state_IDLE_exit();
          state_RECORDING_entry();
        }
    else if (current_state == STATE_RECORDING)
        {
          state_RECORDING_exit();
          state_IDLE_entry();
        }
  }
  switch (current_state)
  {
  case STATE_IDLE:
    state_IDLE_main();break;
  case STATE_RECORDING:
    state_RECORDING_main();break;
   default : uasrt_logging_printf ("undefined state\r\n");
  }
}

static void state_IDLE_entry(void)
{
  uasrt_logging_printf("Entering IDLE...\r\n");
  current_state = STATE_IDLE;
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
}
static void state_IDLE_main(void)
{
    uasrt_logging_printf("Processing IDLE...\r\n");
    HAL_Delay(2000);
}
static void state_IDLE_exit(void)
{
  uasrt_logging_printf("Exiting IDLE...\r\n");
}
static void state_RECORDING_entry(void)
{
   uasrt_logging_printf("Entering RECORDING...\r\n");
   current_state = STATE_RECORDING;
   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
   TSL2591_begin(&hi2c2);

   uasrt_logging_printf ("SD_link() returns %d\r\n",SD_link ());
   
   if(SD_mount() != FR_OK)
    uasrt_logging_printf ("SD mount failed\r\n");
   else
    uasrt_logging_printf ("SD mounted!\r\n");
   uint8_t sensorPathName[5];
   if (seq == 9999)
    seq = 0;
   else
    seq++;
   sprintf (sensorPathName,"%04d",seq);
   FRESULT dir_error = f_mkdir(sensorPathName);
   if ( dir_error == FR_OK)
    uasrt_logging_printf("mkdir %s ok\r\n", sensorPathName);
   else if ( dir_error == FR_EXIST)
    uasrt_logging_printf("mkdir %s existed\r\n", sensorPathName);
    else
     uasrt_logging_printf("mkdir %s error\r\n", sensorPathName);
  uint8_t filename[14];
    strcpy (filename, sensorPathName);
    strcat (filename, "/");
    strcat (filename, sensorPathName);
    strcat (filename, ".csv");
   if(f_open(&sensorFile, filename, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
    uasrt_logging_printf ("open file %s failed\r\n", filename);
    else
    uasrt_logging_printf ("open file %s ok\r\n", filename);
    uint8_t bufferToWrite[20];
    sprintf (bufferToWrite, "lux,si_temp,si_humd,sht_temp,sht_humd\r\n");
    f_write (&sensorFile,bufferToWrite, strlen (bufferToWrite),NULL);
    sprintf (sensorPathName,"%04d",seq);
    strcpy (filename, sensorPathName);
    strcat (filename, "/");
    strcat (filename, sensorPathName);
    strcat (filename, ".wav");
    if (audioRecord_start(filename))
    {
      uasrt_logging_printf("audiorecord start failed ok\r\n");
    }
    else
      uasrt_logging_printf("audiorecord start created %s ok\r\n", filename);
}
static void state_RECORDING_main(void)
{
    uasrt_logging_printf("Processing RECORDING...\r\n");
    uint32_t lux_raw = TSL2591_getFullLuminosity (&hi2c2);
    uint16_t lux_ch0, lux_ch1;
    lux_ch0 = ((lux_raw & 0x000000FF) << 8) + ((lux_raw & 0x0000FF00) >> 8);
    lux_ch1 = ((lux_raw & 0x00FF0000) >> 8) + ((lux_raw & 0xFF000000) >> 24);
    //uasrt_logging_printf ("Lux_raw: ch0 %d ch1 %d\r\n", lux_ch0, lux_ch1);
    uint32_t lux_result = TSL2591_calculateLux(&hi2c2, lux_ch0, lux_ch1);
    uasrt_logging_printf ("Lux: %d\r\n", lux_result);

    uint8_t cmd[] = {0x24, 0x00};

    
    uint8_t sht_data[6];
    HAL_I2C_Master_Transmit (&hi2c2, 0x0044 << 1, cmd, 2, 100);
    HAL_Delay(1);
    while (HAL_I2C_Master_Receive (&hi2c2, 0x0044 << 1, sht_data, 6, 100) != HAL_OK)
    {
      HAL_Delay(10);
    }
    //uasrt_logging_printf ("SHT_raw: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", sht_data[0], sht_data[1],sht_data[2],sht_data[3],sht_data[4],sht_data[5]);
    float sht_temp = (sht_data[0] * 256.0 + sht_data[1]) / 65535.0 * 175 - 45;
    float sht_humd = (sht_data[3] * 256.0 + sht_data[4]) / 65535.0 * 100;
    uasrt_logging_printf ("SHT_value: temp %.2f humd %.2f \r\n", sht_temp, sht_humd);

    uint8_t si_data[6];
    cmd[0] = 0xF5; cmd[1] = 0xF3;
    HAL_I2C_Master_Transmit (&hi2c2, 0x0040 << 1, cmd, 1, 100);
    HAL_Delay(1);
    while (HAL_I2C_Master_Receive (&hi2c2, 0x0040 << 1, si_data, 3, 100) != HAL_OK)
    {
      HAL_Delay(10);
    }
    HAL_I2C_Master_Transmit (&hi2c2, 0x0040 << 1, cmd + 1, 1, 100);
    HAL_Delay(1);
    while (HAL_I2C_Master_Receive (&hi2c2, 0x0040 << 1, si_data + 3, 3, 100) != HAL_OK)
    {
      HAL_Delay(10);
    }
    //uasrt_logging_printf ("Si_raw: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\r\n", si_data[0], si_data[1], si_data[2],si_data[3],si_data[4],si_data[5]);
    float si_humd = (si_data[0] * 256.0 + si_data[1]) / 65535.0 * 125 - 6;
    float si_temp = (si_data[3] * 256.0 + si_data[4]) / 65535.0 * 175.72 - 46.85;
    uasrt_logging_printf ("SI_value: temp %.2f humd %.2f \r\n", si_temp, si_humd);
    uint8_t bufferToWrite[20];
    sprintf (bufferToWrite, "%d,%.2f,%.2f,%.2f,%.2f\r\n",lux_result,si_temp,si_humd,sht_temp,sht_humd);
    f_write (&sensorFile,bufferToWrite, strlen (bufferToWrite),NULL);

     audioListFill = SAI_getListFill(); // Check fill level (for debugging)
      if(audioListFill > peakListFill) {
        peakListFill = audioListFill;
      }

    TimingDelay = AUDIO_PROCESSING_MS;
    uint8_t error = 0;
    while ( (TimingDelay > 0) && (state_change_flag == 0)) {
        error = audioRecord_process();
        if (error)
          uasrt_logging_printf ("audio record error %d\r\n",error);
    }
}
static void state_RECORDING_exit(void)
{
  uasrt_logging_printf("Exiting RECORDING...\r\n");
  if(f_close(&sensorFile) != FR_OK)
    uasrt_logging_printf ("close file failed\r\n");
    else
    uasrt_logging_printf ("close file ok\r\n");
  if (audioRecord_finishUp ())
    uasrt_logging_printf ("close audio record failed\r\n");
  else
    uasrt_logging_printf ("close audio record ok\r\n");
  SD_unmount();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
   if (GPIO_Pin == GPIO_PIN_13)
   {
      //uasrt_logging_printf("interrupt recorded!\r\n");
      state_change_flag = 1;
   }
}




/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
