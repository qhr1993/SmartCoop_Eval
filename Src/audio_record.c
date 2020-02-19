/*
*     Functions to stream audio to SD card (WAV file)
*
*     how it works:
*       ADC_audio.c is responsible for setting up the ADC and storing the audio into a local buffer
*       audio_record.c moves this data to the SD card ensuring all SD card issues are passed up to the caller
*               so that they can be handled appropriately (reseting the SD card here could break other parts of 
*               the code which already have open files)
*
*/

#include "audio_record.h"
#include "ADC_audio.h"
#include "ff.h"
#include "SD.h"
#include "main.h"
#include "WAV.h"
#include <stdint.h>

struct {
  FIL file;
  uint32_t blockCount;
  uint8_t isRecording;
  uint8_t fileIsOpen;
} audioRecord_inst = {0};

void _audioRecord_cleanUp();

/*
  audioRecord_start: start the ADC, start the timer, opens the file
*/
int audioRecord_start(uint8_t * filename) {
  audioRecord_inst.isRecording = 0;
  audioRecord_inst.fileIsOpen = 0;
  audioRecord_inst.blockCount = 0;
  
  int error;
  
  #ifdef DEBUG_PRINT
  debug_printf("start recording\n");
  #endif


  error = f_open(&audioRecord_inst.file, filename, FA_OPEN_ALWAYS | FA_WRITE);
  if(error != FR_OK) {
    #ifdef DEBUG_PRINT
    debug_printf("fopen error!\n");
    #endif
    _audioRecord_cleanUp();
    return AUDIO_RECORD_SD_ERROR;
  }

  //delete old file data
  if(f_truncate(&audioRecord_inst.file) != FR_OK) {
    _audioRecord_cleanUp();
    return AUDIO_RECORD_SD_ERROR;
  }

  //begin recording process
  //Timer_start();
  SAI_DMAstart();
  audioRecord_inst.isRecording = 1;
  return 0;
}

/*
  audioRecord_process: write next page of data to SD card, chose to only write one page at a time so that
                       a large backlog of data will not cause the application to freeze whilst it writes the
                       data to the SD card. This means that, for example, the watchdog can be handled.
*/
int audioRecord_process() {
  uint32_t bw;
  uint16_t * data = SAI_readNextArray();
  uint16_t * data_ext = &(data[ADC_DMAbufsize/4]);
  int error = 0;
  if(data != NULL) {
    //wait for it to be read read
    error = f_write(&audioRecord_inst.file, data, ADC_DMAbufsize/2, &bw);
    error = f_write(&audioRecord_inst.file, data_ext, ADC_DMAbufsize/2, &bw);
    if(error != FR_OK) {
      _audioRecord_cleanUp();
      return AUDIO_RECORD_SD_ERROR;
    }
    audioRecord_inst.blockCount++;
  
    SAI_free();//IMPORTANT!!!!! this must be called as ADC_readNextArray does not free the memory
  }

  return 0;
}

/*
  audioRecord_finishUp: stop the ADC, write any remaining data to the SD
*/
int audioRecord_finishUp() {
  uint32_t bw;
  uint16_t * data = SAI_readNextArray();
  uint16_t * data_ext = &(data[ADC_DMAbufsize/4]);
  int error = 0;
  uint8_t header[44];

  //stop timer/dma, save remaining data to SD, add header to file, close file
  SAI_DMAstop();
  //Timer_stop();
  #ifdef DEBUG_PRINT
  debug_printf("stop recording\n");
  #endif

  //save the last of the data
  data = SAI_readNextArray();
  data_ext = &(data[ADC_DMAbufsize/4]);

  while(data != NULL) {
  error = f_write(&audioRecord_inst.file, data, ADC_DMAbufsize/2, &bw);
  error += f_write(&audioRecord_inst.file, data_ext, ADC_DMAbufsize/2, &bw);
    //wait for it to be read read
    if( error != FR_OK) {
      _audioRecord_cleanUp();
      return AUDIO_RECORD_SD_ERROR;
    }

    audioRecord_inst.blockCount++;
  
    SAI_free();//IMPORTANT!!!!! this must be called as ADC_readNextArray does not free the memory
    //get next array if there is one
    data = SAI_readNextArray();
  }
  #ifdef DEBUG_PRINT
  debug_printf("File size: %u\n", f_size(&audioRecord_inst.file));
  #endif
  WAV_HeaderGen(header,  f_size(&audioRecord_inst.file), 22050, 16);
  //write correct header to start of file
  if(f_lseek(&audioRecord_inst.file, 0) != FR_OK) {
    _audioRecord_cleanUp();
    return AUDIO_RECORD_SD_ERROR;
  }

  error = f_write(&audioRecord_inst.file, header, 44, &bw);
  if(error != FR_OK) {
    _audioRecord_cleanUp();
    return AUDIO_RECORD_SD_ERROR;
  }

  error = f_close(&audioRecord_inst.file);
  if(error != FR_OK) {
    _audioRecord_cleanUp();
    return AUDIO_RECORD_SD_ERROR;
  }
}


/*
  _audioRecord_cleanUp: close audio file and stop recording when an error occurs
          This will be called before returning from any function within this file
          that will return an error, this stops the open file remaining unhandled 
          or the audio DMA to be left running
*/
void _audioRecord_cleanUp() {
  uint16_t * data;

  if(audioRecord_inst.fileIsOpen == 1) {
    f_close(&audioRecord_inst.file);
    audioRecord_inst.fileIsOpen = 0;
  }

  if(audioRecord_inst.isRecording == 1) {
    //stop adc
    SAI_DMAstop();
    //Timer_stop();

    // clear local data
    audioRecord_inst.isRecording = 0;
    audioRecord_inst.blockCount = 0;

    // clear remaining audio data
    do {
      data = SAI_readNextArray();
      SAI_free();
    } while(data != NULL);
  }
}

/*
void audioRecord_test() {
  SD_link();
  SD_mount();

  uint32_t startTime = HAL_GetTick();

  audioRecord_start("testFile.wav");

  while( HAL_GetTick() - startTime < 5000) {
    audioRecord_process();
  }
  

  audioRecord_finishUp();

  SD_unmount();

  #ifdef DEBUG_PRINT
  debug_printf("Test complete\n");
  #endif

  while(1);
}*/