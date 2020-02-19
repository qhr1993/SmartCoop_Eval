#ifndef _AUDIO_RECORD_H_
#define _AUDIO_RECORD_H_

#include <stdint.h>

/*
  Error values
*/
#define AUDIO_RECORD_SD_ERROR -1



int audioRecord_start(uint8_t * filename);

int audioRecord_process();

int audioRecord_finishUp();

//void audioRecord_test();
#endif // _AUDIO_RECORD_H_