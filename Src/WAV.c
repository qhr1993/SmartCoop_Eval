//functions to create a WAV file, see http://soundfile.sapp.org/doc/WaveFormat/ for details
#include "WAV.h"


void WAV_HeaderGen(uint8_t *buf, int num_samples, int sample_rate, int bits_per_sample) {
  uint32_t data_size = num_samples * bits_per_sample/8;
  uint8_t * i = buf;
  //RIFF header
  memcpy(i, "RIFF", 4);
  
  i = buf +4; //chuck size, little endian
  #ifdef DEBUG
//  debug_printf("data size: %i", data_size+36);
  #endif
  *((uint32_t *)(i)) = data_size+36;

  i = buf + 8; //format
  memcpy(i, "WAVE", 4);

  //fmt chunk
  i = buf+12; //subchunk id
  memcpy(i, "fmt ", 4);

  i = buf + 16; //sub chunk size
  *((uint32_t *)(i)) = 16;

  i = buf + 20; //audio format, 1 for no compression
  *((uint16_t *)(i)) = 1;

  i = buf + 22; //num channels
  *((uint16_t *)(i)) = 1;

  i = buf + 24; //sample rate
  *((uint32_t *)(i)) = sample_rate;

  i = buf + 28; //byte rate
  *((uint32_t *)(i)) = sample_rate * bits_per_sample/8;

  i = buf +32; //block allign = num channels * bits per sample/8
  *((uint16_t *)(i)) = bits_per_sample/8;

  i = buf + 34; //bits per sample
  *((uint32_t *)(i)) = bits_per_sample;

  //data subchunk
  i = buf + 36; //subchunk id
  memcpy(i, "data", 4);

  i = buf +40; //size of audio data
  *((uint32_t *)(i)) = data_size;

}


