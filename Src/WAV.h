#ifndef _WAV_H_
#define _WAV_H_


#ifdef DEBUG
#include <__cross_studio_io.h>
#endif
 
#include "stm32f7xx_hal.h"
//#include "usbd_core.h"
//#include "stm32f7xx_nucleo_144.h"

void WAV_HeaderGen(uint8_t *buf, int num_samples, int sample_rate, int bits_per_sample);


#endif