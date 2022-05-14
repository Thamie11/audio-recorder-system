/*
 * MyFunctions.h
 *
 *  Created on: 23 Mar 2020
 *      Author: 18407420
 */

#ifndef SRC_MYFUNCTIONS_H_
#define SRC_MYFUNCTIONS_H_
#include "main.h"

// This is the full DAC buffer size;
#define FULLBUFFERSZE 1024
#define DATASIZE 512
#define PB_PRESSED_TIMEOUT                   ((uint8_t) 10)

/* USER CODE BEGIN PFP */
void led_flicker(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void reset_ever(void);
void uart_transmit_record(uint8_t record);
void uart_transmit_playback(uint8_t play);
void rec_bttncb(void);


/* USER CODE BEGIN PV */
typedef enum AUDIO_Mode
{
  AUDIO_MODE_IDLE,
  AUDIO_MODE_RECORDING_1,
  AUDIO_MODE_RECORDING_2,
  AUDIO_MODE_RECORDING_3,
  AUDIO_MODE_PLAYING_1,
  AUDIO_MODE_PLAYING_2,
  AUDIO_MODE_PLAYING_3,
} AUDIO_ModeType;

uint8_t myADC_Signal_1[FULLBUFFERSZE];
uint8_t myADC_Signal_2[FULLBUFFERSZE];
uint8_t myADC_Signal_3[FULLBUFFERSZE];

uint8_t myDAC_Signal_1[FULLBUFFERSZE];
uint8_t myDAC_Signal_2[FULLBUFFERSZE];
uint8_t myDAC_Signal_3[FULLBUFFERSZE];

#endif /* SRC_MYFUNCTIONS_H_ */
