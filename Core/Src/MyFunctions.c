/*
 * MyFunctions.c
 *
 *  Created on: 23 Mar 2020
 *      Author: 18407420
 */

/* USER CODE BEGIN Includes */
#include "MyFunctions.h"
#include "main.h"
#include "sinewave.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN PV */

enum buttonstate{Released, Pressed};

volatile uint8_t Record_button;
volatile uint8_t button1 = Released;
volatile uint8_t button2 = Released;
volatile uint8_t button3 = Released;
volatile uint8_t Stop_button = Released;
extern volatile uint8_t buttn;
volatile uint8_t StartTick, CurrentTick = 0;

volatile uint8_t counter;
uint8_t debounce = 0;
extern AUDIO_ModeType state;

UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
DAC_HandleTypeDef hdac;
ADC_HandleTypeDef hadc2;


void led_flicker(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){

		counter = 0;
	    while((counter < 80) && (Stop_button == Released))  // Record for 20 sec max & Continously check if the Stop button is released else stop recording
		 {

			if((counter & 1) == 1){
				HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET); // LED x flashes at 250 ms ON and OFF.
			}

			else {
				HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
			}
		 }

	      reset_ever();
}


 void uart_transmit_record(uint8_t record){

	 uint8_t recd1 [10] = {127, 128, 'R', 'e', 'c', 'o', 'r', 'd', '_', '1'};
	 uint8_t recd2 [10] = {127, 128, 'R', 'e', 'c', 'o', 'r', 'd', '_', '2'};
	 uint8_t recd3 [10] = {127, 128, 'R', 'e', 'c', 'o', 'r', 'd', '_', '3'};

	 switch(record){

	 case 1:

		 HAL_UART_Transmit(&huart2, recd1, sizeof(recd1), 1000);
		 HAL_ADC_Start_DMA(&hadc2, (uint32_t*)myADC_Signal_1, FULLBUFFERSZE);
		 //HAL_UART_Transmit_DMA(&huart2, myADC_Signal_1, sizeof(myADC_Signal_1), 1000);
		 break;

	 case 2:

		 HAL_UART_Transmit(&huart2, recd2, sizeof(recd2), 1000);
		 HAL_ADC_Start_DMA(&hadc2, (uint32_t*)myADC_Signal_2, FULLBUFFERSZE);
		 //HAL_UART_Transmit_DMA(&huart2, myADC_Signal_1, sizeof(myADC_Signal_1), 1000);
		 break;

	 case 3:

		 HAL_UART_Transmit(&huart2, recd3, sizeof(recd3), 1000);
		 HAL_ADC_Start_DMA(&hadc2, (uint32_t*)myADC_Signal_3, FULLBUFFERSZE);
		 //HAL_UART_Transmit_DMA(&huart2, myADC_Signal_3, sizeof(myADC_Signal_3), 1000);
		 break;
	 }
 }


 void uart_transmit_playback(uint8_t play){

	 uint8_t playb1 [10] = {127, 128, 'P', 'l', 'a', 'y', '_', '_', '_', '1'};
	 uint8_t playb2 [10] = {127, 128, 'P', 'l', 'a', 'y', '_', '_', '_', '2'};
	 uint8_t playb3 [10] = {127, 128, 'P', 'l', 'a', 'y', '_', '_', '_', '3'};

	 switch(play){

	 case 1:
		  //wave_fillbuffer(myDAC_Signal, buttn, FULLBUFFERSZE);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)myDAC_Signal_1, FULLBUFFERSZE, DAC_ALIGN_12B_R);
		  HAL_UART_Transmit(&huart2, playb1, sizeof(playb1), 1000);
		 break;

	 case 2:
		  //wave_fillbuffer(myDAC_Signal, buttn, FULLBUFFERSZE);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)myDAC_Signal_2, FULLBUFFERSZE, DAC_ALIGN_12B_R);
		  HAL_UART_Transmit(&huart2, playb2, sizeof(playb2), 1000);
		 break;

	 case 3:
		  //wave_fillbuffer(myDAC_Signal, buttn, FULLBUFFERSZE);
		  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)myDAC_Signal_3, FULLBUFFERSZE, DAC_ALIGN_12B_R);
		  HAL_UART_Transmit(&huart2, playb3, sizeof(playb3), 1000);
		 break;
	 }
 }

 //Reset everything and return to the main function
  void reset_ever(void){

 		 uint8_t stop [10] = {127, 128, 'S', 't', 'o', 'p', '_', '_', '_', '_'};
 		 HAL_UART_Transmit(&huart2, stop, sizeof(stop), 1000);

   	 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
   	 		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
   	 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
   	 		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

   	 		Record_button = Released;
   	 		button1 = Released;
   	 		button2 = Released;
   	 		button3 = Released;
   	 		Stop_button = Released;

   	        state = AUDIO_MODE_IDLE;
   }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){//Called Every button interrupt

	//Delay the trigerring of the interrupt for 10ms to account for mechanical bounce:
	if(debounce <= 0){

		switch(GPIO_Pin){

		case GPIO_PIN_8: //btn 1
			break;

		case GPIO_PIN_6: //btn 2
			break;

		case GPIO_PIN_5: //btn 3
			break;

		case GPIO_PIN_10: //Record
		    break;

		case GPIO_PIN_9: //Stop
		    break;
		}

		debounce = 10;
	}


		if(GPIO_Pin == GPIO_PIN_8){

			button1 = Pressed;
		}

		else if(GPIO_Pin == GPIO_PIN_6){

			button2 = Pressed;
		}

		else if(GPIO_Pin == GPIO_PIN_5){

			button3 = Pressed;
		}

		else if(GPIO_Pin == GPIO_PIN_9){

			Stop_button = Pressed;
		}

		else if((GPIO_Pin == GPIO_PIN_10) && (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 1)){

			Record_button = Pressed;
		}

}

void HAL_SYSTICK_Callback(void){ //Called Every 1ms

	if(debounce > 0){
		debounce--;
	}
}

void rec_bttncb(void){ //Check every 1ms if record is ON PRESS and HOLD


	//StartTick = CurrentTick;
	if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 1)){ // && ((CurrentTick - StartTick) > PB_PRESSED_TIMEOUT)){

		Record_button = Pressed;

	}

	else if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == 0)){

		Record_button = Released;

	}
}
