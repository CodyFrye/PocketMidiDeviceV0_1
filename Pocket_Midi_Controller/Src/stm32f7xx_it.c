/**
  ******************************************************************************
  * @file    stm32f7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#include "stm32f7xx_it.h"

/* USER CODE BEGIN 0 */
#include "defines.h"

bool gateKeeper;
uint8_t dummy;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SAI_HandleTypeDef hsai_BlockA1;
extern TIM_HandleTypeDef htim2;

/******************************************************************************/
/*            Cortex-M7 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line0 interrupt.
*		 > checks which key on the keypad (key matrix) is pressed
*/
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
		uint8_t select = (1<<0);
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */


	  if(gateKeeper != __locked)
	  {
		/* re-initialize to set up loop sequence */
		EXTI->IMR &= ~(7<<0);					//mask EXTI to avoid false triggers
	    GPIOC->MODER = 0;
	    GPIOC->PUPDR = (2<<6)|(2<<8)|(2<<10);	//Keep Values from floating

	    /* loop to find which key is pressed */
	    for(uint8_t i = 0; i < 3; i++)
	    		{
	    		  GPIOC->MODER = (1<<(2*i));
	    		  GPIOC->ODR = (1<<i);
	    		  uint32_t instance = GPIOC->IDR & 0b111000;

	    		  switch(instance)
	    		  {
	    		  case (1<<3):
	    				  select |= (1<<3);
	    				  break;
	    		  case (1<<4):
	    				  select |= (1<<4);
	    				  break;
	    		  case (1<<5):
	    				  select |= (1<<5);
	    				  break;
	    		  default:
	    			  	  //select |= (1<<7); //error flag
	    			  	  break;

	    		  }
	    		}

	    /* key matrix lookup table */
	    switch(select & 0b111111)
	    		  {
	    		  case 0b001001:
	    			  	  TransmitMidi(A4, VELOCITY_MAX);
	    				  break;
	    		  case 0b001010:
	    			  	  TransmitMidi(Bb4, VELOCITY_MAX);
	    				  break;
	    		  case 0b001100:
	    			  	  TransmitMidi(B4, VELOCITY_MAX);
	    				  break;
	    		  case 0b010001:
	    			  	  TransmitMidi(C4, VELOCITY_MAX);
	    				  break;
	    		  case 0b010010:
	    			      TransmitMidi(Db4, VELOCITY_MAX);
	    				  break;
	    		  case 0b010100:
	    			  	  TransmitMidi(D4, VELOCITY_MAX);
	    				  break;
	    		  case 0b100001:
	    			  	  TransmitMidi(Eb4, VELOCITY_MAX);
	    				  break;
	    		  case 0b100010:
	    			  	  TransmitMidi(E4, VELOCITY_MAX);
	    				  break;
	    		  case 0b100100:
	    			  	  TransmitMidi(F4, VELOCITY_MAX);
	    				  break;
	    		  default:
	    				  Transmit('K');
	    				  break;
	    		  }

	      /* return to original initialized states */
	      GPIOC->MODER = (1<<6)|(1<<8)|(1<<10);
	      GPIOC->PUPDR = (2<<0)|(2<<2)|(2<<4);
	      GPIOC->ODR = 0b111000;

	      /* enable hardware de-bounce delay */
	      gateKeeper = __locked;		//enable to avoid entering the interrupt again
	      TIM2->CR1 |= (1<<0); 			//start de-bounce timer
	      EXTI->IMR |= (7<<0);			//un-mask EXTI
	  }


  /* USER CODE END EXTI0_IRQn 1 */
}

/**
* @brief This function handles EXTI line1 interrupt.
* 		 > checks which key on the keypad (key matrix) is pressed
*/
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
		uint8_t select = (1<<1);
  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

	  if(gateKeeper != __locked)
	  {

	  /* re-initialize to set up loop sequence */
	  EXTI->IMR &= ~(7<<0); 				//mask EXTI to avoid false triggers
	  GPIOC->MODER = 0;
	  GPIOC->PUPDR = (2<<6)|(2<<8)|(2<<10); //Keep Values from floating

	  /* loop to find which key is pressed */
	  for(uint8_t i = 0; i < 3; i++)
	  		{

		  	  GPIOC->MODER = (1<<(2*i));
		      GPIOC->ODR = (1<<i);
	  		  uint32_t instance = GPIOC->IDR & 0b111000;

	  		  switch(instance)
	  		  {
	  		  case (1<<3):
	  				  select |= (1<<3);
	  				  break;
	  		  case (1<<4):
	  				  select |= (1<<4);
	  				  break;
	  		  case (1<<5):
	  				  select |= (1<<5);
	  				  break;
	  		  default:
	  			  	  select |= (1<<7); //error flag
	  			  	  break;

	  		  }
	  		}

	  /* key matrix lookup table */
	  switch(select & 0b111111)
	  		  {
	  		  case 0b001001:
	  			  	  TransmitMidi(A4, VELOCITY_MAX);
	  				  break;
	  		  case 0b001010:
	  			  	  TransmitMidi(Bb4, VELOCITY_MAX);
	  				  break;
	  		  case 0b001100:
	  			  	  TransmitMidi(B4, VELOCITY_MAX);
	  				  break;
	  		  case 0b010001:
	  			  	  TransmitMidi(C4, VELOCITY_MAX);
	  				  break;
	  		  case 0b010010:
	  			  	  TransmitMidi(Db4, VELOCITY_MAX);
	  				  break;
	  		  case 0b010100:
	  			  	  TransmitMidi(D4, VELOCITY_MAX);
	  				  break;
	  		  case 0b100001:
	  			  	  TransmitMidi(Eb4, VELOCITY_MAX);
	  				  break;
	  		  case 0b100010:
	  			  	  TransmitMidi(E4, VELOCITY_MAX);
	  				  break;
	  		  case 0b100100:
	  			  	  TransmitMidi(F4, VELOCITY_MAX);
	  				  break;
	  		  default:
	  				  Transmit('K');
	  				  break;
	  		  }
	  }

	  /* return to original initialized states */
	  GPIOC->MODER = (1<<6)|(1<<8)|(1<<10);
	  GPIOC->PUPDR = (2<<0)|(2<<2)|(2<<4);
	  GPIOC->ODR = 0b111000;

	  /* enable hardware de-bounce delay */
	  gateKeeper = __locked;	//enable to avoid entering the interrupt again
	  TIM2->CR1 |= (1<<0); 		//start de-bounce timer
	  EXTI->IMR |= (7<<0);		//un-mask EXTI

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
* @brief This function handles EXTI line2 interrupt.
* 		 > checks which key on the keypad (key matrix) is pressed
*/
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
  uint8_t select = (1<<2);
  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

	  if(gateKeeper != __locked)
	  {

		/* re-initialize to set up loop sequence */
		EXTI->IMR &= ~(7<<0); 					//mask EXTI to avoid false triggers
	    GPIOC->MODER = 0;
	    GPIOC->PUPDR = (2<<6)|(2<<8)|(2<<10);	//Keep Values from floating

	    /* loop to find which key is pressed */
	    for(uint8_t i = 0; i < 3; i++)
	    		{
	    		  GPIOC->MODER = (1<<(2*i));
	    	   	  GPIOC->ODR = (1<<i);
	    		  uint32_t instance = GPIOC->IDR & 0b111000;

	    		  switch(instance)
	    		  {
	    		  case (1<<3):
	    				  select |= (1<<3);
	    				  break;
	    		  case (1<<4):
	    				  select |= (1<<4);
	    				  break;
	    		  case (1<<5):
	    				  select |= (1<<5);
	    				  break;
	    		  default:
	    			  	  select |= (1<<7); //error flag
	    			  	  break;

	    		  }
	    		}

	    /* key matrix lookup table */
	    switch(select & 0b111111)
	    		  {
	    		  case 0b001001:
	    			  	  TransmitMidi(A4, VELOCITY_MAX);
	    				  break;
	    		  case 0b001010:
	    			  	  TransmitMidi(Bb4, VELOCITY_MAX);
	    				  break;
	    		  case 0b001100:
	    			  	  TransmitMidi(B4, VELOCITY_MAX);
	    				  break;
	    		  case 0b010001:
	    			  	  TransmitMidi(C4, VELOCITY_MAX);
	    				  break;
	    		  case 0b010010:
	    			  	  TransmitMidi(Db4, VELOCITY_MAX);
	    				  break;
	    		  case 0b010100:
	    			  	  TransmitMidi(D4, VELOCITY_MAX);
	    				  break;
	    		  case 0b100001:
	    			  	  TransmitMidi(Eb4, VELOCITY_MAX);
	    				  break;
	    		  case 0b100010:
	    			  	  TransmitMidi(E4, VELOCITY_MAX);
	    				  break;
	    		  case 0b100100:
	    			  	  TransmitMidi(F4, VELOCITY_MAX);
	    				  break;
	    		  default:
	    				  Transmit('K');
	    				  break;
	    		  }
	  }

	  /* return to original initialized states */
	  GPIOC->MODER = (1<<6)|(1<<8)|(1<<10);
	  GPIOC->PUPDR = (2<<0)|(2<<2)|(2<<4);
	  GPIOC->ODR = 0b111000;

	  /* enable hardware de-bounce delay */
	  gateKeeper = __locked;	//enable to avoid entering the interrupt again
	  TIM2->CR1 |= (1<<0); 		//start de-bounce timer
	  EXTI->IMR |= (7<<0);		//un-mask EXTI

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
	  GPIOB->ODR ^= (1<<7);
	  gateKeeper = __unlocked;
	  //in one pulse mode so turns off automatically
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles SAI1 global interrupt.
*/
void SAI1_IRQHandler(void)
{
  /* USER CODE BEGIN SAI1_IRQn 0 */

  /* USER CODE END SAI1_IRQn 0 */
  HAL_SAI_IRQHandler(&hsai_BlockA1);
  /* USER CODE BEGIN SAI1_IRQn 1 */
  GPIOB->ODR ^= (1<<7);
  /* USER CODE END SAI1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
