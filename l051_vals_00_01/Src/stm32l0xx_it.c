/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim21;


/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

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
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt and DAC1/DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}





extern int usec_timer_flag;

extern  int automat_state;
extern int odd_even;

extern int charge_packet_counter;
extern int positive_impulse_counter;
extern int negative_impulse_counter;
extern int discharge_counter;

extern uint32_t delay_counter;
extern uint32_t chock_length_counter;


extern int DELAY_1_MS;
extern int NUMBER_OF_CHARGE_PULSES;
extern double MODIFIED_NUMBER_OF_CHARGE_PULSES;
extern int DELAY_LENGTH; 
extern int CHOCK_LENGTH;
extern int DISCHARGE_IMPULSE_LENGTH;
extern double INCREMENT;
extern int FORM;

extern int usart_string_received_flag;

/**
* @brief This function handles TIM21 global interrupt.
*/
void TIM21_IRQHandler(void)
{
	// clear it
	(&htim21)->Instance->SR = ~TIM_IT_UPDATE;

	if(automat_state == 1)
	{

		if(odd_even)
		{
			if((charge_packet_counter == 0))
			{
				if((FORM == 2) && (chock_length_counter > (CHOCK_LENGTH/2)))
					INCREMENT = -INCREMENT;

				if(FORM != 0)
				{
					MODIFIED_NUMBER_OF_CHARGE_PULSES += INCREMENT;
					NUMBER_OF_CHARGE_PULSES = (int)MODIFIED_NUMBER_OF_CHARGE_PULSES;
				}
			}

			charge_packet_counter++;
		}
		odd_even = (odd_even + 1) % 2;
		usec_gen_out_GPIO_Port->ODR ^= usec_gen_out_Pin;// toggle usec generator pin


		if(charge_packet_counter >= NUMBER_OF_CHARGE_PULSES)
		{
			charge_packet_counter = 0;
			automat_state = 2;
			positive_impulse_counter = 0;
		}
	}
	else if(automat_state == 2)
	{
		if(positive_impulse_counter == 1)
		{
			// set f1 pin
    		pos_pack_gen_out_GPIO_Port->BSRR |= pos_pack_gen_out_Pin ;
            
		}
		if(positive_impulse_counter >= DISCHARGE_IMPULSE_LENGTH)
		{
			// reset f1 pin
    		pos_pack_gen_out_GPIO_Port->BRR |= pos_pack_gen_out_Pin ;
			positive_impulse_counter = 0;
			negative_impulse_counter = 0;
			automat_state = 3;
            
		}
		else
		{
			//increment counter
			positive_impulse_counter++;
		}
	}
	else if(automat_state == 3)
	{
		if(negative_impulse_counter == 1)
		{
			// set f2 pin
    		neg_pack_gen_out_GPIO_Port->BSRR |= neg_pack_gen_out_Pin ;
            
		}
		if(negative_impulse_counter >= DISCHARGE_IMPULSE_LENGTH)
		{
			// reset f2 pin
    		neg_pack_gen_out_GPIO_Port->BRR |= neg_pack_gen_out_Pin ;
			negative_impulse_counter = 0;
			discharge_counter = 0;
			//automat_state = 4;
			//debug
			automat_state = 5;
            
		}
		else
		{
			//increment counter
			negative_impulse_counter++;
		}
	}
	else if(automat_state == 4)
	{
		if(discharge_counter == 1)
		{
			// set f3 pin
    		f3_out_GPIO_Port->BSRR |= f3_out_Pin ;
            
		}
		if(discharge_counter >= 11)
		{
			// reset f3 pin
    		f3_out_GPIO_Port->BRR |= f3_out_Pin ;
			discharge_counter = 0;
			delay_counter = 0;
			automat_state = 5;
            
		}
		else
		{
			//increment counter
			discharge_counter++;
		}
	}
	else if(automat_state == 5)
	{
		if(delay_counter >= (DELAY_1_MS*DELAY_LENGTH))
		{
			delay_counter = 0;
			automat_state = 1;
			chock_length_counter++;
            
		}
		else
		{
			//increment counter
			delay_counter++;
		}
	}

	//*
	if(chock_length_counter >= CHOCK_LENGTH)
	{
		// disable tim21 interrupt
    	TIM21->DIER &= ~TIM_DIER_UIE;
		
		chock_length_counter = 0;
		automat_state = 0;
		MODIFIED_NUMBER_OF_CHARGE_PULSES = 0;
	}
	//*/

}

extern int usec_timer_flag;
void __HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// set flag
	usec_timer_flag = 1;
}



extern int usart_rxne_flag;
extern char usart_buffer[256];
extern int usart_buffer_index;

void USART1_IRQHandler(void)
{
	uint32_t isrflags   = USART1->ISR;
	uint32_t cr1its     = USART1->CR1;
	uint32_t cr3its;
	uint32_t errorflags;

	uint16_t usart_data;

	/* If no error occurs */
	errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
	if (errorflags == RESET)
	{
    	/* UART in mode Receiver ---------------------------------------------------*/
    	if(((isrflags & USART_ISR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    	{
      		//UART_Receive_IT(huart);
      		//return;
    		usart_data = (uint16_t) USART1->RDR;
			//usart_rxne_flag = 1;

			usart_buffer[usart_buffer_index] = (char)usart_data;
			usart_buffer_index++;

			if((char)usart_data == '\n')
			{

				usart_buffer[usart_buffer_index] = 0;
				usart_string_received_flag = 1;
				usart_buffer_index = 0;

			}
    	}
  }  
}
