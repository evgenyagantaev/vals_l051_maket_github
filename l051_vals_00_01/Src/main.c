
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "ssd1306.h"

/* Private variables ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

int DELAY_1_MS = 50;
int NUMBER_OF_CHARGE_PULSES = 5;
int DELAY_LENGTH = 11;
int CHOCK_LENGTH = 16;
int DISCHARGE_IMPULSE_LENGTH = 11;
int FORM = 2;
double INCREMENT = 1;
double MODIFIED_NUMBER_OF_CHARGE_PULSES = 0;

int usec_timer_flag = 0;

int automat_state = 0;
int odd_even = 0;

int charge_packet_counter = 0;
int positive_impulse_counter = 0;
int negative_impulse_counter = 0;
int discharge_counter = 0;

uint32_t delay_counter = 0;
uint32_t chock_length_counter = 0;

int usart_rxne_flag = 0;
int usart_string_received_flag = 0;

char message[64];

char usart_buffer[256];
int usart_buffer_index = 0;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void usec_timer_action();                                                                                    
/* Private function prototypes -----------------------------------------------*/


int main(void)
{


	/* MCU Configuration----------------------------------------------------------*/ 
                                                                                    
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
                                                                                    
                                                                                    
    /* Configure the system clock */
    SystemClock_Config();
                                                                                    
                                                                                    
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_TIM6_Init();
    MX_TIM22_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_ADC_Init();
                                                                                    
    ssd1306_set_i2c_port(&hi2c1, 1);
  	ssd1306_Init();
  	HAL_Delay(100);

  	ssd1306_Fill(White);
  	ssd1306_UpdateScreen();
  	HAL_Delay(100);
  	ssd1306_Fill(Black);
  	ssd1306_UpdateScreen();
  	HAL_Delay(100);

  	ssd1306_SetCursor(0,0);
  	ssd1306_WriteString("VALS", Font_16x26, White);
  	//ssd1306_SetCursor(0,30);
  	//ssd1306_WriteString("Start..", Font_16x26, White);
  	ssd1306_UpdateScreen();
  	HAL_Delay(100);



	
  	HAL_GPIO_WritePin(led_out_GPIO_Port, led_out_Pin, GPIO_PIN_RESET);// turn led on

	/* Disable SysTick Interrupt */
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

    MX_TIM21_Init();
	HAL_TIM_Base_Start(&htim21);
    TIM21->DIER |= TIM_DIER_UIE;

	// debug
	strcpy(usart_buffer, "e1c00k005l0160d00n0005p00000m000f0\r\n");

	automat_state = 1;
    /* Infinite loop */
    while (1)
    {
		// Disable SysTick Interrupt 
		//SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
		// enable tim21 interrupt
    	//TIM21->DIER |= TIM_DIER_UIE;

		//while(automat_state != 0);

		// disable tim21 interrupt
    	//TIM21->DIER &= ~TIM_DIER_UIE;
		// Enable SysTick Interrupt
		//SysTick->CTRL = SysTick_CTRL_TICKINT_Msk;
  		//HAL_Delay(11);
		//automat_state = 1;

		// toggle gpio
  		//usec_gen_out_GPIO_Port->ODR ^= usec_gen_out_Pin;// toggle usec generator pin

    	//USART1->CR1 |= USART_CR1_RXNEIE;
    	//USART1->CR1 &= ~USART_CR1_RXNEIE;

		//while(((USART1->ISR) & UART_FLAG_RXNE) == RESET);
		//HAL_UART_Transmit(&huart1, message, strlen((const char *)message), 500);
		//*
		//if(automat_state == 0)
		//if(usart_rxne_flag)


		// debug
		usart_string_received_flag = 0;

		if(usart_string_received_flag)                                                                                 
		{
			int on_off_flag = 0;
			int channel = 1;
			int aux;
			// FORM 0 - flat, 1 - ascend, 2 - triangle

			usart_string_received_flag = 0;


			if(usart_buffer[0] == 'e')
			{

				sscanf(usart_buffer, "e%1dc%2dk%3dl%4dd%2dn%4dp%5dm%3df%1d\r\n",                                                   	
			           &on_off_flag, &channel, &NUMBER_OF_CHARGE_PULSES, &DISCHARGE_IMPULSE_LENGTH,
			    	   &DELAY_LENGTH, &CHOCK_LENGTH, &aux, &aux, &FORM);
			    DISCHARGE_IMPULSE_LENGTH = DISCHARGE_IMPULSE_LENGTH/20 + 1;
			    usart_buffer_index = 0;
                                                                                                                               
			    uint8_t channel_address = channel;
                                                                                                                               
			    if(on_off_flag)
			    {
			    	// set address
			    	/*
			    	if(channel == 0)
			    	{
  			    		HAL_GPIO_WritePin(GPIOB, addr0_out_Pin|addr1_out_Pin|addr2_out_Pin 
                              |addr3_out_Pin, GPIO_PIN_RESET);
			    		
			    	}
			    	else if(channel == 1)
			    	{
  			    		HAL_GPIO_WritePin(GPIOB, addr0_out_Pin|addr1_out_Pin|addr2_out_Pin 
                              |addr3_out_Pin, GPIO_PIN_RESET);
  			    		HAL_GPIO_WritePin(GPIOB, addr0_out_Pin, GPIO_PIN_SET);
			    		
			    	}
			    	else if(channel == 2)
			    	{
  			    		HAL_GPIO_WritePin(GPIOB, addr0_out_Pin|addr1_out_Pin|addr2_out_Pin 
                              |addr3_out_Pin, GPIO_PIN_RESET);
  			    		HAL_GPIO_WritePin(GPIOB, addr1_out_Pin, GPIO_PIN_SET);
			    		
			    	}
			    	*/
                                                                                                                               
			    	uint8_t addr0, addr1, addr2, addr3;
                                                                                                                               
			    	addr0 = channel_address & 0x01;
			    	addr1 = (channel_address & 0x02) >> 1;
			    	addr2 = (channel_address & 0x04) >> 2;
			    	addr3 = (channel_address & 0x08) >> 3;
  			    	
			    	HAL_GPIO_WritePin(GPIOB, addr0_out_Pin|addr1_out_Pin|addr2_out_Pin 
                                            |addr3_out_Pin, GPIO_PIN_RESET);
                                                                                                                               
			    	if(addr0)
  			    		HAL_GPIO_WritePin(GPIOB, addr0_out_Pin, GPIO_PIN_SET);
			    	if(addr1)
  			    		HAL_GPIO_WritePin(GPIOB, addr1_out_Pin, GPIO_PIN_SET);
			    	if(addr2)
  			    		HAL_GPIO_WritePin(GPIOB, addr2_out_Pin, GPIO_PIN_SET);
			    	if(addr3)
  			    		HAL_GPIO_WritePin(GPIOB, addr3_out_Pin, GPIO_PIN_SET);


					// calculate waveform parameters
					if(FORM == 1) // ascending
						INCREMENT = (double)((double)NUMBER_OF_CHARGE_PULSES / (double)CHOCK_LENGTH);
					else if(FORM == 2)  // triangle
						INCREMENT = (double)((double)NUMBER_OF_CHARGE_PULSES / ((double)CHOCK_LENGTH / 2.0));

			    	// start generator
    		    	MX_TIM21_Init();
			    	HAL_TIM_Base_Start(&htim21);
                                                                                                                               
			    	automat_state = 1;
			    	chock_length_counter = 0;
			    	// enable tim21 interrupt
    		    	TIM21->DIER |= TIM_DIER_UIE;
			    }// end if(on_off_flag)

			}
			else if(usart_buffer[0] == 'v')
			{

			    int NUMBER_OF_PULSES = 1; 
				int LENGTH_OF_PULSE = 1;
				sscanf(usart_buffer, "v%1dc%2dn%3dl%5dd%5d\r\n", //   v1c00n001l00001d00003\r\n                             	
			           &on_off_flag, &channel, &NUMBER_OF_PULSES, &LENGTH_OF_PULSE, &DELAY_LENGTH);


			    if(on_off_flag)
			    {

					// Enable SysTick Interrupt
					SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

					int i;
					for(i=0; i<NUMBER_OF_PULSES; i++)
					{
						// turn on vibrator
  			    		HAL_GPIO_WritePin(GPIOA, aux_out_Pin, GPIO_PIN_SET);
  						HAL_Delay(LENGTH_OF_PULSE);
						// turn off vibrator
  			    		HAL_GPIO_WritePin(GPIOA, aux_out_Pin, GPIO_PIN_RESET);
  						
						//delay
						HAL_Delay(DELAY_LENGTH);

					}

					// Disable SysTick Interrupt 
					SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
				}
			}


		}// end if(usart_string_received_flag)



    }// end while (1)

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/**
  * @}
  */

/**
  * @}
  */


int usec_timer_counter = 0;
void usec_timer_action()
{
	if(usec_timer_flag)
	{
		// drop flag
		usec_timer_flag = 0;
		// increment counter
		usec_timer_counter++;
		// check counter value
		if(usec_timer_counter > 1)
		{
			// toggle gpio
  			usec_gen_out_GPIO_Port->ODR ^= usec_gen_out_Pin;// toggle usec generator pin
			usec_timer_counter = 0;

		}
	}

}
