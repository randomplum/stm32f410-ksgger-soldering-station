/**
 ******************************************************************************
 * @file    app/src/main.c
 * @author  MCD Application Team
 * @version V1.5.0
 * @date    14-April-2017
 * @brief
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *notice, this list of conditions and the following disclaimer in the
 *documentation and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef UartHandle;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

void Pinmux_Config(void)
{
	GPIO_InitTypeDef gpio_config;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	USARTx_TX_GPIO_CLK_ENABLE();

	gpio_config.Pull = GPIO_NOPULL;
	gpio_config.Speed = GPIO_SPEED_FREQ_HIGH;

	/* UART Begin */
	gpio_config.Mode = GPIO_MODE_AF_PP;
	gpio_config.Pin = USARTx_TX_PIN;
	gpio_config.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &gpio_config);
	gpio_config.Mode = GPIO_MODE_AF_PP;
	gpio_config.Pin = USARTx_RX_PIN;
	gpio_config.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &gpio_config);
	/* UART End */

	gpio_config.Mode = GPIO_MODE_OUTPUT_PP;
	/* Motor 1 */
#define MOTOR2_PWM GPIO_PIN_4
#define MOTOR2_IN1 GPIO_PIN_15
#define MOTOR2_IN2 GPIO_PIN_14
#define MOTOR2_GPIO_PORT GPIOB
	gpio_config.Pin = MOTOR2_PWM;
	HAL_GPIO_Init(MOTOR2_GPIO_PORT, &gpio_config);
	HAL_GPIO_WritePin(MOTOR2_GPIO_PORT, MOTOR2_PWM, GPIO_PIN_SET);
	gpio_config.Pin = MOTOR2_IN1;
	HAL_GPIO_Init(MOTOR2_GPIO_PORT, &gpio_config);
	HAL_GPIO_WritePin(MOTOR2_GPIO_PORT, MOTOR2_IN1, GPIO_PIN_RESET);
	gpio_config.Pin = MOTOR2_IN2;
	HAL_GPIO_Init(MOTOR2_GPIO_PORT, &gpio_config);
	HAL_GPIO_WritePin(MOTOR2_GPIO_PORT, MOTOR2_IN2, GPIO_PIN_SET);

#define ENCODER1_PIN GPIO_PIN_5
#define ENCODER1_PORT GPIOA
	gpio_config.Mode = GPIO_MODE_INPUT;
	gpio_config.Pin = ENCODER1_PIN;
	HAL_GPIO_Init(ENCODER1_PORT, &gpio_config);
}

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void)
{
	HAL_Init();

	/* Configure the system clock to 72 MHz */
	SystemClock_Config();

	/* Configure the pin muxing */
	Pinmux_Config();

	USARTx_CLK_ENABLE()
	UartHandle.Instance = USARTx;
	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&UartHandle) != HAL_OK)
	{
		Error_Handler();
	}
	printf("\n\r");

	printf("\n\rInitialized\n\r");
	/* Infinite loop */
	while (1)
	{
		printf("Hello\r");
		HAL_Delay(1024);
	}
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USARTx and Loop until the end of
	 * transmission */
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);

	return ch;
}

/**
 * @brief  Retargets the C library scanf function to the USART.
 * @param  None
 * @retval None
 */
int __io_getchar(void)
{
	uint8_t ch;
	HAL_UART_Receive(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#ifdef UART_ENABLE_ECHO
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif

	return (char)ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 100000000
  *            HCLK(Hz)                       = 100000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 400
  *            PLL_P                          = 4
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 3
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
static void Error_Handler(void)
{
	while (1)
	{
	}
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line
	   number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
	   line) */

	   printf("ASSERT @ %s:%d\r\n", file, line);

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
