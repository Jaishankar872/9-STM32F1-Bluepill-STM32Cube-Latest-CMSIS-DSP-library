/**
 * This Code is for STM32F103C8TB
 * FFT Library Reference: https://gist.github.com/Tomwi/3842231
 */

#include "main.h"
#include <stdio.h>

#include <math.h>
#include <stdint.h>
#include "fix_fft.h"

#define led_pin GPIO_PIN_13

void setup_LED();
void uart_init();
void SystemClock_Config(void);
UART_HandleTypeDef huart1 = {0}; // Not USART_HandleTypeDef only UART_HandleTypeDef
void Error_Handler(void);

#define FFT_SAMP (6)             // 2^6=64;
#define FFT_SIZE (1 << FFT_SAMP) // FFT size (e.g., 64)

int16_t inBuf[FFT_SIZE * 2]; // Buffer for real and imaginary parts

int16_t data_set[] = {2247,2594,2863,3084,3358,3524,3709,3762,3752,3799,3661,3565,3378,3094,2890,2577,2330,1997,1674,1462,1196,1050,887,761,781,767,904,1015,1160,1448,1671,2000,2276,2528,2877,3102,3368,3567,3653,3781,3780,3786,3683,3513,3379,3108,2894,2611,2265,2018,1693,1472,1225,989,907,782,794,794,840,1033,1185,1451,2385,2493};
int16_t array_size = 64;

// Redirect printf to UART
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 1000);
  return len;
}

int main(void)
{
  SystemClock_Config();
  HAL_Init();
  setup_LED();
  uart_init();
  while (1)
  {

    printf("\n-----------------------------\n");
    printf("Calling the  Library \n");
    // Duplicate the data
    for (int i = 0; i < FFT_SIZE; i++)
    {
        inBuf[2 * i] = data_set[i]; // Read real part from ADC
        inBuf[2 * i + 1] = 0;            // Imaginary part is 0 for real-valued input
    }
    // Perform forward FFT
    fix_fftr(inBuf, FFT_SAMP, 0);

    // Compute magnitude of each frequency bin
    int16_t magnitude[FFT_SIZE / 2];
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        int16_t real = inBuf[2 * i];
        int16_t imag = inBuf[2 * i + 1];
        magnitude[i] = (int16_t)sqrt((real * real) + (imag * imag));
    }

    printf("Array Size = %d \n Calculated \n", array_size);
    for (int i = 0; i < FFT_SIZE / 2; i++)
    {
      printf("%d) %d\n", i + 1, magnitude[i]);
      HAL_Delay(20);
    }

    printf("Task Completed......\n");

    HAL_Delay(5000);
    HAL_GPIO_TogglePin(GPIOC, led_pin);
  }
}

void uart_init()
{
  /**
   * USART Details of STM32F103C8TB - Blue Pill - 128KB Flash
   * PA9 - USART1_TX
   * PA10 - USART1_RX
   */
  // Enable clock for GPIOA and USART1
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  // Configure USART1 TX
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  // GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Configure UART(not USART)
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  HAL_UART_Init(&huart1);
}

void setup_LED()
{

  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = led_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}