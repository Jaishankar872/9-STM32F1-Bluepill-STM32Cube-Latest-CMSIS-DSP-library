/**
 * This Code is for STM32F103C8TB
 */

#include "main.h"
#include <stdio.h>
#include "arm_math.h"

#define led_pin GPIO_PIN_13

void setup_LED();
void uart_init();
void SystemClock_Config(void);
UART_HandleTypeDef huart1 = {0}; // Not USART_HandleTypeDef only UART_HandleTypeDef
void Error_Handler(void);

int16_t data_set[] = {1234, 5678, 9876, 4321, 3456, 7890, 6543, 2109, 8765, 1234, 5678, 9012, 2345, 6789, 4321, 8765, 1098, 3456, 7890, 6543};
int16_t array_size = 19;

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
    printf("Calling the CMSIS DSP Library \n");
    printf("Array Size = %d \n Calculated \n-> Maximum & Minimum out of array \n", array_size);
    for (int i = 0; i < array_size; i++)
    {
      printf("%d) %d\n", i + 1, data_set[i]);
      HAL_Delay(20);
    }
    /**
     * CMSIS-DSP Functions for Different Data Types:
      * Floating-point (float32_t): arm_max_f32()
      * Q31 (32-bit signed fixed-point): arm_max_q31()
      * Q15 (16-bit signed fixed-point): arm_max_q15()
      * Q7 (8-bit signed fixed-point): arm_max_q7()

      Since you're working with int16_t, you should use arm_max_q15().
      Document reference link: https://arm-software.github.io/CMSIS-DSP/latest/group__Max.html
     */
    int16_t _max_cmsis = 0;
    uint32_t _max_cmsis_index = -1;
    int16_t _min_cmsis = 0;
    uint32_t _min_cmsis_index = -1;
    arm_max_q15(data_set, array_size, &_max_cmsis, &_max_cmsis_index);
    arm_min_q15(data_set, array_size, &_min_cmsis, &_min_cmsis_index);
    printf("Maximum Value: %d at index: %ld \n", _max_cmsis, _max_cmsis_index);
    printf("Minimum Value: %d at index: %ld \n", _min_cmsis, _min_cmsis_index);

    // arm_rfft_init_32_q15(&fft_handle, 0, 1);
    // Data Types used here: q15 <=> int16_t
    // fft_handle.fftLenReal = fft_buffer_size;
    // arm_rfft_q15(&fft_handle, AFC_adc_Volt_data, fft_Buf_out);
    // arm_cmplx_mag_q15(fft_Buf_out, fft_Buf_out, fft_buffer_size); // Compute magnitude

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