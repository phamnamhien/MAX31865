/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max31865.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RREF_VALUE      430.0f      // 430 ohm for PT100
#define RNOMINAL_VALUE  100.0f      // PT100 nominal resistance
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
MAX31865_Handle_t hmax31865;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Platform Integration Functions for MAX31865 */
max31865_status_t stm32_spi_write_read(MAX31865_Handle_t *hmax, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
void stm32_cs_low(MAX31865_Handle_t *hmax);
void stm32_cs_high(MAX31865_Handle_t *hmax);
void stm32_delay_ms(uint32_t ms);

/* Application Functions */
void MAX31865_Test(void);
int _write(int file, char *ptr, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief STM32 HAL SPI Write/Read function for MAX31865
 */
max31865_status_t stm32_spi_write_read(MAX31865_Handle_t *hmax, uint8_t *tx_data, uint8_t *rx_data, uint16_t size) {
    HAL_StatusTypeDef hal_status;

    if (size == 1) {
        // Write only (register address)
        hal_status = HAL_SPI_Transmit(&hspi1, tx_data, 1, HAL_MAX_DELAY);
        if (hal_status != HAL_OK) return MAX31865_TIMEOUT;

        // Read response
        hal_status = HAL_SPI_Receive(&hspi1, rx_data, 1, HAL_MAX_DELAY);
        if (hal_status != HAL_OK) return MAX31865_TIMEOUT;
    } else {
        // TransmitReceive for multi-byte operations
        hal_status = HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, size, HAL_MAX_DELAY);
        if (hal_status != HAL_OK) return MAX31865_TIMEOUT;
    }

    return MAX31865_OK;
}

/**
 * @brief Set CS pin low
 */
void stm32_cs_low(MAX31865_Handle_t *hmax) {
    HAL_GPIO_WritePin((GPIO_TypeDef*)hmax->cs_port, hmax->cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief Set CS pin high
 */
void stm32_cs_high(MAX31865_Handle_t *hmax) {
    HAL_GPIO_WritePin((GPIO_TypeDef*)hmax->cs_port, hmax->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief Delay function in milliseconds
 */
void stm32_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

/**
 * @brief Redirect printf to UART
 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/**
 * @brief Test MAX31865 functionality
 */
void MAX31865_Test(void) {
    max31865_status_t status;
    float temperature, resistance;
    uint16_t rtd_raw;
    uint8_t fault_status;

    printf("\r\n=== MAX31865 RTD Temperature Sensor Test ===\r\n");

    /* Initialize Platform Interface structure */
    max31865_platform_t platform = {
        .spi_write_read = stm32_spi_write_read,
        .cs_low = stm32_cs_low,
        .cs_high = stm32_cs_high,
        .delay_ms = stm32_delay_ms,
        .platform_data = NULL
    };

    /* Initialize MAX31865 */
    status = MAX31865_Init(&hmax31865, &platform, GPIOA, GPIO_PIN_4,
                          MAX31865_3WIRE, RREF_VALUE, RNOMINAL_VALUE);

    if (status != MAX31865_OK) {
        printf("ERROR: MAX31865 initialization failed! Status: %d\r\n", status);
        return;
    }

    printf("MAX31865 initialized successfully!\r\n");
    printf("Configuration: 3-wire, RREF=%.1f ohm, RNOMINAL=%.1f ohm\r\n",
           RREF_VALUE, RNOMINAL_VALUE);

    /* Set 60Hz filter */
    MAX31865_SetFilter(&hmax31865, MAX31865_FILTER_60HZ);
    printf("Filter set to 60Hz\r\n\r\n");

    /* Main measurement loop */
    for (int i = 0; i < 10; i++) {
        printf("--- Measurement %d ---\r\n", i + 1);

        /* Read raw RTD value */
        status = MAX31865_ReadRTD(&hmax31865, &rtd_raw);
        if (status == MAX31865_OK) {
            printf("RTD Raw Value: %u (0x%04X)\r\n", rtd_raw, rtd_raw);
        } else if (status == MAX31865_FAULT) {
            printf("RTD Fault detected! Raw: %u\r\n", rtd_raw);
        } else {
            printf("ERROR: Failed to read RTD! Status: %d\r\n", status);
            HAL_Delay(1000);
            continue;
        }

        /* Read resistance */
        status = MAX31865_ReadResistance(&hmax31865, &resistance);
        if (status == MAX31865_OK) {
            printf("Resistance: %.3f ohm\r\n", resistance);
        } else {
            printf("ERROR: Failed to read resistance! Status: %d\r\n", status);
        }

        /* Read temperature */
        status = MAX31865_ReadTemperature(&hmax31865, &temperature);
        if (status == MAX31865_OK) {
            printf("Temperature: %.2f C\r\n", temperature);
        } else {
            printf("ERROR: Failed to read temperature! Status: %d\r\n", status);
        }

        /* Check for faults */
        status = MAX31865_ReadFault(&hmax31865, &fault_status);
        if (status == MAX31865_OK) {
            if (fault_status != 0) {
                printf("FAULT STATUS: 0x%02X - %s\r\n",
                       fault_status, MAX31865_GetFaultString(fault_status));

                /* Clear faults */
                MAX31865_ClearFault(&hmax31865);
                printf("Faults cleared.\r\n");
            } else {
                printf("No faults detected.\r\n");
            }
        } else {
            printf("ERROR: Failed to read fault status! Status: %d\r\n", status);
        }

        printf("\r\n");
        HAL_Delay(2000); // Wait 2 seconds between measurements
    }

    printf("=== Test Complete ===\r\n\r\n");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n\r\n=== STM32F103C8T6 MAX31865 Test ===\r\n");
  printf("System Clock: %lu Hz\r\n", HAL_RCC_GetHCLKFreq());
  printf("Compiled: %s %s\r\n\r\n", __DATE__, __TIME__);

  /* Run MAX31865 test */
  MAX31865_Test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    /* Continuous temperature monitoring */
	    float temperature;
	    max31865_status_t status;

	    status = MAX31865_ReadTemperature(&hmax31865, &temperature);
	    if (status == MAX31865_OK) {
	        printf("Current Temperature: %.2f C\r\n", temperature);
	    } else {
	        printf("Temperature read error: %d\r\n", status);
	    }
	    HAL_Delay(5000); // Update every 5 seconds
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
