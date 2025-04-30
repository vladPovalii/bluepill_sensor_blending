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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include <ssd1306_fonts.h>
#include <ssd1306.h>
#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

// for gyro calibration
float gyro_offset_x = 0;
float gyro_offset_y = 0;
float gyro_offset_z = 0;

void Gyro_Calibrate(void)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    const int samples = 500; // How many samples to average (you can tune)

    for (int i = 0; i < samples; i++)
    {
        MPU6050_Read_All(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        HAL_Delay(2); // Small delay to let sensor stabilize (~2 ms per sample)
    }

    gyro_offset_x = (float)sum_x / samples;
    gyro_offset_y = (float)sum_y / samples;
    gyro_offset_z = (float)sum_z / samples;
}

typedef struct {
    int8_t x_axis;
    int8_t y_axis;
    int8_t z_axis;
} OrientationMatrix;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t ax, ay, az;
int16_t gx, gy, gz;

float alpha = 0.98f;

float pitch, roll, yaw;

char buffer[32];

OrientationMatrix orient = {
    .x_axis = 1,
    .y_axis = -1,
    .z_axis = 1
};

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
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  printf("SSD1306 init done\r\n");

  MPU6050_Init(&hi2c1);
  printf("MPU6050 init done\r\n");

  printf("Starting gyro calibration.Keep the board still!!!\r\n");
  Gyro_Calibrate();
  printf("Gyro calibration ended\r\n");

  uint32_t lastTick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Read data from MPU6050
    MPU6050_Read_All(&hi2c1, &ax, &ay, &az, &gx, &gy, &gz);

    // Apply Orientation Correction (sensor rotated 90 deg, x and y swapped)
    int16_t ax_adj = ax * orient.x_axis;
    int16_t ay_adj = ay * orient.y_axis;
    int16_t az_adj = az * orient.z_axis;

    int16_t gx_adj = gx * orient.x_axis;
    int16_t gy_adj = gy * orient.y_axis;
    int16_t gz_adj = gz * orient.z_axis;

    float gx_dps = ((float)gx_adj - gyro_offset_x) / 131.0f;
    float gy_dps = ((float)gy_adj - gyro_offset_y) / 131.0f;
    float gz_dps = ((float)gz_adj - gyro_offset_z) / 131.0f;

    // Time difference
    uint32_t now = HAL_GetTick();
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;

    // Accel angles
    float acc_pitch = atan2((float)ax_adj, sqrt((float)(ay_adj * ay_adj + az_adj * az_adj))) * (180.0f / M_PI);
    float acc_roll  = atan2((float)ay_adj, sqrt((float)(ax_adj * ax_adj + az_adj * az_adj))) * (180.0f / M_PI);

    // Complementary Filter
//    pitch = alpha * (pitch + gx_dps * dt) + (1.0f - alpha) * acc_pitch;
//    roll  = alpha * (roll  + gy_dps * dt) + (1.0f - alpha) * acc_roll;
//    yaw += gz_dps * dt;
    pitch = acc_pitch;
	roll  = acc_roll;
	yaw += gz_dps * dt;


    // Keep yaw in 0-360 range
    if (yaw >= 360.0f) yaw -= 360.0f;
    if (yaw < 0.0f) yaw += 360.0f;

    // Set cursor to top-left and clear display
	ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);

    // Format and display pitch (also send data to serial terminal)
    sprintf(buffer, "Pitch: %.2f", pitch);
    ssd1306_WriteString(buffer, Font_7x10, White);
    printf("Pitch: %.2f", pitch);

    // Move cursor lower (next line) and display roll (also send data to serial terminal)
    ssd1306_SetCursor(0, 12);
    sprintf(buffer, "Roll: %.2f", roll);
    ssd1306_WriteString(buffer, Font_7x10, White);
    printf("Pitch: %.2f", roll);

    ssd1306_SetCursor(0, 24);
    sprintf(buffer, "Yaw: %.2f", yaw);
    ssd1306_WriteString(buffer, Font_7x10, White);

    // Normalize Roll value to screen position
    // Let's say Roll goes from -45° to +45°
    float roll_clamped = roll;
    if (roll_clamped > 45) roll_clamped = 45;
    if (roll_clamped < -45) roll_clamped = -45;
    uint8_t roll_pos = (uint8_t)((roll_clamped + 45.0f) * (127.0f / 90.0f));

    float pitch_clamped = pitch;
    if (pitch_clamped > 45) pitch_clamped = 45;
    if (pitch_clamped < -45) pitch_clamped = -45;
    uint8_t pitch_pos = (uint8_t)((pitch_clamped + 45.0f) * (63.0f / 90.0f));

    // Draw tilt indicators
    ssd1306_Line(64, 63, roll_pos, 50, White); // Roll bar (horizontal)
    ssd1306_Line(64, 32, 64, pitch_pos, White); // Pitch bar (vertical)


    // 6. Update the screen with new content
    ssd1306_UpdateScreen();

    // 7. Delay before next update
    HAL_Delay(20);

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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

#ifdef  USE_FULL_ASSERT
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
