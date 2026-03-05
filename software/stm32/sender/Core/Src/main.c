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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FXOSC		32000000
#define BITRATE		9600

#define RFM_RST		GPIO_PIN_8

#define BASE_PWM		70		// default forward/backward duty cycle
#define RAMP_MIN		50		// minimum speed when decelerating with Ctrl
#define RAMP_MAX		100		// maximum ramp duty cycle
#define RAMP_STEP		5		// duty cycle step per packet when shift held
#define TURN_OFFSET		10		// differential added/subtracted during moving turns
#define INPLACE_PWM		55		// duty cycle for in-place turns
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void rfm_config();
void rfm_select();
void rfm_deselect();
void rfm_reset();

void rfm_write_reg(uint8_t addr, uint8_t byte);
uint8_t rfm_read_reg(uint8_t addr);

void rfm_write_fifo(uint8_t * data, uint8_t len);
void rfm_send(uint8_t * data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  // PA8 (D9) --> SS
  // PA5 (A4) --> SCK
  // PA6 (A5) --> MISO
  // PA7 (A6) --> MOSI

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  rfm_deselect();
  rfm_reset();

  rfm_config();

  // GUI must send 6 bytes per packet:
  // [W, A, S, D, Shift, Ctrl]  (0xA0 = pressed, 0x05 = unpressed)
  uint8_t keys[6];			// [up, left, down, right, shift, ctrl]

  uint8_t left_dir, right_dir;	// 0 - backwards, 1 - forwards
  uint8_t left_pwm, right_pwm;	// (0, 100)

  // Ramp state — persists across loop iterations
  uint8_t ramp_pwm = BASE_PWM;	// current speed level, climbs when shift held

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	// Receive state of WASD + Shift keys from serial buffer
	// GUI must send exactly 5 bytes per update cycle
	HAL_UART_Receive(&huart2, keys, 6, HAL_MAX_DELAY);

	uint8_t w     = keys[0];
	uint8_t a     = keys[1];
	uint8_t s     = keys[2];
	uint8_t d     = keys[3];
	uint8_t shift = keys[4];
	uint8_t ctrl  = keys[5];

	// 0xA0 indicates pressed key, 0x05 indicates unpressed key

	// ----------------------------------------------------------------
	// Determine base direction and starting PWM
	// ----------------------------------------------------------------
	uint8_t moving_forward  = (w == 0xA0 && s == 0x05);
	uint8_t moving_backward = (w == 0x05 && s == 0xA0);
	uint8_t moving          = moving_forward || moving_backward;

	if (moving_forward)
	{
		left_dir = right_dir = 1;
	}
	else if (moving_backward)
	{
		left_dir = right_dir = 0;
	}
	else
	{
		left_dir = right_dir = 0;
	}

	// ----------------------------------------------------------------
	// Speed ramp: only active while Shift is held during forward/backward
	// Resets to BASE_PWM when Shift is released or no W/S input
	// ----------------------------------------------------------------
	// Ramp behavior:
	//   W/S alone        -> hold ramp_pwm at current value (BASE_PWM on first press)
	//   W/S + Shift      -> accelerate up to RAMP_MAX, held on Shift release
	//   W/S + Ctrl       -> decelerate down to RAMP_MIN, held on Ctrl release
	//   Release W/S      -> reset ramp_pwm to BASE_PWM for next press
	if (!moving)
	{
		// Not moving — reset so next W/S press starts fresh at BASE_PWM
		ramp_pwm = BASE_PWM;
	}
	else if (shift == 0xA0)
	{
		// Accelerate
		ramp_pwm += RAMP_STEP;
		if (ramp_pwm > RAMP_MAX)
			ramp_pwm = RAMP_MAX;
	}
	else if (ctrl == 0xA0)
	{
		// Decelerate
		if (ramp_pwm > RAMP_MIN + RAMP_STEP)
			ramp_pwm -= RAMP_STEP;
		else
			ramp_pwm = RAMP_MIN;
	}
	// else: W/S held, no modifier — hold ramp_pwm as-is

	if (moving)
	{
		left_pwm = right_pwm = ramp_pwm;
	}
	else
	{
		left_pwm = right_pwm = 0;
	}

	// ----------------------------------------------------------------
	// Steering logic
	// ----------------------------------------------------------------
	uint8_t no_vertical   = (w == 0x05 && s == 0x05);
	uint8_t both_vertical = (w == 0xA0 && s == 0xA0);

	if (no_vertical || both_vertical)
	{
		// In-place turns: ignore ramp, use fixed INPLACE_PWM
		if (a == 0xA0 && d == 0x05)		// turn in place left
		{
			left_dir  = 0;
			right_dir = 1;
			left_pwm  = right_pwm = INPLACE_PWM;
		}
		else if (a == 0x05 && d == 0xA0)	// turn in place right
		{
			left_dir  = 1;
			right_dir = 0;
			left_pwm  = right_pwm = INPLACE_PWM;
		}
	}
	else
	{
		// Moving turns: apply differential to current ramp speed.
		// Outer wheel capped at RAMP_MAX (100), inner at BASE_PWM (70).
		if (a == 0xA0 && d == 0x05)		// steer left
		{
			if (moving_forward)
			{
				// Speed up right (outer), slow down left (inner)
				right_pwm = ramp_pwm + TURN_OFFSET;
				left_pwm  = ramp_pwm - TURN_OFFSET;
			}
			else if (moving_backward)
			{
				// Speed up left (outer going back), slow down right
				left_pwm  = ramp_pwm + TURN_OFFSET;
				right_pwm = ramp_pwm - TURN_OFFSET;
			}
		}
		else if (a == 0x05 && d == 0xA0)	// steer right
		{
			if (moving_forward)
			{
				// Speed up left (outer), slow down right (inner)
				left_pwm  = ramp_pwm + TURN_OFFSET;
				right_pwm = ramp_pwm - TURN_OFFSET;
			}
			else if (moving_backward)
			{
				// Speed up right (outer going back), slow down left
				right_pwm = ramp_pwm + TURN_OFFSET;
				left_pwm  = ramp_pwm - TURN_OFFSET;
			}
		}
	}

	// ----------------------------------------------------------------
	// Clamp PWM values to valid range [0, 100]
	// Outer wheel max is RAMP_MAX (100), inner max is BASE_PWM (70)
	// as specified — use unsigned arithmetic guard for underflow too
	// ----------------------------------------------------------------
	if (left_pwm  > RAMP_MAX) left_pwm  = RAMP_MAX;
	if (right_pwm > RAMP_MAX) right_pwm = RAMP_MAX;

	// radio packet structure is
	//
	// [left motor direction, left motor pwm, right motor direction, right motor pwm]
	//
	uint8_t motors[4] = {left_dir, left_pwm, right_dir, right_pwm};

	// Send data over radio to receiver
	rfm_send(motors, 4);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

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

/* USER CODE BEGIN 4 */
void rfm_config()
{
	// Start in Standby mode
	// Mode in RegOpMode = 001
	// ListenOn in RegOpMode = 0
	rfm_write_reg(0x01, 0x04);

	// wait until sender goes into standby mode
	while (!(rfm_read_reg(0x27) & 0x80));

	// Packet Mode, uC does not directly control modulation
	// FSK Modulation, no modulation shaping
	rfm_write_reg(0x02, 0x00);

	uint16_t bitrate = FXOSC / BITRATE;
	rfm_write_reg(0x03, (uint8_t) (bitrate >> 8));
	rfm_write_reg(0x04, (uint8_t) bitrate);

	// 5 kHz frequency deviation
	rfm_write_reg(0x05, 0x00);
	rfm_write_reg(0x06, 0x52);

	// RF Carrier Frequency - 915 MHz
	rfm_write_reg(0x07, 0xE4);
	rfm_write_reg(0x08, 0xC0);
	rfm_write_reg(0x09, 0x00);

	// standard AFC routine
	rfm_write_reg(0x0B, 0x00);

	// AFC performed every time RX mode is entered
	rfm_write_reg(0x1E, 0x04);

	// PayloadSent on DIO0
	rfm_write_reg(0x25, 0x00);

	// default RSSI threshold
	rfm_write_reg(0x29, 0xE4);

	// Preamble and Sync configuration
	rfm_write_reg(0x2C, 0x00); 			// Preamble MSB
	rfm_write_reg(0x2D, 0x08); 			// Preamble LSB (8 bytes)
	rfm_write_reg(0x2E, 0x88); 			// SyncConfig: Sync on
	rfm_write_reg(0x2F, 0x2D); 			// Sync Value 1
	rfm_write_reg(0x30, 0xD4); 			// Sync Value 2

	// Packet Configuration settings
	// Fixed length packets
	// CRC on
	// No address filtering
	rfm_write_reg(0x37, 0x10);

	// Payload length: 4 bytes
	rfm_write_reg(0x38, 4);

	// Set FIFO threshold, TX will start when FIFO is not empty
	rfm_write_reg(0x3C, 0x8F);

	// Set output on PA1 + PA2 (MUST)!!!!
	rfm_write_reg(0x11, 0x7F);
	rfm_write_reg(0x13, 0x0F);
	rfm_write_reg(0x5A, 0x5D);
	rfm_write_reg(0x5C, 0x7C);

	// wait until PA ramps up and radio is ready
	while (!(rfm_read_reg(0x27) & 0x80));
}

void rfm_select()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

void rfm_deselect()
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void rfm_reset()
{
	HAL_GPIO_WritePin(GPIOA, RFM_RST, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, RFM_RST, GPIO_PIN_RESET);
	HAL_Delay(10);
}

void rfm_write_reg(uint8_t addr, uint8_t byte)
{
	// MSB of addr must be 1 on write
	uint8_t buff[2] = {addr | 0x80, byte};

	rfm_select();
	HAL_SPI_Transmit(&hspi1, buff, 2, 1000);
	rfm_deselect();
}

uint8_t rfm_read_reg(uint8_t addr)
{

	// MSB must be 0 for read
	addr &= ~0x80;

	uint8_t send_buff[2] = {addr, 0};
	uint8_t rcv_buff[2];

	rfm_select();
	HAL_SPI_TransmitReceive(&hspi1, send_buff, rcv_buff, 2, 1000);
	rfm_deselect();

	return rcv_buff[1];
}

void rfm_write_fifo(uint8_t * data, uint8_t len)
{
	uint8_t send_buff[len + 1];
	send_buff[0] = 0x80;
	for (uint8_t i = 0; i < len; i++)
	{
		send_buff[i+1] = data[i];
	}

	rfm_select();
	HAL_SPI_Transmit(&hspi1, send_buff, len + 1, 1000);
	rfm_deselect();
}

void rfm_send(uint8_t * data, uint8_t len)
{
	// Write payload into FIFO register
	rfm_write_fifo(data, len);

	// Switch to TX mode
	rfm_write_reg(0x01, 0x0C);

	// Wait until packet has been sent, then return to standby mode
	while (!(rfm_read_reg(0x28) & 0x08));
	rfm_write_reg(0x01, 0x04);
}
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
