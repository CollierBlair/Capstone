/*
<<<<<<< HEAD
 * 4-Motor Rover Control with Input Handling and Speed Ramping
 * STM32L4 - HAL Library Implementation
 *
 * Features:
 * - TEST MODE: Hardwired PWM for motor testing (set TEST_MODE = 1)
 * - NORMAL MODE: Button inputs for Forward, Backward, Left, Right (set TEST_MODE = 0)
 * - Smooth speed ramping up when button held
 * - Smooth speed ramping down when button released
 * - Configurable ramp rates
 * - 4 independent motor outputs (2 per tread)
 * - Uses TIM6 for timing with HAL
 *
 * TEST MODE USAGE:
 * - Set TEST_MODE to 1 in code
 * - Adjust TEST_PWM_DUTY (0-100%) for desired speed
 * - All 4 motors will run forward at the specified duty cycle
 * - No button input needed
 *
 * Hardware connections (based on provided pinout):
 * LEFT TREAD:
 * - Left Front motor PWM: PB0 (TIM1_CH2N)
 * - Left Front motor DIR: PA9 (USART1_TX)
 * - Left Rear motor PWM: PB1 (TIM1_CH3N)
 * - Left Rear motor DIR: PA10 (USART1_RX)
 *
 * RIGHT TREAD:
 * - Right Front motor PWM: PA8 (TIM1_CH1)
 * - Right Front motor DIR: PA12
 * - Right Rear motor PWM: PB6 (TIM16_CH1N)
 * - Right Rear motor DIR: PC14
 *
 * BUTTONS (using available GPIO - only used in NORMAL MODE):
 * - Forward button: PB4 (SPI1_MISO)
 * - Backward button: PB5 (SPI1_MOSI)
 * - Left button: PA4 (ADC12_IN9)
 * - Right button: PA5 (ADC12_IN10)
 * - Stop button: PA6 (ADC12_IN11)
 */

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
=======
 * 2-Track Rover Motor Control with Input Handling and Speed Ramping
 * STM32L4 - Direct register access only
 *
 * Features:
 * - Button inputs for Forward, Backward, Left, Right
 * - Smooth speed ramping up when button held
 * - Smooth speed ramping down when button released
 * - Configurable ramp rates
 *
 * Hardware connections:
 * - Left motor PWM: PA0 (TIM2_CH1)
 * - Left motor DIR: PA4
 * - Right motor PWM: PA1 (TIM2_CH2)
 * - Right motor DIR: PA5
 *
 * - Forward button: PB0 (active low with pull-up)
 * - Backward button: PB1 (active low with pull-up)
 * - Left button: PB2 (active low with pull-up)
 * - Right button: PB3 (active low with pull-up)
 * - Stop button: PB4 (active low with pull-up)
 */

#include "stm32l4xx.h"
#include <stdbool.h>
>>>>>>> working-branch

// Motor parameters
#define MAX_SPEED 100
#define MIN_SPEED 0
#define PWM_PERIOD 999

<<<<<<< HEAD
// Test mode - set to 1 for hardwired PWM testing, 0 for normal button control
#define TEST_MODE 1
#define TEST_PWM_DUTY 50      // Test PWM duty cycle (0-100%)

// Ramping parameters
#define RAMP_UP_STEP 2        // Speed increment per update (faster = more aggressive)
#define RAMP_DOWN_STEP 5      // Speed decrement per update (faster braking)
#define RAMP_UPDATE_MS 200     // Time between ramp updates (milliseconds)

// Target speeds for each tread
=======
// Ramping parameters
#define RAMP_UP_STEP 2        // Speed increment per update (faster = more aggressive)
#define RAMP_DOWN_STEP 5      // Speed decrement per update (faster braking)
#define RAMP_UPDATE_MS 20     // Time between ramp updates (milliseconds)

// Target speeds for each motor
>>>>>>> working-branch
static int8_t left_target_speed = 0;
static int8_t right_target_speed = 0;

// Current actual speeds (for ramping)
static int8_t left_current_speed = 0;
static int8_t right_current_speed = 0;

<<<<<<< HEAD
// Timer counter for timing
static volatile uint32_t timer_counter = 0;

// HAL handles
TIM_HandleTypeDef htim1;   // PWM timer for most motors
TIM_HandleTypeDef htim16;  // PWM timer for one motor
TIM_HandleTypeDef htim6;   // Timing timer

=======
>>>>>>> working-branch
// Button state structure
typedef struct {
    bool forward;
    bool backward;
    bool left;
    bool right;
    bool stop;
} ButtonState;

// Function prototypes
<<<<<<< HEAD
void SystemClock_Config(void);
void motor_init(void);
void button_init(void);
void timer6_init(void);
void Error_Handler(void);
=======
void motor_init(void);
void button_init(void);
void systick_init(void);
>>>>>>> working-branch
ButtonState read_buttons(void);
void process_inputs(ButtonState buttons);
void update_motor_speeds(void);
void set_motor_pwm(int8_t left_speed, int8_t right_speed);
void motor_stop(void);
<<<<<<< HEAD

/*
 * System Clock Configuration
 * Configure the system clock to 80 MHz
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    // Configure the main internal regulator output voltage
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    // Initialize the RCC Oscillators according to the specified parameters
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;  // 4 MHz
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;  // 4MHz * 40 / 1 / 2 = 80MHz
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    // Initialize the CPU, AHB and APB buses clocks
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

/*
 * Initialize GPIO and Timers for motor control
 */
void motor_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    // Zero-initialize the timer handles
    memset(&htim1, 0, sizeof(htim1));
    memset(&htim16, 0, sizeof(htim16));

    // Enable clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();

    // ===== Configure PWM GPIO Pins =====

    // TIM1 pins: PA8 (CH1), PB0 (CH2N), PB1 (CH3N)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // TIM16 pin: PB6 (CH1N)
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF14_TIM16;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ===== Configure Direction GPIO Pins =====
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Initialize direction pins to low
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

    // ===== Configure TIM1 (3 channels) =====
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 79;  // 80MHz / 80 = 1MHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Configure PWM channels
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // Channel 1 (Right Front - PA8)
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    // Channel 2N (Left Front - PB0)
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    // Channel 3N (Left Rear - PB1)
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }

    // Configure break and dead time (required for TIM1)
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
    sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    sBreakDeadTimeConfig.Break2Filter = 0;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    // ===== Configure TIM16 (1 channel) =====
    htim16.Instance = TIM16;
    htim16.Init.Prescaler = 79;  // 80MHz / 80 = 1MHz
    htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim16.Init.Period = PWM_PERIOD;
    htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim16.Init.RepetitionCounter = 0;
    htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim16) != HAL_OK) {
        Error_Handler();
    }

    // Channel 1N (Right Rear - PB6)
    if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }

    // Start PWM on all channels
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);    // Right Front (CH1)
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // Left Front (CH2N)
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // Left Rear (CH3N)
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);// Right Rear (CH1N)
=======
void delay_ms(uint32_t ms);

// SysTick counter for timing
static volatile uint32_t systick_counter = 0;

/*
 * Initialize GPIO and Timer for motor control
 */
void motor_init(void) {
    // Enable clocks for GPIOA and TIM2
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // Configure PA0 and PA1 as alternate function (TIM2_CH1 and TIM2_CH2)
    GPIOA->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
    GPIOA->MODER |= GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1;  // Alternate function

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1);
    GPIOA->AFR[0] |= (1 << GPIO_AFRL_AFSEL0_Pos) | (1 << GPIO_AFRL_AFSEL1_Pos);  // AF1

    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED0 | GPIO_OSPEEDR_OSPEED1;  // High speed

    // Configure PA4 and PA5 as GPIO output (direction pins)
    GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5);
    GPIOA->MODER |= GPIO_MODER_MODE4_0 | GPIO_MODER_MODE5_0;  // Output mode
    GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5;

    // Configure TIM2 for PWM
    TIM2->PSC = 79;  // 80MHz / 80 = 1MHz timer clock
    TIM2->ARR = PWM_PERIOD;

    // PWM mode 1 for channels 1 and 2
    TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC2M);
    TIM2->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);  // CH1
    TIM2->CCMR1 |= (TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);  // CH2

    // Enable preload
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    TIM2->CR1 |= TIM_CR1_ARPE;

    // Enable channel outputs
    TIM2->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    // Initialize to stopped
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;

    // Start timer
    TIM2->CR1 |= TIM_CR1_CEN;
>>>>>>> working-branch
}

/*
 * Initialize button input pins
 */
void button_init(void) {
<<<<<<< HEAD
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clocks (already enabled in motor_init)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure button pins as inputs with pull-up (active low)
    // PB4, PB5 for Forward/Backward
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PA4, PA5, PA6 for Left/Right/Stop
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 * Initialize TIM6 for timing (1ms tick)
 */
void timer6_init(void) {
    // Zero-initialize the timer handle
    memset(&htim6, 0, sizeof(htim6));

    // Enable TIM6 clock
    __HAL_RCC_TIM6_CLK_ENABLE();

    // Configure TIM6 for 1ms interrupts
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 7999;  // 80MHz / 8000 = 10kHz
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 9;  // 10kHz / 10 = 1kHz (1ms)
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if ((HAL_StatusTypeDef)HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }

    // Enable TIM6 interrupt
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    // Start the timer with interrupt
    if ((HAL_StatusTypeDef)HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

/*
 * TIM6 interrupt handler
 */
void TIM6_DAC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);
}

/*
 * TIM6 Period elapsed callback
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        timer_counter++;
    }
=======
    // Enable GPIOB clock
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

    // Configure PB0-PB4 as inputs
    GPIOB->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 |
                      GPIO_MODER_MODE2 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4);

    // Enable pull-up resistors (buttons are active low)
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 |
                      GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3 | GPIO_PUPDR_PUPD4);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD0_0 | GPIO_PUPDR_PUPD1_0 |
                     GPIO_PUPDR_PUPD2_0 | GPIO_PUPDR_PUPD3_0 | GPIO_PUPDR_PUPD4_0);
}

/*
 * Initialize SysTick for timing (1ms tick)
 */
void systick_init(void) {
    // Configure SysTick for 1ms interrupts (assuming 80MHz clock)
    SysTick->LOAD = 80000 - 1;  // 80MHz / 80000 = 1kHz (1ms)
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

/*
 * SysTick interrupt handler
 */
void SysTick_Handler(void) {
    systick_counter++;
>>>>>>> working-branch
}

/*
 * Get current time in milliseconds
 */
uint32_t millis(void) {
<<<<<<< HEAD
    return timer_counter;
}

/*
 * Delay function using HAL
 */
void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
=======
    return systick_counter;
}

/*
 * Delay function using SysTick
 */
void delay_ms(uint32_t ms) {
    uint32_t start = millis();
    while ((millis() - start) < ms);
>>>>>>> working-branch
}

/*
 * Read button states
 * Returns ButtonState structure with current button states
 */
ButtonState read_buttons(void) {
    ButtonState buttons;
<<<<<<< HEAD

    // Buttons are active low
    buttons.forward = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
    buttons.backward = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
    buttons.left = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET);
    buttons.right = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET);
    buttons.stop = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET);
=======
    uint32_t idr = GPIOB->IDR;

    // Buttons are active low
    buttons.forward = !(idr & GPIO_IDR_ID0);
    buttons.backward = !(idr & GPIO_IDR_ID1);
    buttons.left = !(idr & GPIO_IDR_ID2);
    buttons.right = !(idr & GPIO_IDR_ID3);
    buttons.stop = !(idr & GPIO_IDR_ID4);
>>>>>>> working-branch

    return buttons;
}

/*
 * Process button inputs and set target speeds
 */
void process_inputs(ButtonState buttons) {
    // Check for stop button first - highest priority
    if (buttons.stop) {
        left_target_speed = 0;
        right_target_speed = 0;
        return;
    }

    // Calculate base speed based on forward/backward
    int8_t base_speed = 0;

    if (buttons.forward && !buttons.backward) {
        base_speed = MAX_SPEED;
    } else if (buttons.backward && !buttons.forward) {
        base_speed = -MAX_SPEED;
    }

    // Apply turning if moving
    if (base_speed != 0) {
        if (buttons.left && !buttons.right) {
            // Turn left - reduce left motor speed
            left_target_speed = base_speed / 2;
            right_target_speed = base_speed;
        } else if (buttons.right && !buttons.left) {
            // Turn right - reduce right motor speed
            left_target_speed = base_speed;
            right_target_speed = base_speed / 2;
        } else {
            // Go straight
            left_target_speed = base_speed;
            right_target_speed = base_speed;
        }
    } else {
        // Not moving forward/backward, check for pivot turns
        if (buttons.left && !buttons.right) {
            // Pivot left
            left_target_speed = -MAX_SPEED / 2;
            right_target_speed = MAX_SPEED / 2;
        } else if (buttons.right && !buttons.left) {
            // Pivot right
            left_target_speed = MAX_SPEED / 2;
            right_target_speed = -MAX_SPEED / 2;
        } else {
            // No buttons pressed - stop
            left_target_speed = 0;
            right_target_speed = 0;
        }
    }
}

/*
 * Update motor speeds with ramping
<<<<<<< HEAD
 * Call this periodically (e.g., every 200ms)
 */
void update_motor_speeds(void) {
    // Ramp left tread
=======
 * Call this periodically (e.g., every 20ms)
 */
void update_motor_speeds(void) {
    // Ramp left motor
>>>>>>> working-branch
    if (left_current_speed < left_target_speed) {
        left_current_speed += RAMP_UP_STEP;
        if (left_current_speed > left_target_speed) {
            left_current_speed = left_target_speed;
        }
    } else if (left_current_speed > left_target_speed) {
        left_current_speed -= RAMP_DOWN_STEP;
        if (left_current_speed < left_target_speed) {
            left_current_speed = left_target_speed;
        }
    }

<<<<<<< HEAD
    // Ramp right tread
=======
    // Ramp right motor
>>>>>>> working-branch
    if (right_current_speed < right_target_speed) {
        right_current_speed += RAMP_UP_STEP;
        if (right_current_speed > right_target_speed) {
            right_current_speed = right_target_speed;
        }
    } else if (right_current_speed > right_target_speed) {
        right_current_speed -= RAMP_DOWN_STEP;
        if (right_current_speed < right_target_speed) {
            right_current_speed = right_target_speed;
        }
    }

    // Apply speeds to motors
    set_motor_pwm(left_current_speed, right_current_speed);
}

/*
 * Stop motors immediately (emergency stop)
 * Sets all speeds to zero without ramping
 */
void motor_stop(void) {
    left_target_speed = 0;
    right_target_speed = 0;
    left_current_speed = 0;
    right_current_speed = 0;
    set_motor_pwm(0, 0);
}

/*
<<<<<<< HEAD
 * Set motor PWM and direction for all 4 motors
 * Both motors on each tread receive the same speed/direction
 */
void set_motor_pwm(int8_t left_speed, int8_t right_speed) {
    uint8_t left_abs_speed, right_abs_speed;

    // ===== LEFT TREAD (both motors) =====
    if (left_speed >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);   // Left Front DIR
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  // Left Rear DIR
        left_abs_speed = left_speed;
    } else {
        // Backward direction
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);  // Left Front DIR
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET); // Left Rear DIR
        left_abs_speed = -left_speed;
    }

    // ===== RIGHT TREAD (both motors) =====
    if (right_speed >= 0) {
        // Forward direction
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // Right Front DIR
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  // Right Rear DIR
        right_abs_speed = right_speed;
    } else {
        // Backward direction
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET); // Right Front DIR
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Right Rear DIR
        right_abs_speed = -right_speed;
    }

    // Set PWM duty cycles for LEFT TREAD
    uint32_t left_pwm_value = (PWM_PERIOD * left_abs_speed) / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, left_pwm_value);  // Left Front (CH2N)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, left_pwm_value);  // Left Rear (CH3N)

    // Set PWM duty cycles for RIGHT TREAD
    uint32_t right_pwm_value = (PWM_PERIOD * right_abs_speed) / 100;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, right_pwm_value);  // Right Front (CH1)
    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, right_pwm_value); // Right Rear (CH1N)
}

/*
 * Error Handler
 */
void Error_Handler(void) {
    // Disable interrupts
    __disable_irq();

    // Stop motors for safety
    motor_stop();

    // Infinite loop
    while (1) {
        // You could add LED blinking here to indicate error
    }
=======
 * Set motor PWM and direction
 */
void set_motor_pwm(int8_t left_speed, int8_t right_speed) {
    // Left motor direction
    if (left_speed >= 0) {
        GPIOA->BSRR = GPIO_BSRR_BS4;  // Forward
    } else {
        GPIOA->BSRR = GPIO_BSRR_BR4;  // Backward
        left_speed = -left_speed;
    }

    // Right motor direction
    if (right_speed >= 0) {
        GPIOA->BSRR = GPIO_BSRR_BS5;  // Forward
    } else {
        GPIOA->BSRR = GPIO_BSRR_BR5;  // Backward
        right_speed = -right_speed;
    }

    // Set PWM duty cycles
    TIM2->CCR1 = (PWM_PERIOD * left_speed) / 100;
    TIM2->CCR2 = (PWM_PERIOD * right_speed) / 100;
>>>>>>> working-branch
}

/*
 * Main function
 */
int main(void) {
    uint32_t last_update = 0;

<<<<<<< HEAD
    // Initialize HAL
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize peripherals
    timer6_init();
    motor_init();

#if TEST_MODE
    // TEST MODE: Set hardwired PWM for testing
    // All motors run at TEST_PWM_DUTY% in forward direction
    set_motor_pwm(TEST_PWM_DUTY, TEST_PWM_DUTY);

    // Infinite loop - motors keep running at fixed speed
    while (1) {
        // Motors run continuously at hardwired PWM value
        // You can comment out the line below and uncomment the HAL_Delay to blink an LED if needed
        // HAL_Delay(500);
    }
#else
    // NORMAL MODE: Button control with ramping
=======
    // Initialize peripherals
    systick_init();
    motor_init();
>>>>>>> working-branch
    button_init();

    while (1) {
        // Read button inputs
        ButtonState buttons = read_buttons();

        // Process inputs to determine target speeds
        process_inputs(buttons);

        // Update motor speeds with ramping at fixed intervals
        if ((millis() - last_update) >= RAMP_UPDATE_MS) {
            update_motor_speeds();
            last_update = millis();
        }

        // Optional: Add small delay to reduce CPU usage
<<<<<<< HEAD
        // HAL_Delay(1);
    }
#endif
=======
        // The main work is done in timed intervals anyway
    }
>>>>>>> working-branch

    return 0;
}
