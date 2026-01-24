/*
 * 2-Motor Rover Control with Input Handling and Speed Ramping
 * STM32L432KC - Direct Register Implementation
 *
 * Features:
 * - TEST MODE: Hardwired PWM for motor testing (set TEST_MODE = 1)
 * - NORMAL MODE: Button inputs for Forward, Backward, Left, Right (set TEST_MODE = 0)
 * - Smooth speed ramping up when button held
 * - Smooth speed ramping down when button released
 * - Configurable ramp rates
 * - 2 motor set outputs (left tread and right tread)
 * - Uses direct register access for reliable TIM1 operation
 *
 * Hardware connections:
 * LEFT TREAD MOTORS:
 * - Left motors PWM: PA8 (TIM1_CH1) PIN:D9
 * - Left motors DIR: PA11 PIN:D10
 *
 * RIGHT TREAD MOTORS:
 * - Right motors PWM: PA9 (TIM1_CH2) PIN:D1
 * - Right motors DIR: PA12 PIN:D2
 *
 * BUTTONS (only used in NORMAL MODE):
 * - Forward button: PB4
 * - Backward button: PB5
 * - Left button: PA4
 * - Right button: PA5
 * - Stop button: PA6
 */

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// Motor parameters
#define MAX_SPEED 100
#define MIN_SPEED 0
#define PWM_PERIOD 99       // Keep at 99 for easy 0-100% duty cycle

// Test mode - set to 1 for hardwired PWM testing, 0 for normal button control
#define TEST_MODE 1
#define TEST_PWM_DUTY 80      // Test PWM duty cycle (0-100%)

// Ramping parameters
#define RAMP_UP_STEP 2        // Speed increment per update (faster = more aggressive)
#define RAMP_DOWN_STEP 5      // Speed decrement per update (faster braking)
#define RAMP_UPDATE_MS 200    // Time between ramp updates (milliseconds)

// Target speeds for each tread
static int8_t left_target_speed = 0;
static int8_t right_target_speed = 0;

// Current actual speeds (for ramping)
static int8_t left_current_speed = 0;
static int8_t right_current_speed = 0;

// Timer counter for timing
static volatile uint32_t timer_counter = 0;

// HAL handles (only TIM6 used for timing in NORMAL mode)
TIM_HandleTypeDef htim6;

// Button state structure
typedef struct {
    bool forward;
    bool backward;
    bool left;
    bool right;
    bool stop;
} ButtonState;

// Function prototypes
void SystemClock_Config(void);
void motor_init(void);
void button_init(void);
void led_init(void);
void timer6_init(void);
void Error_Handler(void);
ButtonState read_buttons(void);
void process_inputs(ButtonState buttons);
void update_motor_speeds(void);
void set_motor_pwm(int8_t left_speed, int8_t right_speed);
void motor_stop(void);
uint32_t millis(void);

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

    // Initialize the RCC Oscillators
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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

    // Enable MSI Auto calibration
    HAL_RCCEx_EnableMSIPLLMode();
}

/*
 * Initialize TIM1 for motor PWM control using direct register access
 * This method is more reliable than HAL on this platform
 */
void motor_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Force reset TIM1 completely
    __HAL_RCC_TIM1_FORCE_RESET();
    HAL_Delay(10);
    __HAL_RCC_TIM1_RELEASE_RESET();
    HAL_Delay(10);

    // Ensure TIM1 clock is enabled
    __HAL_RCC_TIM1_CLK_ENABLE();

    // Manually initialize TIM1 registers directly
    TIM1->PSC = 3999;   // Prescaler: 80MHz / 4000 = 20 kHz
    TIM1->ARR = 99;     // Period: 20kHz / 100 = 200 Hz PWM
    TIM1->CCR1 = 0;     // Channel 1 compare (left motors)
    TIM1->CCR2 = 0;     // Channel 2 compare (right motors)

    // Configure PWM mode 1 for channels 1 and 2
    TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);  // PWM mode 1 CH1
    TIM1->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);  // PWM mode 1 CH2

    // Enable preload for both channels
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    // Enable CH1 and CH2 outputs
    TIM1->CCER |= TIM_CCER_CC1E;   // Enable CH1 output
    TIM1->CCER |= TIM_CCER_CC2E;   // Enable CH2 output

    // Configure GPIO for PWM outputs
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // PA8 (TIM1_CH1) and PA9 (TIM1_CH2) as alternate function
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // PA11 and PA12 as GPIO outputs for direction control
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;  // GPIO mode
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Initialize direction pins to forward (LOW = forward with inverted logic)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

    // Enable main output (CRITICAL for TIM1 - advanced timer)
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Enable auto-reload preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    // Generate update event to load all registers
    TIM1->EGR |= TIM_EGR_UG;

    // Enable the counter
    TIM1->CR1 |= TIM_CR1_CEN;
}

/*
 * Initialize LED (PB3)
 */
void led_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

/*
 * Initialize button input pins
 */
void button_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure button pins as inputs with pull-up (active low)
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 * Initialize TIM6 for timing (1ms tick)
 */
void timer6_init(void) {
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 7999;  // 80MHz / 8000 = 10kHz
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 9;  // 10kHz / 10 = 1kHz (1ms)
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
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
}

/*
 * Get current time in milliseconds
 */
uint32_t millis(void) {
    return timer_counter;
}

/*
 * Read button states
 */
ButtonState read_buttons(void) {
    ButtonState buttons;

    buttons.forward = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);
    buttons.backward = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_RESET);
    buttons.left = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET);
    buttons.right = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET);
    buttons.stop = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_RESET);

    return buttons;
}

/*
 * Process button inputs and set target speeds
 */
void process_inputs(ButtonState buttons) {
    if (buttons.stop) {
        left_target_speed = 0;
        right_target_speed = 0;
        return;
    }

    int8_t base_speed = 0;

    if (buttons.forward && !buttons.backward) {
        base_speed = MAX_SPEED;
    } else if (buttons.backward && !buttons.forward) {
        base_speed = -MAX_SPEED;
    }

    if (base_speed != 0) {
        if (buttons.left && !buttons.right) {
            left_target_speed = base_speed / 2;
            right_target_speed = base_speed;
        } else if (buttons.right && !buttons.left) {
            left_target_speed = base_speed;
            right_target_speed = base_speed / 2;
        } else {
            left_target_speed = base_speed;
            right_target_speed = base_speed;
        }
    } else {
        if (buttons.left && !buttons.right) {
            left_target_speed = -MAX_SPEED / 2;
            right_target_speed = MAX_SPEED / 2;
        } else if (buttons.right && !buttons.left) {
            left_target_speed = MAX_SPEED / 2;
            right_target_speed = -MAX_SPEED / 2;
        } else {
            left_target_speed = 0;
            right_target_speed = 0;
        }
    }
}

/*
 * Update motor speeds with ramping
 */
void update_motor_speeds(void) {
    // Ramp left tread
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

    // Ramp right tread
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

    set_motor_pwm(left_current_speed, right_current_speed);
}

/*
 * Stop motors immediately
 */
void motor_stop(void) {
    left_target_speed = 0;
    right_target_speed = 0;
    left_current_speed = 0;
    right_current_speed = 0;
    set_motor_pwm(0, 0);
}

/*
 * Set motor PWM and direction
 */
void set_motor_pwm(int8_t left_speed, int8_t right_speed) {
    uint8_t left_abs_speed, right_abs_speed;

    // LEFT MOTORS
    if (left_speed >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);  // Forward
        left_abs_speed = left_speed;
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);  // Backward
        left_abs_speed = -left_speed;
    }

    // RIGHT MOTORS
    if (right_speed >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);  // Forward
        right_abs_speed = right_speed;
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);  // Backward
        right_abs_speed = -right_speed;
    }

    // Set PWM duty cycles using direct register access
    // Motor driver is inverted: subtract from PWM_PERIOD to flip it
    // So 80% speed command → 20% PWM duty → motor runs fast
    TIM1->CCR1 = PWM_PERIOD - ((PWM_PERIOD * left_abs_speed) / 100);
    TIM1->CCR2 = PWM_PERIOD - ((PWM_PERIOD * right_abs_speed) / 100);
}

/*
 * Error Handler
 */
void Error_Handler(void) {
    __disable_irq();
    motor_stop();

    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        for(volatile uint32_t i = 0; i < 100000; i++);
    }
}

/*
 * Main function
 */
int main(void) {
    uint32_t last_update = 0;

    // Initialize HAL
    HAL_Init();

    // Configure system clock
    SystemClock_Config();

    // Initialize peripherals
    led_init();
    motor_init();

    // Startup indicator - 3 fast blinks
    for(int i = 0; i < 3; i++) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(200);
    }

#if TEST_MODE
    // TEST MODE: Set hardwired PWM for testing
    set_motor_pwm(TEST_PWM_DUTY, TEST_PWM_DUTY);

    // Infinite loop with LED heartbeat
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
        HAL_Delay(1000);
    }
#else
    // NORMAL MODE: Button control with ramping
    timer6_init();
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

        // Toggle LED as heartbeat
        static uint32_t last_led_toggle = 0;
        if ((millis() - last_led_toggle) >= 1000) {
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
            last_led_toggle = millis();
        }
    }
#endif

    return 0;
}
