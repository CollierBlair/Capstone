/*
 * 2-Track Rover Motor Control with Input Handling and Speed Ramping
 * STM32L4 - HAL Library Implementation
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

#include "main.h"
#include <stdbool.h>

// Motor parameters
#define MAX_SPEED 100
#define MIN_SPEED 0
#define PWM_PERIOD 999

// Ramping parameters
#define RAMP_UP_STEP 2        // Speed increment per update (faster = more aggressive)
#define RAMP_DOWN_STEP 5      // Speed decrement per update (faster braking)
#define RAMP_UPDATE_MS 20     // Time between ramp updates (milliseconds)

// Target speeds for each motor
static int8_t left_target_speed = 0;
static int8_t right_target_speed = 0;

// Current actual speeds (for ramping)
static int8_t left_current_speed = 0;
static int8_t right_current_speed = 0;

// Button state structure
typedef struct {
    bool forward;
    bool backward;
    bool left;
    bool right;
    bool stop;
} ButtonState;

// HAL handles for peripherals
TIM_HandleTypeDef htim2;
GPIO_InitTypeDef GPIO_InitStruct = {0};

// Function prototypes
void motor_init(void);
void button_init(void);
ButtonState read_buttons(void);
void process_inputs(ButtonState buttons);
void update_motor_speeds(void);
void set_motor_pwm(int8_t left_speed, int8_t right_speed);
void motor_stop(void);

/*
 * Initialize GPIO and Timer for motor control using HAL
 */
void motor_init(void) {
    // Configure PA0 and PA1 as alternate function (TIM2_CH1 and TIM2_CH2)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure PA4 and PA5 as GPIO output (direction pins)
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = 0;  // Not used for output pins
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Configure TIM2 for PWM
    // Note: Clock will be enabled by HAL_TIM_PWM_MspInit when HAL_TIM_PWM_Init is called
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 79;  // 80MHz / 80 = 1MHz timer clock
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = PWM_PERIOD;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }

    // Configure PWM channels
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }

    // Start PWM generation
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

/*
 * Initialize button input pins using HAL
 */
void button_init(void) {
    // Enable GPIOB clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure PB0-PB4 as inputs with pull-up resistors (buttons are active low)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = 0;  // Not used for input pins
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*
 * Get current time in milliseconds using HAL
 */
uint32_t millis(void) {
    return HAL_GetTick();
}

/*
 * Read button states using HAL
 * Returns ButtonState structure with current button states
 */
ButtonState read_buttons(void) {
    ButtonState buttons;

    // Buttons are active low, so invert the GPIO read
    buttons.forward = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET);
    buttons.backward = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET);
    buttons.left = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_RESET);
    buttons.right = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_RESET);
    buttons.stop = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET);

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
 * Call this periodically (e.g., every 20ms)
 */
void update_motor_speeds(void) {
    // Ramp left motor
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

    // Ramp right motor
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
 * Set motor PWM and direction using HAL
 */
void set_motor_pwm(int8_t left_speed, int8_t right_speed) {
    // Left motor direction
    if (left_speed >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // Forward
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // Backward
        left_speed = -left_speed;
    }

    // Right motor direction
    if (right_speed >= 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);  // Forward
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // Backward
        right_speed = -right_speed;
    }

    // Set PWM duty cycles using HAL
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (PWM_PERIOD * left_speed) / 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (PWM_PERIOD * right_speed) / 100);
}

/*
 * Main function
 * Note: HAL_Init() and SystemClock_Config() should be called before this
 * in the main.c file generated by STM32CubeIDE
 */
int main(void) {
    uint32_t last_update = 0;

    // Initialize HAL library (should be done in main.c, but included here for completeness)
    // HAL_Init();
    // SystemClock_Config();

    // Initialize peripherals
    motor_init();
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
        // The main work is done in timed intervals anyway
    }

    return 0;
}
