/*
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

// Function prototypes
void motor_init(void);
void button_init(void);
void systick_init(void);
ButtonState read_buttons(void);
void process_inputs(ButtonState buttons);
void update_motor_speeds(void);
void set_motor_pwm(int8_t left_speed, int8_t right_speed);
void motor_stop(void);
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
}

/*
 * Initialize button input pins
 */
void button_init(void) {
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
}

/*
 * Get current time in milliseconds
 */
uint32_t millis(void) {
    return systick_counter;
}

/*
 * Delay function using SysTick
 */
void delay_ms(uint32_t ms) {
    uint32_t start = millis();
    while ((millis() - start) < ms);
}

/*
 * Read button states
 * Returns ButtonState structure with current button states
 */
ButtonState read_buttons(void) {
    ButtonState buttons;
    uint32_t idr = GPIOB->IDR;

    // Buttons are active low
    buttons.forward = !(idr & GPIO_IDR_ID0);
    buttons.backward = !(idr & GPIO_IDR_ID1);
    buttons.left = !(idr & GPIO_IDR_ID2);
    buttons.right = !(idr & GPIO_IDR_ID3);
    buttons.stop = !(idr & GPIO_IDR_ID4);

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
}

/*
 * Main function
 */
int main(void) {
    uint32_t last_update = 0;

    // Initialize peripherals
    systick_init();
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
