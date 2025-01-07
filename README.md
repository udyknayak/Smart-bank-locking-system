# Smart-bank-locking-system
#Program in RTOS Environment

#include "LPC17xx.h"  // Device header file for LPC1768
#include <RTL.h>      // RTX header file

// Definitions
#define REED_SENSOR_PIN (1 << 10)  // P2.10 for reed sensor (input)
#define LED_PIN         (1 << 18)  // P1.18 for LED (output)

#define TOUCH_ADC_CHANNEL 0       // ADC channel for P0.23 (AD0.0)
#define TOUCH_PIN_ADC (1U << 23)  // P0.23
#define BUZZER_PIN (1U << 31)     // P1.31

#define TOUCH_THRESHOLD 500       // Threshold value for the touch sensor

// Function Prototypes
void GPIO_Init(void);
void setupADC(void);
uint16_t readADC(uint8_t channel);
__task void task_main(void);
__task void task_LED(void);
__task void task_buzzer(void);

// Task IDs
OS_TID task_LED_id, task_buzzer_id;

// Mutex 
OS_MUT gpio_mutex;             // [KERNEL OBJECT] Mutex to protect GPIO access

int main(void) {
    // Initialize RTX kernel
    os_sys_init(task_main);  // Start task scheduler
    while (1) {
        // Main loop should not execute any application code
    }
}

__task void task_main(void) {
    // Initialize GPIO and ADC
    GPIO_Init();
    setupADC();

    // Initialize mutex 
    os_mut_init(&gpio_mutex);                  // [KERNEL OBJECT] Mutex initialization

    // Create tasks
    task_LED_id = os_tsk_create(task_LED, 1);    // Create LED task
    task_buzzer_id = os_tsk_create(task_buzzer, 2);  // Create buzzer task

    os_tsk_delete_self();  // Delete the main task as it's no longer needed
}

__task void task_LED(void) {
    while (1) {
        os_mut_wait(&gpio_mutex, 0xFFFF);  // [KERNEL OBJECT] Mutex acquisition for GPIO access

        // Handle reed sensor and LED
        if (LPC_GPIO2->FIOPIN & REED_SENSOR_PIN) {
            // Reed sensor HIGH (door closed): Turn off LED
            LPC_GPIO1->FIOCLR = LED_PIN;
        } else {
            // Reed sensor LOW (door open): Turn on LED
            LPC_GPIO1->FIOSET = LED_PIN;
        }

        os_mut_release(&gpio_mutex);  // [KERNEL OBJECT] Mutex release after GPIO access

        os_dly_wait(5);  // Delay for 50 ms (assuming 1 ms tick in RTX config)
    }
}

__task void task_buzzer(void) {
    uint16_t adcValue;

    while (1) {
        adcValue = readADC(TOUCH_ADC_CHANNEL);

        os_mut_wait(&gpio_mutex, 0xFFFF);  // [KERNEL OBJECT] Mutex acquisition for GPIO access

        // Handle touch sensor and buzzer
        if (adcValue > TOUCH_THRESHOLD) {
            LPC_GPIO1->FIOSET = BUZZER_PIN;  // Turn on the buzzer
        } else {
            LPC_GPIO1->FIOCLR = BUZZER_PIN;  // Turn off the buzzer
        }

        os_mut_release(&gpio_mutex);  // [KERNEL OBJECT] Mutex release after GPIO access

        os_dly_wait(5);  // Delay for 50 ms (assuming 1 ms tick in RTX config)
    }
}

// Function to initialize GPIO pins
void GPIO_Init(void) {
    // Configure P2.10 (reed sensor) as GPIO input
    LPC_PINCON->PINSEL4 &= ~(3 << 20);  // P2.10 as GPIO
    LPC_GPIO2->FIODIR &= ~REED_SENSOR_PIN;  // Set P2.10 as input

    // Configure P1.18 (LED) as GPIO output
    LPC_PINCON->PINSEL3 &= ~(3 << 4);  // P1.18 as GPIO
    LPC_GPIO1->FIODIR |= LED_PIN;     // Set P1.18 as output

    // Configure P1.31 (buzzer) as GPIO output
    LPC_SC->PCONP |= (1U << 15);              // Power up GPIO
    LPC_PINCON->PINSEL3 &= ~(3U << 30);       // Configure P1.31 as GPIO
    LPC_GPIO1->FIODIR |= BUZZER_PIN;          // Set P1.31 as output
    LPC_GPIO1->FIOCLR = BUZZER_PIN;           // Ensure buzzer is off initially
}

// Function to configure ADC for touch sensor
void setupADC(void) {
    LPC_SC->PCONP |= (1U << 12);              // Power up ADC
    LPC_PINCON->PINSEL1 |= (1U << 14);        // Configure P0.23 as AD0.0
    LPC_ADC->ADCR = (1U << TOUCH_ADC_CHANNEL) |  // Select ADC channel
                    (4U << 8) |                  // ADC clock divider
                    (1U << 21);                  // Enable ADC
}

// Function to read ADC value from the specified channel
uint16_t readADC(uint8_t channel) {
    LPC_ADC->ADCR |= (1U << 24);              // Start conversion
    while (!(LPC_ADC->ADGDR & (1U << 31)));   // Wait for conversion to complete
    return (LPC_ADC->ADGDR >> 4) & 0xFFF;     // Return 12-bit ADC result
}


Basic Programs:
1) Reed Sensor

#include "LPC17xx.h"  // Device header file for LPC1768

#define REED_SENSOR_PIN (1 << 10)  // P2.10 for reed sensor (input)
#define LED_PIN         (1 << 18)  // P1.18 for LED (output)

// Function to initialize GPIO pins
void GPIO_Init(void) {
    // Configure P2.10 (reed sensor) as GPIO input
    LPC_PINCON->PINSEL4 &= ~(3 << 20);  // P2.10 as GPIO
    LPC_GPIO2->FIODIR &= ~REED_SENSOR_PIN;  // Set P2.10 as input
    
    // Configure P1.18 (LED) as GPIO output
    LPC_PINCON->PINSEL3 &= ~(3 << 4);  // P1.18 as GPIO
    LPC_GPIO1->FIODIR |= LED_PIN;     // Set P1.18 as output
}

// Simple delay function (blocking delay)
void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 20000; j++) {
            __asm volatile ("NOP");  // Do nothing
        }
    }
}

int main(void) {
    GPIO_Init();  // Initialize GPIO pins
    
    while (1) {
        if (LPC_GPIO2->FIOPIN & REED_SENSOR_PIN) {
            // Reed sensor HIGH (door closed): Turn off LED
            LPC_GPIO1->FIOCLR = LED_PIN;
        } else {
            // Reed sensor LOW (door open): Turn on LED
            LPC_GPIO1->FIOSET = LED_PIN;
        }
        delay_ms(50);  // Debounce delay
    }
}

2) Touch sensor

#include <lpc17xx.h>  // LPC1768 header file

#define TOUCH_SENSOR_PIN  (1 << 10)  // P2.10 as Touch Sensor Input
#define BUZZER_PIN        (1 << 22) // P0.22 as Buzzer Output

void delay_ms(uint32_t ms) {
    uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 5000; j++); // Approximate delay
    }
}

int main() {
    // Step 1: Configure GPIO
    LPC_GPIO2->FIODIR &= ~TOUCH_SENSOR_PIN; // Set P2.10 as Input (Touch Sensor)
    LPC_GPIO0->FIODIR |= BUZZER_PIN;        // Set P0.22 as Output (Buzzer)

    // Step 2: Infinite loop to monitor sensor state
    while (1) {
        if (LPC_GPIO2->FIOPIN & TOUCH_SENSOR_PIN) {
            // Touch detected: Turn Buzzer ON
            LPC_GPIO0->FIOSET = BUZZER_PIN;
        } else {
            // No touch detected: Turn Buzzer OFF
            LPC_GPIO0->FIOCLR = BUZZER_PIN;
        }
        delay_ms(100); // Add small delay for stability
    }
}
