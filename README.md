# IGV_With_BareMetalOS
# Intelligent Guided Vehicle (IGV) Project

## Overview

The Intelligent Guided Vehicle (IGV) project involves developing an autonomous vehicle designed for agriculture-based activities like seeding and crop cutting. The system utilizes modern AI and embedded electronics to operate effectively. This project is implemented using the BareMetal OS (BMOS) on STM32 microcontrollers.

## Hardware

- **Microcontroller:** STM32F407G DISC-1
- **Other Components:**
  - Raspberry Pi 4
  - ESP32
  - Motors
  - Sensors
  - Servos

## Software

- **Embedded Software:**
  - STM32CubeIDE
  - Arduino IDE
  - BareMetal OS
  - C Programming
- **Programming Languages:**
  - C
  - Python

## Project Structure

### Source Code

The main source code for the project is contained in `main.c`. Below is a brief overview of the code:

#### Includes and Defines

- **osKernel.c:** Contains the Bare Metal OS kernel code.
- **string.h, stdio.h:** Standard libraries for string operations and formatted I/O.
- **GPIO and Timer Definitions:** Configure ports and pins for motor and servo control.

#### Private Variables

Used for motor and servo control, UART communication, and task state management.

#### Function Prototypes

Prototypes for initialization functions and task functions.

### Main Function

#### Initialization:

```c
HAL_Init();
SystemClock_Config();
MX_GPIO_Init();
MX_TIM3_Init();
MX_USART2_UART_Init();
```
Initializes HAL library, system clock, GPIO, timers, and UART.

Task Creation:
```c
BMOS_CreateTask(PulseLEDTask, "PulseLED", 128, NULL, 5);
BMOS_CreateTask(DCMotorTask, "DCMotor", 128, NULL, 5);
BMOS_CreateTask(LED4Task, "LED4", 128, NULL, 5);
BMOS_CreateTask(Servo1Task, "Servo1", 128, NULL, 5);
```
Creates four tasks with specified priorities and stack sizes.

Scheduler Start:
```c
BMOS_StartScheduler();
```
Starts the Bare Metal OS scheduler.

Infinite Loop:
```c
Copy code
while (1) {
    /* Your main loop code here */
}
```
Task Functions
PulseLEDTask:
```c

void PulseLEDTask(void) {
    while (1) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
        BMOS_Delay(1000);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);
        BMOS_Delay(1000);
    }
}
```
Blinks an LED on GPIOD Pin 15 with a 1-second interval.

Servo1Task:
```c

void Servo1Task(void) {
    while (1) {
        if (srm_cmd >= 21 && srm_cmd <= 23) {
            // Adjust servo positions based on command
        }
        // Update PWM values for servos
        __HAL_TIM_SET_COMPARE(&Dir_Servo_Tmr, Servo_R_F, Servo_R_F_POS);
        // More PWM updates...
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
        BMOS_Delay(1000);
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
        BMOS_Delay(1000);
    }
}
```
Controls servo positions and blinks an LED based on servo operations.

LED4Task:
```c
void LED4Task(void) {
    while (1) {
        if (MainCMD <= 20) {
            dcm_cmd = MainCMD;
        } else if (MainCMD >= 20 && MainCMD <= 35) {
            srm_cmd = MainCMD;
        }
        BMOS_Delay(1);
    }
}
```
Updates commands for DC motors and servos based on UART input.

DCMotorTask:
```c
void DCMotorTask(void) {
    while (1) {
        HAL_GPIO_WritePin(Motor_EN_Port, M_R_EN_A | M_R_EN_B | M_L_EN_A | M_L_EN_B, 1);
        // Control DC motor based on dcm_cmd
        // Update GPIO pins to control motor direction
        BMOS_Delay(1000);
    }
}
```
Controls DC motors based on received commands.

Initialization Functions
- **MX_TIM3_Init: Configures Timer 3 for PWM signal generation, used for controlling servos.
- **MX_USART2_UART_Init: Sets up UART for communication.
- **MX_GPIO_Init: Initializes GPIO pins used for various controls.
  
Error Handling and Callbacks
- **HAL_UART_RxCpltCallback: Handles UART receive complete interrupt, processes commands.
- **HAL_TIM_PeriodElapsedCallback: Increments a global tick counter on Timer 2 interrupts.
- **Error_Handler: Handles errors by halting execution.
  
Summary
The code uses the Bare Metal OS to manage tasks in the STM32 microcontroller. It sets up GPIO, timers, and UART, then creates and schedules tasks for LED pulsing, servo control, DC motor control, and command processing. The system operates efficiently with task switching managed by the Bare Metal OS, ensuring real-time performance for the Intelligent Guided Vehicle (IGV).

####
