# Smart Greenhouse Embedded System

## About the Project

This project is an embedded Smart Greenhouse system developed in C using IAR Embedded Workbench and the SAM3X8E ARM microcontroller (Arduino Due platform).

The purpose of the system is to monitor and control environmental conditions inside a greenhouse, including temperature and light. The system interacts directly with hardware components such as temperature and light sensors, an LCD display, a keypad, LEDs, a push button and a servo motor.

## Features

- Real-time temperature measurement
- Light intensity measurement using an ADC
- LCD-based user interface with multiple menu views
- Keypad navigation and user input
- Adjustable minimum and maximum temperature limits
- Temperature alarm when configured limits are exceeded
- Temperature logging and historical data storage
- Calculation of minimum, maximum and average temperature
- Date and time tracking
- Servo motor control using PWM
- Light and darkness tracking for greenhouse lighting
- Fast simulation mode for testing
- Hardware interrupt handling

## Implementation

The system was programmed in C with direct interaction with the SAM3X8E microcontroller registers.

The implementation includes:

- GPIO configuration for hardware components
- ADC configuration for the light sensor
- Timer/counter based temperature measurement
- PWM generation for servo motor control
- SysTick-based timing
- Interrupt handling
- LCD communication and display functions
- Matrix keypad scanning
- Menu-based user interface
- Temperature data logging using a fixed-size memory pool
- Daily temperature statistics and timestamps

The temperature logging system stores measurements over time and automatically reuses the oldest memory entries when the available storage is full.

## Hardware

The project uses:

- SAM3X8E ARM microcontroller / Arduino Due platform
- Temperature sensor
- Light sensor
- LCD display
- 3x4 keypad
- Servo motor
- LEDs
- Push button

## Technologies

- C
- Embedded Systems
- IAR Embedded Workbench
- ARM / SAM3X8E
- GPIO
- ADC
- PWM
- Timers and Interrupts
- Embedded memory management

## Source Code

The main implementation is available in [`main.c`](main.c).

## What I Learned

Through this project, I gained practical experience in low-level embedded C programming and hardware/software integration. I worked with microcontroller registers, sensors, interrupts, timers, PWM, ADC, user interfaces and memory management while combining several hardware components into one embedded system.

## Academic Project

Developed as part of my Computer Engineering studies at Halmstad University.
