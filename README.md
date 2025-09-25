# Scorbot STM32 Controller

## Introduction
This project adapts the Scorbot robotic arm control from Arduino to the STM32F439ZI (Nucleo-144) microcontroller.  
The goal was to create an **autonomous system**, independent of MATLAB, reusing parts of the original code and rewriting others for the new hardware.

---

## System Overview
- **Deterministic control:** 20 ms fixed-time loops for reliable and repeatable movements.
- **Autonomous cycles:** Robot performs a full movement and returns to the starting position automatically.
- **Modular architecture:** State machine manages robot states:
  - `IDLE` – motors off, waiting for commands
  - `MOVEMENT` – executing motion
  - `ENCODER` – reading motor positions
  - `PID` – calculating PWM control
  - `RETURNING` – returning to start

---

## Hardware
- **STM32F439ZI Nucleo-144**: ARM Cortex-M4, 2 MB Flash, 256 KB SRAM, PWM timers, ADC/DAC, multiple interfaces (UART, SPI, I2C, CAN, USB, Ethernet)  
- **Scorbot ER-V**: 5-DOF robotic arm with DC motors and encoders  
- **H-bridges and wiring**: Six H-bridges control five motors (supports optional gripper motor)  

---

## Software
- **Arduino IDE** compatible for STM32 development  
- **PID motor control** for precise movements  
- **FreeRTOS** for preemptive task scheduling:  
  - `pidTask` – PID calculation for each motor  
  - `moveMotor` – applies PWM to motors  
  - `read_motor_encoders` – updates motor positions  
  - `loggerTask` – monitors system performance  
- **Serial debug** via UART

---

## Results
The system runs autonomously, with precise, repeatable motion.  
Modular code and RTOS task management allow easy maintenance and future expansion.

