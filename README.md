# SSR_IP - AC Dimmer Control with STM32F107

This project implements an AC dimmer control system using an STM32F107 microcontroller. It features:

- Zero-crossing detection for precise AC phase control
- UART interface for setting dimmer values (0-100%)
- Current sensing with real-time monitoring
- Pressure sensing
- High-precision timing using Timer1 for microsecond-level control

## Hardware Components

- STM32F107 microcontroller
- Zero-crossing detection circuit connected to PE13
- Solid State Relay (SSR) connected to PC6
- Current sensors (4 channels)
- Pressure sensor (1 channel)

## Features

- Dimming range: 0-100% (0 = off, 100 = full on)
- Current measurement with 135mV/A sensors
- Real-time monitoring of sensor values via UART
- Microsecond-precision phase control for smooth dimming

## Communication

- UART1 at 115200 baud for control and monitoring
- Enter values 0-100 to set dimming level
- Sensor values displayed every 500ms

## Project Structure

- Core/Src - Main source files
- Core/Inc - Header files
- Drivers - STM32 HAL drivers
