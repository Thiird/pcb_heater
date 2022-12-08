# PCB Heater Firmware

The firmware for the ESP32 is written with the ESP-IDF framework and runs FreeRTOS.

The task run by the MCU are described below (no specific order).

## Monitoring the heater

The temperature of the heater is monitored through an NTC Thermistor as part of a voltage divider. The current going through the heater is monitored by sampling the Voltage on the low-side of the heater as Vref of a voltage divider.

## Listening for button presses

The user can access three buttons:

- **Temp +**, to increase the temperature setpoint
- **Temp -**, to decrease the temperature setpoint
- **Start/Stop**, to start/stop the heating element

## Updating the OLED screen

Through the I2C OLED screen the user can monitor the heater temperature and the setpoint.
