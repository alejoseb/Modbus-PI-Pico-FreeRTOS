# Modbus RTU Master and Slave library for Raspberry Pi Pico based on the C++ SDK and FreeRTOS

Includes a project example for independent Master and Slave instances running concurrently.

This repository is a port of the Modbus library for STM32 microcontrollers: https://github.com/alejoseb/Modbus-STM32-HAL-FreeRTOS, 

also, it uses the port of FreeRTOS to the Raspberry Pi Pico published at: https://github.com/PicoCPP/RPI-pico-FreeRTOS

The Library supports any combination of Master or Slave instances.

## How to use the examples
- Install the Raspberry PI Pico C++ SDK according to the official [documentation](https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
- Configure the environment to work with Visual Studio Code
- Clone this repository and initialize the submodules

```bash 
git clone https://github.com/alejoseb/Modbus-PI-Pico-FreeRTOS.git
cd Modbus-PI-Pico-FreeRTOS
git submodule update --init --recursive

```
- Open the repository folder in Visual Studio Code
- update the [launch.json](https://github.com/alejoseb/Modbus-PI-Pico-FreeRTOS/blob/main/.vscode/launch.json) file according to your environment.
- Start the debugging session according to the official documentation

## Recommended Modbus Master and Slave testing tools for Linux and Windows

### Master client Qmodbus
Linux:    https://github.com/ed-chemnitz/qmodbus

Windows:  https://sourceforge.net/projects/qmodbus/

### Slave simulator
Linux: https://sourceforge.net/projects/pymodslave/

Windows: https://sourceforge.net/projects/modrssim2/
