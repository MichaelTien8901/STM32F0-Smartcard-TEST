# STM32F0-Smartcard-TEST
This is a sample program to test the smart card function. 

## Problem
   STM32CubeF0 firmware version 1.8.0 generated code doesn't work for smart card command.  But it was working at previous version 1.6.0.  (I haven't tried 1.7.0)
   
   ("init commit" is firmware version 1.6.0.  "firmware 1.8.0" is for firmware version 1.8.0)
   
## Software

  * STM32CubeMx version 4.21.0, STM32CubeF0 version 1.6.0 and 1.8.0
  * Keil uVision V5.23.0.0
  
## Hardware
* STM32F072RB Discovery Board
* ST8024L smart card interface
* AT88SC0104C smartcard

## Connection

| STM32F072RB  | PIN NAME   | ST8024L         |
|:------------:|-----------:|----------------:|
|      PB9(62) |SCARD_RESET | RSTIN (20)      |
|      PC13(2) |SCARD_3_5V  | 5V/~3V (3)      |
|      PC14(3) |SCARD_OFF   | ~OFF(23)        |
|      PC15(4) |SCARD_VCC   | ~CMDVCC(19)     |
|      PA9(42) |SC_UART_TX  | I/OUC(26)       |
|      PA8(41) |SC_UART_CK  | XTAL1(24)       |

##  Answer To Reset(ATR) of AT88SC0104C
  Both firmware version have no problem to receive ATR 
  0x3B, 0xB2, 0x11, 0x00, 0x10, 0x80, 0x00, 0x01. 
  
## Read Card Manufacture Code command
  * 0x00 0xB6 0x00 0x0C 0x04
  * This manufacture code is configured by user.  Currently the test card had the code 7f 94 89 10.

## Firmware STM32F0 CubeF0 version 1.6.0 (init commit)

![fw 1.6.0 command read configureation](https://github.com/MichaelTien8901/STM32F0-Smartcard-TEST/blob/master/pics/fw1_6_0.png "Data bus for command read configureation")

## Firmware STM32F0 CubeF0 version 1.8.0
![fw 1.8.0 command read configureation](https://github.com/MichaelTien8901/STM32F0-Smartcard-TEST/blob/master/pics/fw1_8_0.png "Data bus for command read configureation")

The data bus is not the same with different version of firmware.  Data in firmware 1.8.0 seems corrupt and data can't be seen in UART.
