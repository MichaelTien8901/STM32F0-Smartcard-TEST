# STM32F0-Smartcard-TEST
This is a sample program to test the smart card function. 
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

## ATR of AT88SC0104C
  0x3B, 0xB2, 0x11, 0x00, 0x10, 0x80, 0x00, 0x01
  
## Read Card Manufacture Code command
  0x00 0xB6 0x00 0x0C 0x04
  This manufacture code is configured by user.  Currently the test card had the code 7f 94 89 10.

## Firmware STM32F0 CubeF0 version 1.6.0

![fw 1.6.0 command read configureation](https://github.com/MichaelTien8901/STM32F0-Smartcard-TEST/blob/master/pics/fw1_6_0.png "Data bus for command read configureation")

## Firmware STM32F0 CubeF0 version 1.8.0
![fw 1.8.0 command read configureation](https://github.com/MichaelTien8901/STM32F0-Smartcard-TEST/blob/master/pics/fw1_8_0.png "Data bus for command read configureation")
