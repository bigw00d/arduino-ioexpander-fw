# arduino-ioexpander-fw
IO Expander Device Arduino Firmware with UART

## Overview

<img width="80%" src="./img/overview.png" />  

## Requirement

- Arduino Nano or Arduino Pro Mini
- Arduino IDE (Version: 1.8.5)

## Usage 
 - Use host sample application  
     - [arduino-ioexpander-lib(Arduino)](https://github.com/bigw00d/arduino-ioexpander-lib)
     - [android-ioexpander-host-sample(Android)](https://github.com/bigw00d/android-ioexpander-host-sample)

## Description

###  Pin connections

|Arduino Host Dev  |IO expander Device  |Note  |
|---|---|---|
|TX  | RX ||
|RX  | TX ||
|GND  |GND  ||

### Sequences 
<img width="80%" src="./img/TransferSequence.png" />  

 * Func No. is 7bit
 * W = 0b0, R = 0b1  
 * 'Size' is 8bit 
 * STX : 0x20, ETX : 0x30, ACK : 0x06, NACK : 0x15  

### Function Maps 
<img width="80%" src="./img/FunctionMaps.png" />  

 * Reading I2C Data: Write sequence(Write Slave Address, Read Length) -> Read Sequence 
 * IO_DIR : OUT = 0b1, IN = 0b0 

## Port Assignment

<img width="50%" src="./img/PortAsignArduinoNano.png" />  

 * Supports interrupts on Digital0 only.
