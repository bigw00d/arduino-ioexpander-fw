# arduino-ioexpander-fw
IO Expander Device Arduino Firmware with uart

## Example

<img width="80%" src="./img/example.png" />  

## Requirement

- Arduino Nano or Arduino Pro Mini
- Arduino IDE (Version: 1.8.5)

##  Pin connections

|Host Device  |IO expander Device  |Note  |
|---|---|---|
|3V3 |3V3  ||
|GND  |GND  ||
|GPIO  | /INT |IO Ex -> Host|
|TX  | RX ||
|RX  | TX ||
|GPIO  | /START |Host -> IO Ex|

## Usage 
 - Use arduino-ioexpander-lib (Arduino Library. see [arduino-ioexpander-lib](https://github.com/bigw00d/arduino-ioexpander-lib))
