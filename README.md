[![Build Status](https://travis-ci.com/almedso/epd-gde021a1.svg?branch=master)](https://travis-ci.com/caemor/epd-waveshare)

This library contains a driver for E-Paper Modules with Controller GDE021A1 ((172x72 B/W Pixel via SPI)).

It uses the [embedded graphics](https://crates.io/crates/embedded-graphics) library for the optional graphics support.

A 2018-edition compatible version (Rust 1.31+) is needed.

Other similiar libraries with support for much more displays are [u8g2](https://github.com/olikraus/u8g2) and [GxEPD](https://github.com/ZinggJM/GxEPD) for arduino.

## Usage


```Rust
// Setup the epd


// Setup the graphics

```

### Interface

| Interface | Description |
| :---: |  :--- |
| DIN   | 	SPI MOSI |
| CLK   | 	SPI SCK |
| CS    | 	SPI chip select (Low active) |
| DC    | 	Data/Command control pin (High for data, and low for command) |
| RST   | 	External reset pin (Low for reset) |
| BUSY  | 	Busy state output pin (Low for busy)  |

## Credits

* [Waveshare EPD driver](https://github.com/caemor/epd-waveshare)
* [SSD1306 OLED display driver](https://github.com/jamwaffles/ssd1306)
* [Sample C++ Source from STM](https://os.mbed.com/teams/ST/code/EPD_GDE021A1//file/6ee9c1afd6ec/EPD_GDE021A1.cpp)
* [GDE021A1 Specification](http://www.e-paper-display.com/GDE021A1%20V2.0%20Specification315e.pdf?method=picker&flag=all&id=cbf74932-4964-43d6-9f0c-d1c45feaec77&fileId=294&v=3.zip)