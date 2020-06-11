[![Build Status](https://travis-ci.com/almedso/epd-gde021a1.svg?branch=master)](https://travis-ci.com/caemor/epd-waveshare)

This library contains a driver for E-Paper Modules with Controller GDE021A1 ((172x72 B/W Pixel via SPI)).

It uses the [embedded graphics](https://crates.io/crates/embedded-graphics) library for the optional graphics support.

A 2018-edition compatible version (Rust 1.31+) is needed.

Other similiar libraries with support for much more displays are [u8g2](https://github.com/olikraus/u8g2) and [GxEPD](https://github.com/ZinggJM/GxEPD) for arduino.

## Examples

There are multiple examples in the examples folder. Use `cargo run --example example_name` to try them.

```Rust
// Setup the epd
let mut epd = EPD4in2::new(&mut spi, cs, busy, dc, rst, &mut delay)?;

// Setup the graphics
let mut display = Display4in2::default();

// Draw some text
display.draw(
    let _ = Text::new("Hello Rust!", Point::new(x, y))
        .into_styled(text_style!(
            font = Font12x16,
            text_color = Black,
            background_color = White
        ))
        .draw(display);
);

// Transfer the frame data to the epd and display it
epd.update_and_display_frame(&mut spi, &display.buffer())?;
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
| PWR   |   Power pin

## Credits

* [Waveshare EPD driver](https://github.com/caemor/epd-waveshare)
* [SSD1306 OLED display driver](https://github.com/jamwaffles/ssd1306)