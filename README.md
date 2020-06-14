[![Build Status](https://travis-ci.com/almedso/epd-gde021a1.svg?branch=master)](https://travis-ci.com/caemor/epd-waveshare)

This library contains a driver for E-Paper Modules with Controller GDE021A1 ((172x72 B/W Pixel via SPI)).

It uses the [embedded graphics](https://crates.io/crates/embedded-graphics) library for the optional graphics support.

A 2018-edition compatible version (Rust 1.31+) is needed.

## Resource Needs

* The GED021A1 ePaper Display is Black/White (4 gray levels per pixel)
* Each Pixel value is encoded by 2 bit.
* Before the ePaper display is refreshed all data is written into RAM.
  A RAM cache storage of (2 bit per 172 x 72 Pixels equals to) 3096 Bytes is
  mandatory.
* Besides the RAM only the typical SPI peripherals (clock, CLK pin MOSI pin,) and
  additional arbitrary GPIO pins (Data/Command, Busy, Reset) are needed.

## Usage


For example, the initialize sequence is like (on a Cortex MCU, using the
[embedded hal](https://crates.io/crates/embedded-hal) crate)

Set the default compile target in e.g *.cargo/config* file to
```config
[build]
target = "thumbv6m-none-eabi"    # Cortex-M0 and Cortex-M0+
```
in order to support Cortex-M0.


```Rust

    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure the clock.
    let mut rcc = dp.RCC.freeze(Config::hsi16());

    // Acquire the GPIOx peripheral.
    // This also enables the clock for GPIOx in the RCC register.
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    // The GPIO's
    let chip_sel = gpioa.pa15.into_push_pull_output();
    let data_cmd = gpiob.pb11.into_push_pull_output();
    let reset = gpiob.pb2.into_push_pull_output();
    let busy = gpiob.pb8.into_pull_up_input();

    // The SPI
    let mosi = gpiob.pb5;
    let clk = gpiob.pb3;
    let spi = dp.SPI1.spi((clk, NoMiso, mosi), MODE_0, 1_000_000.hz(), &mut rcc);

    // the time delay
    let mut delay = cp.SYST.delay(rcc.clocks);

    // and finally the display structure
    let mut disp =  GDE021A1::new(spi, reset, Some(chip_sel), data_cmd, busy);
    // initialize the display
    disp.init(&mut delay).expect("could not init display");

```

Once the display is initialized it can be used using the
[embedded graphics](https://crates.io/crates/embedded-graphics) library like
e.g. like

```Rust
    // all pixels turn white
    disp.clear();

    // draw some fancy stuff and graphics via embedded graphics api
    let elem =  Circle::new(Point::new(140, 36), 25)
         .into_styled(PrimitiveStyle::with_fill(BinaryColor::On));
    elem.draw(&mut disp);
    let elem = Text::new("Hello Rust!", Point::new(1, 8))
        .into_styled(TextStyle::new(Font6x8, BinaryColor::On));
    elem.draw(&mut disp);

    // refresh the epaper
    disp.refresh(&mut delay).expect("could not flush display");

```

### Testing

The embedded target is most likely very different from the host.
and pre-configured in your *.cargo/config* file.
However, test (specifically in CI environment run on host)

Thus, tests can savely run by specifying your host target like

```shell
cargo test --target=x86_64-unknown-linux-gnu
```

### Interface

| Interface | Description |
| :---: |  :--- |
| DIN   | 	SPI MOSI |
| CLK   | 	SPI SCK |
| CS    | 	SPI chip select (Low active)- optional |
| DC    | 	Data/Command control pin (High for data, and low for command) |
| RST   | 	External reset pin (Low for reset) |
| BUSY  | 	Busy state output pin (Low for busy)  |

## Credits

* [Waveshare EPD driver](https://github.com/caemor/epd-waveshare)
* [SSD1306 OLED display driver](https://github.com/jamwaffles/ssd1306)
* [Sample C++ Source from STM](https://os.mbed.com/teams/ST/code/EPD_GDE021A1//file/6ee9c1afd6ec/EPD_GDE021A1.cpp)
* [GDE021A1 Specification](http://www.e-paper-display.com/GDE021A1%20V2.0%20Specification315e.pdf?method=picker&flag=all&id=cbf74932-4964-43d6-9f0c-d1c45feaec77&fileId=294&v=3.zip)
* [GxEPD in C++](https://github.com/ZinggJM/GxEPD)