//! A simple Driver for the DGE021A1 ePaper Controller (172x72 B/W via SPI)
//! - Built using [`embedded-hal`] traits.
//! - Graphics support is added through [`embedded-graphics`] crate
//!
//! [`embedded-graphics`]: https://docs.rs/embedded-graphics/
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!
//! # Examples
//!
//! One full example can be found in
//! [the examples/ folder](https://github.com/almedso/epd-gde021a21/blob/master/examples)
//!
//! Firstly initialize the display ...
//!
//! ```rust
//!     use stm32l0xx_hal::{pac, gpio::*, prelude::*, spi::*, rcc::{Config,RccExt}};
//!     use epd_gde021A1::GDE021A1;
//!
//!     let dp = pac::Peripherals::take().unwrap();
//!     let cp = cortex_m::Peripherals::take().unwrap();
//!
//!     // Configure the clock
//!     let mut rcc = dp.RCC.freeze(Config::hsi16());
//!
//!     // Acquire the GPIOx peripheral.
//!     // This also enables the clock for GPIOx in the RCC register.
//!     let gpioa = dp.GPIOA.split(&mut rcc);
//!     let gpiob = dp.GPIOB.split(&mut rcc);
//!
//!     // Configure the pins
//!     let chip_sel = gpioa.pa15.into_push_pull_output();
//!     let data_cmd = gpiob.pb11.into_push_pull_output();
//!     let reset = gpiob.pb2.into_push_pull_output();
//!     let busy = gpiob.pb8.into_pull_up_input();
//!
//!     // Configure the SPI
//!     let mosi = gpiob.pb5;
//!     let clk = gpiob.pb3;
//!     let spi = dp.SPI1.spi((clk, NoMiso, mosi), MODE_0, 1_000_000.hz(), &mut rcc);
//!
//!     // Get the time delay
//!     let mut delay = cp.SYST.delay(rcc.clocks);
//!
//!     // Finally initialize the display structure
//!     let mut disp =  GDE021A1::new(spi, reset, Some(chip_sel), data_cmd, busy);
//!     disp.init(&mut delay).expect("could not init display");
//!
//! ```
//!
//! Secondly use the created display by writing to the RAM buffer and finally refreshing the chip.
//!
//! ```rust
//!     extern crate embedded_graphics;
//!     use embedded_graphics::{
//!         pixelcolor::BinaryColor,
//!         style::{PrimitiveStyle, TextStyle},
//!         primitives::Circle,
//!         fonts::{Font6x8, Text},
//!         prelude::*,
//!     };
//!
//!     disp.clear();  // All pixels turn white - RAM buffer only
//!
//!     // Draw a circle on the RAM buffer
//!     let elem =  Circle::new(Point::new(140, 36), 25)
//!          .into_styled(PrimitiveStyle::with_fill(BinaryColor::On));
//!     elem.draw(&mut disp);  // Draw inside the RAM buffer
//!
//!     // Draw some text
//!     let elem = Text::new("Hello Rust!", Point::new(1, 8))
//!         .into_styled(TextStyle::new(Font6x8, BinaryColor::On));
//!     elem.draw(&mut disp);  // Draw inside the RAM buffer
//!
//!     // ePaper display needs to be refreshed  - write the RAM buffer to the chip
//!     disp.refresh(&mut delay).expect("could not flush display");
//!
//! ```
//! # Features
//!
//! ## `graphics` (enabled by default)
//!
//! Enable the `graphics` feature in `Cargo.toml` to get access to features in the
//! [`embedded-graphics`] crate. This adds the `.draw()` method to the [`GDE021A1`] struct which
//! accepts any `embedded-graphics` compatible item.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [`embedded-graphics`]: https://docs.rs/embedded-graphics

#![crate_type = "lib"]
#![no_std]
#![deny(missing_docs)]

use core::convert::TryInto;
use embedded_hal::{
    blocking::delay::DelayUs, blocking::spi, digital::v2::InputPin, digital::v2::OutputPin,
};
use num_derive::ToPrimitive;

#[derive(Debug)]
/// Errors that can occur while using the display
pub enum Error<CommError, PinError> {
    /// SPI communication error
    Comm(CommError),
    /// GPIO Output error
    Pin(PinError),
}

/// GDE021A1 instructions.
#[derive(ToPrimitive)]
enum Instruction {
    GateDrivingVoltageControl = 0x03,
    DeepSleepModeDisable = 0x10,
    DateEntryModeSetting = 17,
    MasterActivation = 0x20,
    DisplayUpdateDisableRamBypass = 0x21,
    DisplayUpdateControl2 = 0x22,
    WriteRam = 0x24, // 36,
    WriteVCOMRegister = 44,
    WriteLUTRegister = 50,
    BorderWaveform = 0x3C,
    SetRamXStartEndAddress = 0x44, //68
    SetRamYStartEndAddress = 69,
    SetRamXAddressCounter = 0x4E,      // 78
    SetRamYAddressCounter = 0x4F,      // 79
    BoosterInternalFeedbackSel = 0xF0, // 240
}

#[derive(ToPrimitive, Clone, Copy)]
/// The ePaper supports four grey levels per pixel as color
pub enum Color {
    #[doc(hidden)]
    BLACK = 0b00,
    #[doc(hidden)]
    DARKGRAY = 0b01,
    #[doc(hidden)]
    LIGHTGRAY = 0b10,
    #[doc(hidden)]
    WHITE = 0b11,
}

/// Look-up table for the ePaper (90 bytes)
const WF_LUT: [u8; 90] = [
    0x82, 0x00, 0x00, 0x00, 0xAA, 0x00, 0x00, 0x00, 0xAA, 0xAA, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0x00,
    0x55, 0xAA, 0xAA, 0x00, 0x55, 0x55, 0x55, 0x55, 0xAA, 0xAA, 0xAA, 0xAA, 0x55, 0x55, 0x55, 0x55,
    0xAA, 0xAA, 0xAA, 0xAA, 0x15, 0x15, 0x15, 0x15, 0x05, 0x05, 0x05, 0x05, 0x01, 0x01, 0x01, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x41, 0x45, 0xF1, 0xFF, 0x5F, 0x55, 0x01, 0x00, 0x00, 0x00,
];

// x dimension
const WIDTH: u16 = 172;
const GEOM_WIDTH: i32 = WIDTH as i32;
// y dimension 4 Pixel are encoded by one byte
// geometrical hight is 72 (i.e. 18 * 4 Pixel)
const HEIGHT: u16 = 18;
const GEOM_HEIGHT: i32 = (4 * HEIGHT) as i32;

/// Shift pixels
fn pixel_to_byte(position: usize, color: Color) -> u8 {
    let val: u8 = color as u8;
    let p = position << 1;
    val << p
}

/// Generate and-able mask for position
fn position_mask(position: usize) -> u8 {
    let val: u8 = 0b000_0011;
    let p = position << 1;
    !(val << p)
}

/// Compute corresponding byte
fn overwrite_pixel_in_byte(byte: u8, position: usize, color: Color) -> u8 {
    let val: u8 = byte & position_mask(position);
    val | pixel_to_byte(position, color)
}

/// The ePaper GDE021A1 Driver data structure
///
/// It is composed of
/// * GPIO pins to set/read
/// * SPI interfacae to write data to
/// * a RAM buffer of 3096 byte (172x72 pixel, two bit per pixel)
///
/// Draw something to the display as well as clearing the display
/// happens in MCU RAM only. The physical display will only updated by the refresh
/// method, since writing to the chip is resource consuming (communication, energy, etc.)
///
/// Drawing must be on the RAM cache of the MCU and not on the RAM cache of the chip
/// because the manipulation of single pixels without affecting neighbor pixels must be supported
/// by the embedded graphics trait and communicating with the chip only allows updates of a set of
/// four pixels.
///
/// This driver only supports binary pixel color:
/// * *embedded_graphics::pixelcolor::BinaryColor::On* maps to *black*
/// * *embedded_graphics::pixelcolor::BinaryColor::Off* maps to *white*.
///
pub struct GDE021A1<SPI, RST, CS, DC, BSY>
where
    SPI: spi::Write<u8>,
    RST: OutputPin,
    CS: OutputPin,
    DC: OutputPin,
    BSY: InputPin,
{
    /// SPI interface
    spi: SPI,
    /// Reset pin.
    rst: RST,
    /// CS pin
    cs: Option<CS>,
    /// Busy pin
    bsy: BSY,
    /// Data Command selector pin
    dc: DC,
    // Buffer covering the RAM to drive single pixel
    buffer: [u8; (WIDTH * HEIGHT) as usize],
}

impl<SPI, RST, CS, DC, BSY, PinError, SPIError> GDE021A1<SPI, RST, CS, DC, BSY>
where
    SPI: spi::Write<u8, Error = SPIError>,
    RST: OutputPin<Error = PinError>,
    CS: OutputPin<Error = PinError>,
    DC: OutputPin<Error = PinError>,
    BSY: InputPin<Error = PinError>,
{
    /// Create a new driver instance that uses SPI connection.
    pub fn new(spi: SPI, rst: RST, cs: Option<CS>, dc: DC, bsy: BSY) -> Self {
        GDE021A1 {
            spi,
            rst,
            cs,
            dc,
            bsy,
            buffer: [0xFF; (WIDTH * HEIGHT) as usize],
        }
    }

    /// Initialize the display controller according to STM demo app
    pub fn init(&mut self, delay: &mut dyn DelayUs<u32>) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;
        self.hard_reset(delay)?;
        delay.delay_us(10_000); // 10 ms initial delay

        self.write_command(Instruction::DeepSleepModeDisable)?;
        self.write_data(0x00)?; // 0 means disable 1 would mean enable
        self.write_command(Instruction::DateEntryModeSetting)?;
        self.write_data(0x03)?;
        self.write_command(Instruction::SetRamXStartEndAddress)?;
        self.write_data(0x00)?; // RAM X address start = 00h
                                // RAM X address end = 11h (17 * 4pixels by address = 72 pixels)
        self.write_data(0x11)?;
        self.write_command(Instruction::SetRamYStartEndAddress)?;
        self.write_data(0x00)?; // RAM Y address start = 0
        self.write_data(0xAB)?; // RAM Y address end = 171
        self.write_command(Instruction::SetRamXAddressCounter)?;
        self.write_data(0x00)?;
        self.write_command(Instruction::SetRamYAddressCounter)?;
        self.write_data(0x00)?;
        self.write_command(Instruction::BoosterInternalFeedbackSel)?;
        self.write_data(0x1F)?; // internal feedback is used
        self.write_command(Instruction::DisplayUpdateDisableRamBypass)?;
        // Disable RAM bypass and set GS transition to GSA = GS0 and GSB = GS3
        self.write_data(0x03)?;
        self.write_command(Instruction::WriteVCOMRegister)?;
        self.write_data(0xA0)?;
        self.write_command(Instruction::BorderWaveform)?;
        self.write_data(0x64)?;
        self.write_command(Instruction::WriteLUTRegister)?;
        for elem in WF_LUT.iter() {
            self.write_data(*elem)?;
        }

        self.disable_cs(delay)?;
        Ok(())
    }

    /// Clear just resets the buffer to white
    pub fn clear(&mut self) {
        self.clear_with_color(Color::WHITE);
    }

    /// Flush buffer to update entire display
    ///
    /// The RAM buffer of the display on the MCU is transferred to the ePaper chip and
    /// the update command sequence instructs the chip to physically present the new content.
    /// The call is blocked until the busy pin signals completion of the operation of the physical
    /// display
    pub fn refresh(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        self.write_buffer(delay)?;

        self.enable_cs(delay)?;

        self.write_command(Instruction::DisplayUpdateDisableRamBypass)?;
        self.write_data(0x03)?; // data sequence option
        self.write_command(Instruction::DisplayUpdateControl2)?;
        self.write_data(0xC4)?; // data sequence option
        self.write_command(Instruction::MasterActivation)?;

        self.disable_cs(delay)?;
        self.busy_wait();

        self.close_charge_pump(delay)?;

        Ok(())
    }

    /// Enable the chip and plan for some delay
    fn enable_cs(&mut self, delay: &mut dyn DelayUs<u32>) -> Result<(), Error<SPIError, PinError>> {
        if let Some(cs) = self.cs.as_mut() {
            cs.set_low().map_err(Error::Pin)?;
            delay.delay_us(100);
        }
        Ok(())
    }

    /// Disable the chip after some delay
    fn disable_cs(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        if let Some(cs) = self.cs.as_mut() {
            delay.delay_us(100);
            cs.set_high().map_err(Error::Pin)?;
        }
        Ok(())
    }

    /// Reset the chip
    fn hard_reset(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        self.rst.set_low().map_err(Error::Pin)?;
        delay.delay_us(10_000);
        self.rst.set_high().map_err(Error::Pin)?;
        delay.delay_us(10_000);
        Ok(())
    }

    fn write_command(&mut self, command: Instruction) -> Result<(), Error<SPIError, PinError>> {
        self.dc.set_low().map_err(Error::Pin)?;
        self.spi.write(&[command as u8]).map_err(Error::Comm)?;
        Ok(())
    }

    fn write_data(&mut self, data: u8) -> Result<(), Error<SPIError, PinError>> {
        self.dc.set_high().map_err(Error::Pin)?;
        self.spi.write(&[data]).map_err(Error::Comm)?;
        Ok(())
    }

    /// Clear just resets the buffer to a defined color
    fn clear_with_color(&mut self, color: Color) {
        let val: u8 = pixel_to_byte(0, color);
        let val: u8 = pixel_to_byte(1, color) | val;
        let val: u8 = pixel_to_byte(2, color) | val;
        let val: u8 = pixel_to_byte(3, color) | val;
        for byte in self.buffer.iter_mut() {
            *byte = val;
        }
    }

    // Set the Display section to update
    fn set_display_window(
        &mut self,
        x_pos: u8,
        y_pos: u8,
        width: u8,
        height: u8,
    ) -> Result<(), Error<SPIError, PinError>> {
        /* Set Y position and the height */
        self.write_command(Instruction::SetRamXStartEndAddress)?;
        self.write_data(y_pos)?;
        self.write_data(height)?;
        /* Set X position and the width */
        self.write_command(Instruction::SetRamYStartEndAddress)?;
        self.write_data(x_pos)?;
        self.write_data(width)?;
        /* Set the height counter */
        self.write_command(Instruction::SetRamXAddressCounter)?;
        self.write_data(y_pos)?;
        /* Set the width counter */
        self.write_command(Instruction::SetRamYAddressCounter)?;
        self.write_data(x_pos)?;
        Ok(())
    }

    /// Write buffer
    fn write_buffer(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;

        // select the complete window and complete display buffer
        self.set_display_window(0, 0, WIDTH as u8 - 1, HEIGHT as u8 - 1)?;
        // Write the complete RAM
        self.write_command(Instruction::WriteRam)?;
        for elem in 0..self.buffer.len() {
            self.write_data(self.buffer[elem])?;
        }

        self.disable_cs(delay)?;

        Ok(())
    }

    /// Wait until busy flag is cleared by the chip
    fn busy_wait(&self) {
        while match self.bsy.is_high() {
            Ok(x) => x,
            _ => false,
        } {}
    }

    /// Flush buffer to update entire display according to manual suggestion
    fn close_charge_pump(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;

        self.write_command(Instruction::DisplayUpdateControl2)?;
        self.write_data(0x03)?; // Disable CP and disable clock signal
        self.write_command(Instruction::MasterActivation)?;

        self.disable_cs(delay)?;
        delay.delay_us(400_000);
        self.busy_wait();

        Ok(())
    }

    /// Initialize the display controller according to manual suggestion
    pub fn alt_init(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;
        self.hard_reset(delay)?;
        delay.delay_us(1_000); // 10 ms initial delay

        self.write_command(Instruction::DisplayUpdateDisableRamBypass)?;
        self.write_data(0x8F)?;
        self.write_command(Instruction::GateDrivingVoltageControl)?;
        self.write_data(0x00)?;
        self.write_command(Instruction::WriteRam)?;
        for _i in 0..5760 {
            self.write_data(0xFF)?;
        }
        self.write_data(0xF8)?;
        self.write_command(Instruction::MasterActivation)?;

        self.disable_cs(delay)?;
        Ok(())
    }
}

#[cfg(feature = "graphics")]
extern crate embedded_graphics;
#[cfg(feature = "graphics")]
use self::embedded_graphics::{drawable, geometry::Size, pixelcolor::BinaryColor, DrawTarget};

#[cfg(feature = "graphics")]
impl<SPI, CS, RST, DC, BSY, PinError, SPIError> DrawTarget<BinaryColor>
    for GDE021A1<SPI, CS, RST, DC, BSY>
where
    SPI: spi::Write<u8, Error = SPIError>,
    RST: OutputPin<Error = PinError>,
    CS: OutputPin<Error = PinError>,
    DC: OutputPin<Error = PinError>,
    BSY: InputPin<Error = PinError>,
{
    type Error = Error<SPIError, PinError>;

    fn size(&self) -> Size {
        Size::new(WIDTH.try_into().unwrap(), (4 * HEIGHT).try_into().unwrap())
    }

    fn draw_pixel(
        &mut self,
        pixel: drawable::Pixel<BinaryColor>,
    ) -> Result<(), Error<SPIError, PinError>> {
        let drawable::Pixel(coord, color) = pixel;
        if let Ok((x @ 0..=GEOM_WIDTH, y @ 0..=GEOM_HEIGHT)) = coord.try_into() {
            let c: Color = match color {
                BinaryColor::On => Color::BLACK,
                BinaryColor::Off => Color::WHITE,
            };
            // Convert to remainder for pixel position and base for y address
            let p: usize = (y % 4).try_into().unwrap();
            let p: usize = 3 - p;
            let y: usize = (y / 4).try_into().unwrap();
            // Mirror x to make text appear correctly
            let x: usize = (GEOM_WIDTH - x).try_into().unwrap();
            let byte: u8 = self.buffer[y + x * HEIGHT as usize];
            self.buffer[y + x * HEIGHT as usize] = overwrite_pixel_in_byte(byte, p, c);
        }
        Ok(())
    }
}

#[cfg(test)]

mod tests {

    use crate::{pixel_to_byte, position_mask, Color};

    #[test]
    fn it_should_convert_pixel_to_byte_at_zero_pos() {
        assert_eq!(0b0000_0000, pixel_to_byte(0, Color::BLACK));
        assert_eq!(0b0000_0001, pixel_to_byte(0, Color::DARKGRAY));
        assert_eq!(0b0000_0010, pixel_to_byte(0, Color::LIGHTGRAY));
        assert_eq!(0b0000_0011, pixel_to_byte(0, Color::WHITE));
    }

    #[test]
    fn it_should_convert_pixel_to_byte_at_one_pos() {
        assert_eq!(0b0000_0000, pixel_to_byte(1, Color::BLACK));
        assert_eq!(0b0000_0100, pixel_to_byte(1, Color::DARKGRAY));
        assert_eq!(0b0000_1000, pixel_to_byte(1, Color::LIGHTGRAY));
        assert_eq!(0b0000_1100, pixel_to_byte(1, Color::WHITE));
    }

    #[test]
    fn it_should_convert_pixel_to_byte_at_two_pos() {
        assert_eq!(0b0000_0000, pixel_to_byte(2, Color::BLACK));
        assert_eq!(0b0001_0000, pixel_to_byte(2, Color::DARKGRAY));
        assert_eq!(0b0010_0000, pixel_to_byte(2, Color::LIGHTGRAY));
        assert_eq!(0b0011_0000, pixel_to_byte(2, Color::WHITE));
    }

    #[test]
    fn it_should_convert_pixel_to_byte_at_three_pos() {
        assert_eq!(0b0000_0000, pixel_to_byte(3, Color::BLACK));
        assert_eq!(0b0100_0000, pixel_to_byte(3, Color::DARKGRAY));
        assert_eq!(0b1000_0000, pixel_to_byte(3, Color::LIGHTGRAY));
        assert_eq!(0b1100_0000, pixel_to_byte(3, Color::WHITE));
    }

    #[test]
    fn it_should_compute_a_position_mask() {
        assert_eq!(0b1111_1100, position_mask(0));
        assert_eq!(0b1111_0011, position_mask(1));
        assert_eq!(0b1100_1111, position_mask(2));
        assert_eq!(0b0011_1111, position_mask(3));
    }
}
