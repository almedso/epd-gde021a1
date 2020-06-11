//! A simple Driver for the DGE021A1 ePaper Controller (172x72 B/W via SPI)
//! - Built using [`embedded-hal`] traits.
//! - Graphics support is added through [`embedded-graphics`]
//!
//! [`embedded-graphics`]: https://docs.rs/embedded-graphics/
//! [`embedded-hal`]: https://docs.rs/embedded-hal
//!

//!
//! # Example
//!
//!```rust, no_run
//!# use embedded_hal_mock::*;
//!# fn main() -> Result<(), MockError> {
//!use embedded_graphics::{
//!    pixelcolor::BinaryColor::On as Black, prelude::*, primitives::Line, style::PrimitiveStyle,
//!};
//!use epd_gpd021a1::{prelude::*};
//!#
//!# let expectations = [];
//!# let mut spi = spi::Mock::new(&expectations);
//!# let expectations = [];
//!# let cs_pin = pin::Mock::new(&expectations);
//!# let busy_in = pin::Mock::new(&expectations);
//!# let dc = pin::Mock::new(&expectations);
//!# let rst = pin::Mock::new(&expectations);
//!# let mut delay = delay::MockNoop::new();
//!
//!// Setup EPD
//!let mut epd = GPD021A1::new(&mut spi, cs_pin, busy_in, dc, rst, &mut delay)?;
//!
//!// Use embedded graphics for drawing a line
//!let _ = Line::new(Point::new(0, 120), Point::new(0, 295))
//!    .into_styled(PrimitiveStyle::with_stroke(Black, 1))
//!    .draw(&mut display);
//!
//!    // Display updated frame
//!epd.update_frame(&mut spi, &display.buffer())?;
//!epd.display_frame(&mut spi)?;
//!
//!// Set the EPD to sleep
//!epd.sleep(&mut spi)?;
//!# Ok(())
//!# }
//!```
//!

#![no_std]
#![allow(missing_docs)]
#[cfg(feature = "graphics")]

use num_derive::ToPrimitive;
use core::convert::TryInto;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::InputPin;

#[derive(Debug)]
pub enum Error<CommError, PinError> {
    Comm(CommError),
    Pin(PinError),
}

/// GDE021A1 instructions.
#[derive(ToPrimitive)]
enum Instruction {
    DeepSleepModeDisable = 16,
    DateEntryModeSetting = 17,
    MasterActivation = 32,
    DisableRamBypass = 33,
    DisplayUpdateControl = 34,
    WriteVCOMRegister = 44,
    WriteLUTRegister = 50,
    BorderWaveform = 60,
    SetRamXStartEndAddress = 68,
    SetRamYStartEndAddress = 69,
    SetRamXAddressCounter = 78,
    SetRamYAddressCounter = 79,
    BoosterInternalFeedbackSel = 240,
}

#[derive(ToPrimitive)]
pub enum Color {
    BLACK = 0x00,
    DARKGRAY = 0x55,
    LIGHTGRAY = 0xAA,
    WHITE = 0xFF
}


/* Look-up table for the epaper (90 bytes) */
const WF_LUT: [u16; 90 ]= [
  0x82,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,
  0xAA,0xAA,0x00,0x00,0xAA,0xAA,0xAA,0x00,
  0x55,0xAA,0xAA,0x00,0x55,0x55,0x55,0x55,
  0xAA,0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,
  0xAA,0xAA,0xAA,0xAA,0x15,0x15,0x15,0x15,
  0x05,0x05,0x05,0x05,0x01,0x01,0x01,0x01,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x41,0x45,0xF1,0xFF,0x5F,0x55,0x01,0x00,
  0x00,0x00,
];

const WIDTH: i32 = 172;
const HEIGHT: i32 = 72;

pub struct GDE021A1<SPI, RST, CS, DC, BSY>
where
    SPI: spi::Write<u8>,
    RST: OutputPin,
    CS: OutputPin,
    DC: OutputPin,
    BSY: InputPin,
{
    /// SPI pin
    spi: SPI,

    /// Reset pin.
    rst: RST,

    /// CS pin
    cs: Option<CS>,

    /// Busy pin
    bsy: BSY,

    /// Data Command selector pin
    dc: DC

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
        }
    }

    fn enable_cs(&mut self, delay: &mut dyn DelayUs<u32>) -> Result<(), Error<SPIError, PinError>> {
        if let Some(cs) = self.cs.as_mut() {
            cs.set_high().map_err(Error::Pin)?;
            delay.delay_us(1);
        }
        Ok(())
    }

    fn disable_cs(
        &mut self,
        delay: &mut dyn DelayUs<u32>,
    ) -> Result<(), Error<SPIError, PinError>> {
        if let Some(cs) = self.cs.as_mut() {
            delay.delay_us(1);
            cs.set_high().map_err(Error::Pin)?;
        }
        Ok(())
    }

    /// Initialize the display controller
    pub fn init(&mut self, delay: &mut dyn DelayUs<u32>) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;
        self.hard_reset(delay)?;
        delay.delay_us(10 * 1000);  // 10 ms initial delay

        self.write_command(Instruction::DeepSleepModeDisable)?;
        self.write_data(0x00)?;
        self.write_command(Instruction::DateEntryModeSetting)?;
        self.write_data(0x03)?;
        self.write_command(Instruction::SetRamXStartEndAddress)?;
        self.write_data(0x00)?;  /* RAM X address start = 00h */
        self.write_data(0x11)?;  /* RAM X address end = 11h (17 * 4pixels by address = 72 pixels) */
        self.write_command(Instruction::SetRamYStartEndAddress)?;
        self.write_data(0x00)?;   /* RAM Y address start = 0 */
        self.write_data(0xAB)?;   /* RAM Y address end = 171 */
        self.write_command(Instruction::SetRamXAddressCounter)?;
        self.write_data(0x00)?;
        self.write_command(Instruction::SetRamYAddressCounter)?;
        self.write_data(0x00)?;
        self.write_command(Instruction::BoosterInternalFeedbackSel)?;
        self.write_data(0x1F)?;
        self.write_command(Instruction::DisableRamBypass)?;
        self.write_data(0x03)?;
        self.write_command(Instruction::WriteVCOMRegister)?;
        self.write_data(0xA0)?;
        self.write_command(Instruction::BorderWaveform)?;
        self.write_data(0x64)?;
        self.write_command(Instruction::WriteLUTRegister)?;
        for i in 0..90 { self.write_data(WF_LUT[i])?; }
        self.disable_cs(delay)?;
        Ok(())
    }

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
        self.spi.write(&[command as u8 ]).map_err(Error::Comm)?;
        Ok(())
    }

    fn write_data(&mut self, data: u16) -> Result<(), Error<SPIError, PinError>> {
        self.dc.set_high().map_err(Error::Pin)?;
        let low = (data & 0xFF00 ) as u8;
        let high = (data >> 8) as u8;
        self.spi.write(&[high, low]).map_err(Error::Comm)?;
        Ok(())
    }


    pub fn clear(&mut self, delay: &mut dyn DelayUs<u32>) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;
        self.write_rectangle(0, 0, WIDTH as u16, HEIGHT as u16, Color::WHITE as u16)?;
        self.disable_cs(delay)?;
        Ok(())
    }

    fn write_pixel(&mut self, color: u16) -> Result<(), Error<SPIError, PinError>>{
        self.write_command(Instruction::BorderWaveform)?;
        self.write_data(color)?;
        Ok(())
    }

    fn set_display_window(&mut self, x_pos: u16 , y_pos: u16, width: u16, height: u16)
    -> Result<(), Error<SPIError, PinError>> {
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

    /// Flush buffer to update entire display
    pub fn flush(&mut self, delay: &mut dyn DelayUs<u32>) -> Result<(), Error<SPIError, PinError>> {
        self.enable_cs(delay)?;

        self.write_command(Instruction::MasterActivation)?;
        self.write_data(0xC4)?;  // data sequence option
        self.write_command(Instruction::DisplayUpdateControl)?;

        self.disable_cs(delay)?;
        self.busy_wait();
        self.hard_reset(delay)?;

        Ok(())
    }

    fn write_rectangle(&mut self,
        x1: u16, y1: u16, x2: u16, y2: u16,
        color: u16,
    ) -> Result<(), Error<SPIError, PinError>>
    {

        self.set_display_window(x1, y1, x2 -1 ,  y2 -1)?;
        for _index in 0..((x2 - x1) * (y2 - y1)) {
            self.write_pixel(color)?;
        }
        Ok(())
    }

    fn busy_wait(&self) {
        while match self.bsy.is_high() {
            Ok(x) => x,
            _ => false,
        } {}
    }

}


#[cfg(feature = "graphics")]
extern crate embedded_graphics;
#[cfg(feature = "graphics")]
use self::embedded_graphics::{
    drawable,
    pixelcolor::BinaryColor,
    DrawTarget,
    geometry::Size,
};


#[cfg(feature = "graphics")]
impl<SPI, CS, RST, DC, BSY, PinError, SPIError> DrawTarget<BinaryColor> for GDE021A1 <SPI, CS, RST, DC, BSY>
where
    SPI: spi::Write<u8, Error = SPIError>,
    RST: OutputPin<Error = PinError>,
    CS: OutputPin<Error = PinError>,
    DC: OutputPin<Error = PinError>,
    BSY: InputPin<Error = PinError>,
{
    // type Error = core::convert::Infallible;
    type Error = Error<SPIError, PinError>;

    fn size(&self) -> Size {
        Size::new(WIDTH.try_into().unwrap(), HEIGHT.try_into().unwrap())
    }

    fn draw_pixel(&mut self, pixel: drawable::Pixel<BinaryColor>) -> Result<(), Self::Error> {
        let drawable::Pixel(coord, color) = pixel;
        if let Ok((x @ 0..=WIDTH, y @ 0..=HEIGHT)) = coord.try_into() {
            let c: u16 = match color {
                BinaryColor::On => Color::BLACK as u16,
                BinaryColor::Off => Color::WHITE as u16,
            };
            self.write_rectangle(x as u16, y as u16, x as u16 + 1 , y as u16 + 1, c )?;
        }
        Ok(())
    }
}

