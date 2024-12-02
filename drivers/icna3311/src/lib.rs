#![no_std]

use core::{
    fmt::Display,
    marker::{Copy, PhantomData},
};
use embedded_graphics::{
    pixelcolor::{Gray8, Rgb666, Rgb888},
    prelude::{Dimensions, DrawTarget, GrayColor, PixelColor, Point, RgbColor, Size},
    primitives::Rectangle,
    Pixel,
};
use embedded_hal::{
    digital::OutputPin,
    spi::{Operation, SpiDevice, MODE_0},
};
use fixed::types::U0F8;

pub struct Icna3311<Spi, EnPin, RstPin, PixColor> {
    spi: Spi,
    en_pin: EnPin,
    rst_pin: RstPin,
    mode: HalfDuplexSpiMode,
    color: PhantomData<PixColor>,
}

pub const WIDTH: usize = 280;
pub const HEIGHT: usize = 456;

pub enum DisplayError<SpiErr> {
    OutOfBoundsError,
    SpiError(SpiErr),
}

pub enum Icna3311ColorFormat {
    // Rgb111,
    // Rgb332,
    Rgb565,
    Rgb666,
    Rgb888,
    Gray8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ColorDir {
    Rgb,
    Bgr,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Dir {
    Increasing,
    Decreasing,
}
pub trait Icna3311PixelColor: PixelColor {
    const COLOR_FORMAT: Icna3311ColorFormat;
    fn as_bytes(iter: impl Iterator<Item = Self>) -> impl Iterator<Item = u8>;
}

impl Icna3311PixelColor for Rgb888 {
    const COLOR_FORMAT: Icna3311ColorFormat = Icna3311ColorFormat::Rgb888;

    fn as_bytes(iter: impl Iterator<Item = Self>) -> impl Iterator<Item = u8> {
        iter.flat_map(|c| [c.r(), c.g(), c.b()])
    }
}
impl Icna3311PixelColor for Rgb666 {
    const COLOR_FORMAT: Icna3311ColorFormat = Icna3311ColorFormat::Rgb666;

    fn as_bytes(iter: impl Iterator<Item = Self>) -> impl Iterator<Item = u8> {
        iter.flat_map(|c| [c.r(), c.g(), c.b()])
    }
}

// impl Icna3111PixelColor for Rgb565{
// 	const COLOR_FORMAT: Icna3111ColorFormat = Icna3111ColorFormat::Rgb565;

// 	fn as_bytes(iter: impl Iterator<Item=Self>) -> impl Iterator<Item=u8> {
// 		todo!()
// 	}
// }
impl Icna3311PixelColor for Gray8 {
    const COLOR_FORMAT: Icna3311ColorFormat = Icna3311ColorFormat::Gray8;

    fn as_bytes(iter: impl Iterator<Item = Self>) -> impl Iterator<Item = u8> {
        iter.map(|c| c.luma())
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum HalfDuplexSpiMode {
    #[default]
    Simple,
    Dual,
    Quad,
}

pub trait ErrorType {
    type Error;
}
pub trait ModalHalfDuplexSpiDeviceTransaction<Word, Error>{
    fn write(&mut self  , buf: &[Word]      , mode: HalfDuplexSpiMode) -> Result<(), Error>; 
    fn read(&mut self   , buf: &mut [Word]  , mode: HalfDuplexSpiMode) -> Result<(), Error>;
}

pub trait HalfDuplexSpiDevice<Word=u8>: ErrorType
where
    Word: Copy + 'static,
{
    type Transaction<'a>: ModalHalfDuplexSpiDeviceTransaction<Word, Self::Error> + 'a where Self: 'a;
    fn start_transaction<'a>(&'a mut self) -> Result<Self::Transaction<'a>, Self::Error>;
}

impl<Spi, EnPin, RstPin, PixColor> Icna3311<Spi, EnPin, RstPin, PixColor>
where
    Spi: HalfDuplexSpiDevice<u8>, //SpiDevice<u1> ?
    EnPin: OutputPin,
    RstPin: OutputPin,
{
    pub fn new(spi: Spi, en_pin: EnPin, rst_pin: RstPin) -> Self {
        Self {
            spi,
            en_pin,
            rst_pin,
            mode: HalfDuplexSpiMode::Simple,
            color: PhantomData,
        }
    }
    pub fn change_spi_mode<NewSpi: SpiDevice<u8>>(
        mut self,
        spi_mode: HalfDuplexSpiMode,
        spi_modifier: impl FnOnce(Spi) -> NewSpi,
    ) -> Icna3311<NewSpi, EnPin, RstPin, PixColor> {
        let enter_mode_cmd = match &spi_mode {
            &HalfDuplexSpiMode::Simple => 0xff,
            &HalfDuplexSpiMode::Dual => 0x3b,
            &HalfDuplexSpiMode::Quad => 0x38,
        };

        self.cmd(enter_mode_cmd, &[])
            .map_err(|_| panic!("Something bad happened when trying to change spi mode"))
            .unwrap();

        let new_spi = spi_modifier(self.spi);

        Icna3311 {
            spi: new_spi,
            color: self.color,
            en_pin: self.en_pin,
            rst_pin: self.rst_pin,
            mode: spi_mode,
        }
    }
    pub fn enable(&mut self) {
        self.en_pin.set_high().unwrap();
    }
    pub(crate) fn cmd(&mut self, addr: u8, param_data: impl Iterator<Item=u8>) -> Result<(), DisplayError<Spi::Error>> {
        
        let mut transaction = self.spi.start_transaction().map_err(DisplayError::SpiError)?;
        let cmd_data = [0x02, 0x00, addr, 0x00];
        transaction.write(&cmd_data, self.mode).map_err(DisplayError::SpiError)?;
        for param_byte in param_data{
            transaction.write(&[param_byte],self.mode).map_err(DisplayError::SpiError)?;
        }
        
        Ok(())
    }

    pub(crate) fn read<const LEN: usize>(
        &mut self,
        addr: u8,
    ) -> Result<[u8; LEN], DisplayError<Spi::Error>> {
        let transaction = self.spi.start_transaction();
        let cmd_data = [0x03, 0x00, addr, 0x00];
        
        let mut res_buf = [0_u8; LEN];
        let mut transactions = [
            (Operation::Write(&cmd_data), self.mode),
            (Operation::Read(&mut res_buf), self.mode),
        ];
        self.spi
            .transaction_with_mode(&mut transactions)
            .map_err(DisplayError::SpiError)?;
        Ok(res_buf)
    }

    pub fn verify_bounds(&self, point: &Point) -> Result<(), DisplayError<Spi::Error>> {
        if (point.x >= 0)
            && (point.y >= 0)
            && ((point.x as usize) < WIDTH)
            && ((point.y as usize) < HEIGHT)
        {
            Ok(())
        } else {
            Err(DisplayError::OutOfBoundsError)
        }
    }
    pub fn verify_rect_in_bounds(&self, rect: &Rectangle) -> Result<(), DisplayError<Spi::Error>> {
        self.verify_bounds(&rect.top_left)?;
        self.verify_bounds(&rect.bottom_right().unwrap())?;
        Ok(())
    }

    pub fn set_memory_data_access_control(
        &mut self,
        row_dir: Dir,
        col_dir: Dir,
        color_ord: ColorDir,
    ) -> Result<(), DisplayError<Spi::Error>> {
        let parm_data = {
            let mut parm_data: u8 = 0;
            if row_dir == Dir::Decreasing {
                parm_data |= 0b1000_0000;
            }
            if col_dir == Dir::Decreasing {
                parm_data |= 0b0100_0000;
            }
            if color_ord == ColorDir::Bgr {
                parm_data |= 0b0000_1000
            }
            [parm_data]
        };
        self.cmd(0x36, &parm_data)
    }
    pub fn set_pixel_format(
        &mut self,
        pixel_format: Icna3311ColorFormat,
    ) -> Result<(), DisplayError<Spi::Error>> {
        let parm = {
            let ifpf: u8 = match pixel_format {
                Icna3311ColorFormat::Gray8 => 0b001,
                // Icna3111ColorFormat::Rgb332 => 0b010,
                // Icna3111ColorFormat::Rgb111 => 0b011,
                Icna3311ColorFormat::Rgb565 => 0b101,
                Icna3311ColorFormat::Rgb666 => 0b110,
                Icna3311ColorFormat::Rgb888 => 0b111,
            };
            [0b0000_0_000 | ifpf]
        };
        self.cmd(0x3a, &parm)?;
        Ok(())
    }

    pub fn set_display_on(&mut self, set_on: bool) -> Result<(), DisplayError<Spi::Error>> {
        if set_on {
            self.cmd(0x29, &[])?;
        } else {
            self.cmd(0x28, &[])?;
        }
        Ok(())
    }

    pub fn set_brightness(&mut self, brightness: U0F8) -> Result<(), DisplayError<Spi::Error>> {
        let brightness_byte = brightness.to_be_bytes();
        self.cmd(0x51, &brightness_byte)?;
        Ok(())
    }

    pub(crate) fn draw_rect(
        &mut self,
        rect: &Rectangle,
        raw_data: impl Iterator<Item = u8>,
    ) -> Result<(), DisplayError<Spi::Error>> {
        self.verify_rect_in_bounds(&rect)?;
        {   // first we set the rect to write to, using CASET and RASET (refer to datasheet)
            // the data gets written in a funky way. This function takes care of translating it correctly
            let get_coords_bytes = |start_coord: i32, end_coord: i32| -> [u8; 4] {
                let get_coord_bits = |coord: i32| -> [u8; 2] {
                    let coord = coord as u32;
                    let bottom_bits = (coord & 0xff) as u8;
                    let top_bits = ((coord >> 8) & 0b0000_0011) as u8;
                    [top_bits, bottom_bits]
                };
                let start_bytes = get_coord_bits(start_coord);
                let end_bytes = get_coord_bits(end_coord);
                [start_bytes[0], start_bytes[1], end_bytes[0], end_bytes[1]]
            };

            // set col addresses (CASET)
            let col_addresses_bits =
                get_coords_bytes(rect.top_left.x, rect.bottom_right().unwrap().x);
            self.cmd(0x2a, &col_addresses_bits)?;

            // set row addresses (RASET)
            let row_addresses_bits =
                get_coords_bytes(rect.top_left.y, rect.bottom_right().unwrap().y);
            self.cmd(0x2b, &row_addresses_bits)?;
        }
        // then we write the pixel data (RAMWR).
        self.cmd(0x2C, &[])?;
        for byte in raw_data {
            self.spi.write(&[byte]).map_err(DisplayError::SpiError)?;
        }
        Ok(())
    }

    // pub fn init<C: Icna3311PixelColor>(mut self) -> Icna3311<C, Spi, EnPin, RstPin> {
    //     self.en_pin.set_high().unwrap();

    //     let Ok(_) = self.set_pixel_format(C::COLOR_FORMAT) else {
    //         panic!("Couldn't set pixel format")
    //     };

    //     let Ok(_) =
    //         self.set_memory_data_access_control(Dir::Increasing, Dir::Increasing, ColorDir::Rgb)
    //     else {
    //         panic!("Couldn't set memory access control")
    //     };

    // }
}

impl<Spi, EnPin, RstPin, PixColor> DrawTarget for Icna3311<Spi, EnPin, RstPin, PixColor>
where
    PixColor: Icna3311PixelColor,
    Spi: HalfDuplexSpiDevice,
    EnPin: OutputPin,
    RstPin: OutputPin,
{
    type Color = PixColor;

    type Error = DisplayError<Spi::Error>;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        // this is not the most efficient method, but it is hard to do so
        // without eating up too much memory or using alloc or some other funky stuff.
        for p in pixels {
            self.fill_contiguous(
                &Rectangle {
                    top_left: p.0,
                    size: Size::new_equal(1),
                },
                core::iter::once(p.1),
            )?;
        }
        Ok(())
    }
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.draw_rect(area, PixColor::as_bytes(colors.into_iter()))?;
        Ok(())
    }
}

impl<Spi, EnPin, RstPin, PixColor> Dimensions for Icna3311<Spi, EnPin, RstPin, PixColor>
where
    PixColor: Icna3311PixelColor,
    Spi: HalfDuplexSpiDevice,
    EnPin: OutputPin,
    RstPin: OutputPin,
{
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(Point::zero(), Size::new(WIDTH as u32, HEIGHT as u32))
    }
}
