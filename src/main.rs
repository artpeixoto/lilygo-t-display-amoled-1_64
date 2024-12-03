#![no_std]
#![no_main]

pub mod display_adapter;

use display_adapter::blocking::ModalHalfDuplexEspSpi;
use embedded_graphics::mono_font::ascii::{FONT_6X13_BOLD, FONT_9X18};
use embedded_graphics::mono_font::iso_8859_13::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb888;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::{geometry, prelude::*};
use embedded_graphics::text::renderer::CharacterStyle;
use embedded_graphics::text::Text;
use embedded_hal::i2c::SevenBitAddress;
use esp_backtrace as _;
use esp_hal::gpio::{Flex, GpioPin, Input, Io, Level, Output, OutputPin, Pull};
use esp_hal::i2c::master::{Config as I2cConfig, I2c};
use esp_hal::{delay::Delay, dma::Dma, prelude::*, spi::master::Spi};

use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::{peripherals, prelude::*};
use ft3168::{Ft3168, Touch};
use fugit::{HertzU32, Instant, KilohertzU32};
use icna3311::{HalfDuplexSpiMode, Icna3311};
use pipe::Pipeable;

#[entry]
fn main() -> ! {
    #[allow(unused)]
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    let io = Io::new(peripherals.IO_MUX);

    let dma = Dma::new(peripherals.DMA);
    let spi_dma = dma
        .channel0
        .configure(true, esp_hal::dma::DmaPriority::Priority4);
    let spi = Spi::new(peripherals.SPI2)
        .with_mosi(Flex::new(peripherals.GPIO11))
        .with_miso(Flex::new(peripherals.GPIO13))
        .with_sck(Output::new(peripherals.GPIO12, Level::Low));
    let spi_cs = Output::new(peripherals.GPIO10, Level::High);

    let spi = ModalHalfDuplexEspSpi::new(spi, spi_cs, Delay::new());

    let rst_pin = Output::new(peripherals.GPIO17, Level::Low);
    let en_pin = Output::new(peripherals.GPIO16, Level::High);
    let mut display = Icna3311::new(spi, en_pin, rst_pin)
        .with_spi_mode(HalfDuplexSpiMode::Quad, |x| ModalHalfDuplexEspSpi {
            spi: x
                .spi
                .with_sio2(Flex::new(peripherals.GPIO14))
                .with_sio3(Flex::new(peripherals.GPIO15)),
            ..x
        })
        .with_pixel_format::<Rgb888>()
        .map_err(|e| panic!())
        .unwrap();

    display.enable();
    display.fill_solid(&display.bounding_box(), RgbColor::WHITE);

    let touch_i2c = 
		I2c::new(
			peripherals.I2C1,
			I2cConfig {
				frequency: HertzU32::kHz(100),
				..Default::default()
			},
		)
		.with_sda(Flex::new(peripherals.GPIO7))
		.with_scl(Flex::new(peripherals.GPIO6));
	let touch_int_pin = Input::new(peripherals.GPIO9, Pull::Up);
	// let touch_rst_pin = Output::new(peripherals.GPIO8, Level::Low);

    let mut touch_sensor = Ft3168::new(
		touch_i2c, 
		touch_int_pin, 
		0x38,
		// touch_rst_pin, 
		|| (esp_hal::time::now().ticks() / 1_000).pipe(|x| Instant::<u32, 1, 1000>::from_ticks(x as u32))
	);

    Text::new(
			"Hello",
			Point::new(10, 30),
			MonoTextStyleBuilder
				::new()
				.font(&FONT_10X20)
				.text_color(RgbColor::BLACK)
				.build(),
		)
		.draw(&mut display)
		.map_err(|e| panic!())
		.unwrap();

	display.fill_solid(&Rectangle::with_center(Point::new(110, 228), Size::new_equal(30)), Rgb888::RED).map_err(|x| panic!()).unwrap();

	display.fill_solid(&Rectangle::with_center(Point::new(140, 228), Size::new_equal(30)), Rgb888::GREEN)
		.map_err(|x| panic!()).unwrap();

	display.fill_solid(&Rectangle::with_center(Point::new(170, 228), Size::new_equal(30)), Rgb888::BLUE)
		.map_err(|x| panic!()).unwrap();

    Text::new(
			"The green square should be in \nthe center of the screen",
			Point::new(10, 250),
			MonoTextStyleBuilder
				::new()
				.font(&FONT_6X13_BOLD)
				.text_color(RgbColor::BLACK)
				.build(),
		)
		.draw(&mut display)
		.map_err(|e| panic!())
		.unwrap();

    loop {
		if let Ok(Some( touches )) = touch_sensor.get_touches(){
			Text::new(
				"There are touches.\n I won't elaborate further.",
				Point::new(12, 42), 
				MonoTextStyleBuilder
					::new()
					.font(&FONT_9X18)
					.text_color(RgbColor::BLACK)
					.build()
			)
			.draw(&mut display)
			.map_err(|e| panic!())
			.unwrap();


		}
        // log::info!("Hello world!");
        delay.delay(100.millis());
    }
}
