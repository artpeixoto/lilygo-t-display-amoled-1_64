#![no_std]
#![no_main]

pub mod display_adapter;

use esp_backtrace as _;
use esp_hal::gpio::{Flex, GpioPin, Io, Level, Output, OutputPin};
use esp_hal::{delay::Delay, dma::Dma, prelude::*, spi::master::{Spi}};

use esp_hal::spi::master::Config as SpiConfig;
use esp_hal::{peripherals, prelude::*};
use fugit::{HertzU32, KilohertzU32};
use icna3311::Icna3311;

#[entry]
fn main() -> ! {
    #[allow(unused)]
    let peripherals = esp_hal::init(esp_hal::Config::default());
	let delay 		= Delay::new();
	let io 			= Io::new(peripherals.IO_MUX);

	let dma = Dma::new(peripherals.DMA);
	let spi_dma = dma.channel0.configure(true, esp_hal::dma::DmaPriority::Priority4);
	let spi = 
		Spi::new(peripherals.SPI2)
			.with_mosi(Flex::new(peripherals.GPIO11))
			.with_miso(Flex::new(peripherals.GPIO13))
			.with_sck(Output::new(peripherals.GPIO12, Level::Low))
			.with_cs(Output::new(peripherals.GPIO10, Level::Low))
		;
	let rst_pin = Output::new(peripherals.GPIO17, Level::Low);
	let en_pin  = Output::new(peripherals.GPIO16, Level::High);

	let display = Icna3311::new(spi, en_pin, rst_pin);

	// let spi_modifier = 
	// 		.with_sio2(Flex::new(peripherals.GPIO14))
	// 		.with_sio3(Flex::new(peripherals.GPIO15))
	


    // esp_println::logger::init_logger_from_env();

    loop {
        // log::info!("Hello world!");
        delay.delay(500.millis());
    }
}
