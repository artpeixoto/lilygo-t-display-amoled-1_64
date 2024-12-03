use embedded_hal::{
    delay::DelayNs as BDelayNs,
    digital::OutputPin,
    spi::{Operation, SpiBus, SpiDevice},
};
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use esp_hal::{
    spi::{
        master::{Address, Command, Instance as SpiMasterInstance, Spi},
        AnySpi, SpiDataMode,
    },
    Blocking,
};
use icna3311::{ErrorType, HalfDuplexSpiMode, ModalHalfDuplexSpiDevice, ModalHalfDuplexSpiDeviceTransaction};

pub struct ModalHalfDuplexEspSpi<'d, ExecMode, SpiInst, CsPin, Delay> {
    pub spi		: Spi<'d, ExecMode, SpiInst>,
    pub cs_pin	: CsPin,
    pub delay	: Delay,
}

impl<'d, ExecMode, SpiInst, CsPin, Delay> ModalHalfDuplexEspSpi<'d, ExecMode, SpiInst, CsPin, Delay> {
	pub fn new(spi: Spi<'d, ExecMode, SpiInst>, cs_pin: CsPin, delay: Delay) -> Self {
		Self { spi, cs_pin, delay }
	}
}

impl<'d, SpiInst, CsPin, Delay> 
	ModalHalfDuplexSpiDevice for ModalHalfDuplexEspSpi<'d, Blocking, SpiInst, CsPin, Delay> 
where
    SpiInst	: SpiMasterInstance,
    CsPin	: OutputPin,
    Delay	: BDelayNs,
{
	type Transaction<'a> = ModalHalfDuplexEspSpiTransaction<'a, 'd, SpiInst, CsPin, Delay> where Self: 'a;
	
	fn start_transaction<'a>(&'a mut self) -> Result<Self::Transaction<'a>, Self::Error> {
		self.cs_pin.set_low().unwrap();
		Ok( ModalHalfDuplexEspSpiTransaction{
			inner: self
		})
	}
}

pub struct ModalHalfDuplexEspSpiTransaction<'t,'d,  SpiInst, CsPin, Delay> 
where
    SpiInst	: SpiMasterInstance,
    CsPin	: OutputPin,
    Delay	: BDelayNs,
{
    inner: &'t mut ModalHalfDuplexEspSpi<'d, Blocking, SpiInst, CsPin, Delay>,
}

impl<'d, 't, SpiInst, CsPin, Delay>
    ModalHalfDuplexEspSpiTransaction<'t,'d,  SpiInst, CsPin, Delay>
where
    SpiInst	: SpiMasterInstance,
    CsPin	: OutputPin,
    Delay	: BDelayNs,
{
    fn get_esp_hal_mode(mode: HalfDuplexSpiMode) -> esp_hal::spi::SpiDataMode {
        match mode {
            HalfDuplexSpiMode::Simple => SpiDataMode::Single,
            HalfDuplexSpiMode::Dual => SpiDataMode::Dual,
            HalfDuplexSpiMode::Quad => SpiDataMode::Quad,
        }
    }
}

impl<'d, 't, SpiInst, CsPin, Delay> 
ModalHalfDuplexSpiDeviceTransaction<u8> for ModalHalfDuplexEspSpiTransaction<'t, 'd,  SpiInst, CsPin, Delay>
where
    SpiInst: SpiMasterInstance,
    CsPin: OutputPin,
    Delay: BDelayNs,
{
    type Error = <ModalHalfDuplexEspSpi<'d, Blocking, SpiInst, CsPin, Delay> as ErrorType>::Error;

    fn write(&mut self, buf: &[u8], mode: HalfDuplexSpiMode) -> Result<(), <ModalHalfDuplexEspSpi<'d, Blocking, SpiInst, CsPin, Delay> as ErrorType>::Error> {
		let mode = Self::get_esp_hal_mode(mode);
		self.inner.spi.half_duplex_write(mode, Command::None, Address::None, 0, buf)?;
		Ok(())
    }

    fn read(&mut self, buf: &mut [u8], mode: HalfDuplexSpiMode) -> Result<(), <ModalHalfDuplexEspSpi<'d, Blocking, SpiInst, CsPin, Delay> as ErrorType>::Error> {
		let mode = Self::get_esp_hal_mode(mode);
		self.inner.spi.half_duplex_read(mode, Command::None, Address::None, 0, buf)?;
		Ok(())
    }
}

impl<'d, 't, SpiInst, CsPin, Delay> 
Drop for ModalHalfDuplexEspSpiTransaction<'t, 'd,  SpiInst, CsPin, Delay>
where
    SpiInst	: SpiMasterInstance,
    CsPin	: OutputPin,
    Delay	: BDelayNs,
{
    fn drop(&mut self) {
        self.inner.cs_pin.set_high().unwrap();
    }
}

impl<'d, SpiInst, CsPin, Delay> 
ErrorType for ModalHalfDuplexEspSpi<'d, Blocking, SpiInst, CsPin, Delay>
where
    SpiInst: SpiMasterInstance,
    CsPin: OutputPin,
{
    type Error = <Spi<'d, Blocking, SpiInst> as embedded_hal::spi::ErrorType>::Error;
}
