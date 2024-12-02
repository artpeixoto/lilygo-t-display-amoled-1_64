use embedded_hal::{
    delay::DelayNs as BDelayNs,
    digital::OutputPin,
    spi::{ErrorType, Operation, SpiBus, SpiDevice},
};
use embedded_hal_bus::spi::{DeviceError, ExclusiveDevice};
use esp_hal::{
    spi::{
        master::{Address, Command, Instance as SpiMasterInstance, Spi},
        AnySpi, SpiDataMode,
    },
    Blocking,
};
use icna3311::MultiModeSpiDevice;


pub struct MultimodeEspSpi<'d, ExecMode, SpiInst, CsPin, Delay> {
 	spi		: Spi<'d, ExecMode, SpiInst>,
	cs_pin	: CsPin,
	delay 	: Delay
}

impl<'d, SpiInst, CsPin, Delay> 
	ErrorType for MultimodeEspSpi<'d, Blocking, SpiInst, CsPin, Delay>
	where SpiInst: SpiMasterInstance, CsPin: OutputPin 
{
    type Error = <Spi<'d, Blocking, SpiInst> as ErrorType>::Error;
}

impl<'d, SpiInst, CsPin, Delay> 
	SpiDevice for MultimodeOwnedEspSpi<'d, Blocking, SpiInst, CsPin, Delay>
	where SpiInst: SpiMasterInstance, CsPin: OutputPin, Delay: BDelayNs 
{
    fn transaction(&mut self, operations: &mut [Operation<'_, u8>]) -> Result<(), Self::Error> {
		self.cs_pin.set_low();
		for op in operations.iter_mut(){
			match op{
				Operation::Transfer(read_buf, write_buf) => {

				}
				Operation::Read(read_buf) 				=> {
					self.spi.read(words)	
				},
				Operation::Write(_) 			=> todo!(),
				Operation::TransferInPlace(_) 	=> todo!(),
				Operation::DelayNs(_) => todo!(),	
			}
		}
		self.cs_pin.set_high();
		Ok(())
    }
}

impl<'d, SpiInst, CsPin, Delay> 
	MultiModeSpiDevice for MultimodeOwnedEspSpi<'d, Blocking, SpiInst, CsPin, Delay>
	where 
		SpiInst	: SpiMasterInstance, 
		CsPin: OutputPin, 
		Delay: BDelayNs,
{
	fn transaction_with_mode(
		&mut self, 
		operations: &mut [(Operation<'_, u8>, icna3311::SpiMode)]
	) -> Result<(), Self::Error> {
		self.cs_pin.set_low();
		for (op, mode) in operations{
			let mode = match mode {
				icna3311::SpiMode::Simple 	=> SpiDataMode::Single,
				icna3311::SpiMode::Dual 	=> SpiDataMode::Dual,
				icna3311::SpiMode::Quad 	=> SpiDataMode::Quad,
			};			
			
		};

		Ok(())
	}
	
		// fn write_mode(&mut self, mode: &icna3311::SpiMode, data: &[u8]) -> Result<(), Self::Error> {
			// let mode = match mode {
			// 	icna3311::SpiMode::Simple 	=> SpiDataMode::Single,
			// 	icna3311::SpiMode::Dual 	=> SpiDataMode::Dual,
			// 	icna3311::SpiMode::Quad 	=> SpiDataMode::Quad,
			// };
		// 	self.spi.bus_mut().half_duplex_write(mode, Command::None, Address::None, 0, data).map_err(|e| DeviceError::Spi(e))?;
		// 	Ok(())
		// }

		// fn read_mode(&mut self, mode: &icna3311::SpiMode, buf: &mut [u8]) -> Result<(), Self::Error> {
		// 	let mode = match mode {
		// 		icna3311::SpiMode::Simple 	=> SpiDataMode::Single,
		// 		icna3311::SpiMode::Dual 	=> SpiDataMode::Dual,
		// 		icna3311::SpiMode::Quad 	=> SpiDataMode::Quad,
		// 	};
			
		// 	self.spi
		// 		.bus_mut()
		// 		.half_duplex_read(mode, Command::None, Address::None, 0, buf)
		// 		.map_err(|e| DeviceError::Spi(e))?;
		// 	Ok(())   
		// }
}
