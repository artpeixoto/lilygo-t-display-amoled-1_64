#![no_std]
use embedded_hal::{digital::InputPin, i2c::{AddressMode, I2c, SevenBitAddress}} ;
use fugit::{Instant, MillisDurationU32, TimerInstantU32};
use heapless::Vec;
use nalgebra::Vector2;
use ux::{u2, u4};

pub struct Ft3168<I2cT, IntPin, TimeGetter> 
	// where I2cT: I2c, IntPin: InputPin 
{
	i2c						: I2cT,
	int_pin					: IntPin, 
	i2c_addr				: SevenBitAddress,
	last_touch_read_instant	: Option<Instant<u32, 1, 1_000>>,// each tick is 1 millisecond
	time_getter 			: TimeGetter,
}

pub enum WorkMode{
	Working,
	Testing,
}


impl<I2cT, IntPin, TimeGetter> Ft3168<I2cT, IntPin, TimeGetter>
{
	pub fn new(i2c: I2cT, int_pin: IntPin, i2c_addr: SevenBitAddress, time_getter: TimeGetter) -> Self {
		Self { i2c, int_pin, i2c_addr, last_touch_read_instant: None, time_getter }
	}
}
impl<I2cT, IntPin, TimeGetter> Ft3168<I2cT, IntPin, TimeGetter>
where 
	I2cT		: I2c,
	IntPin		: InputPin,
	TimeGetter	: FnMut() -> Instant<u32, 1,1_000>  
{
	fn write_const_len<const LEN: usize>(&mut self, addr: u8, data: impl Iterator<Item=u8>) -> Result<(), Ft3168Error<I2cT::Error>>{
		let data = {
			let mut data_buf: [u8; LEN] = [0;LEN];
			data_buf[0] = addr;
			for (ix, data_byte) in (1..).zip(data){
				data_buf[ix] = data_byte;
			}
			data_buf
		};

		self.i2c.write(self.i2c_addr.clone(), &data).map_err(Ft3168Error::I2cError)?;
		
		Ok(())
	}

	fn read_const_len<const LEN: usize>(&mut self, addr: u8) -> Result<[u8; LEN],Ft3168Error<I2cT::Error>>{
		self.i2c.write(self.i2c_addr.clone(), &[addr]).map_err(Ft3168Error::I2cError)?;
		let mut buf = [0_u8;LEN];
		self.i2c.read(self.i2c_addr.clone(), &mut buf).map_err(Ft3168Error::I2cError)?;
		Ok(buf)
	}

	pub fn set_mode(&mut self, mode: WorkMode) -> Result<(), Ft3168Error<I2cT::Error>> {
		let payload = match mode{
			WorkMode::Working => 0b0000_0000,
			WorkMode::Testing => 0b0100_0000,
		};
		self.write_const_len::<2>(0x00, [payload].into_iter())
	}

	pub fn get_touches(&mut self,) -> nb::Result<Option<Touches>, Ft3168Error<I2cT::Error>>{
		// the poll rate for the driver is 100Hz. 
		if self.last_touch_read_instant.is_some_and(|inst| { 
			let now = (self.time_getter)();
			now - inst < MillisDurationU32::millis(10) 
		}) { return nb::Result::Err( nb::Error::WouldBlock )} 

		// The int pin is pulled down when there is a touch, and kept up otherwise
		if !self.int_pin.is_low().unwrap() { return Ok(None) ;}

		const READ_LEN: usize =  0x0E - 0x01; // last addr - first_addr
		let mut data = self.read_const_len::<READ_LEN>(0x01)?.into_iter();
		let gesture = match data.next().unwrap() { 
			0x10 => Ok(Some(Gesture::Move(MoveDir::Up))),
			0x14 => Ok(Some(Gesture::Move(MoveDir::Right))),
			0x18 => Ok(Some(Gesture::Move(MoveDir::Down))),
			0x1C => Ok(Some(Gesture::Move(MoveDir::Left))),
			0x48 => Ok(Some(Gesture::Zoom(ZoomDir::In))),
			0x49 => Ok(Some(Gesture::Zoom(ZoomDir::Out))),
			0x00 => Ok(None),
			_ => Err(Ft3168Error::InvalidDataReceived) 
		}?;

		let touch_count = data.next().unwrap() as usize;

		let mut touches = Vec::new();

		for _touch_ix in 0..(Ord::min(2, touch_count)) {
			let touch_xh = data.next().unwrap();
			let event = {
				let event_flag= (touch_xh & 0b1100_0000) >> 6;
				match event_flag{
					0b10	=> TouchEvent::Contact,
					0b00	=> TouchEvent::PressDown,
					0b01	=> TouchEvent::LiftUp,
					0b11	=> TouchEvent::NoEvent,
					_ 		=> unreachable!()
				}
			};
			let x_position_msb = (( touch_xh & 0b0000_1111 ) as u16) << 8;

			let touch_xl = data.next().unwrap();
			
			let x_position_lsb = touch_xl as u16;
			let x_position = x_position_msb | x_position_lsb;
			
			let touch_yh = data.next().unwrap();
			let id = (touch_yh & 0b1111_0000) >> 4;

			if id == 0x0F { continue } // The documentation describes this an invalid id. 
			
			let id = TouchId(u4::new(id));
			let y_position_msb = ((touch_yh & 0b0000_1111) as u16) << 8;				
			let touch_yl = data.next().unwrap();
			let y_position_lsb = touch_yl as u16;
			let y_position = y_position_msb | y_position_lsb;

			let pos  = TouchPos(Vector2::new(x_position, y_position));
			let weight = TouchWeight(data.next().unwrap());
			let area = TouchArea(u4::new((data.next().unwrap() & 0b1111_0000) >> 4) );
			let touch = Touch{
				id,
				pos,
				area,
				weight,
				event
			};

			touches.push(touch).unwrap();
		}

		let res = Touches{
			gesture: gesture,
			touch_count: touch_count,
			touches
		};
		self.last_touch_read_instant = Some((self.time_getter)());
		Ok(Some(res))
	}
}

pub struct Touches{
	pub gesture		: Option<Gesture>,
	pub touch_count	: usize,
	pub touches		: Vec<Touch, 2>
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Ft3168Error<I2cErr>{
	InvalidDataReceived,
	I2cError(I2cErr)
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum MoveDir{
	Up,
	Down,
	Right,
	Left,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum ZoomDir{
	In,
	Out
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Gesture{
	Move(MoveDir),
	Zoom(ZoomDir),
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TouchId(pub u4);

#[derive(Default, PartialOrd, Ord, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TouchArea(pub u4);

#[derive(Default, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TouchPos(pub Vector2<u16>);

#[derive(Default, PartialOrd, Ord, Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct TouchWeight(pub u8);

#[derive( Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum TouchEvent{
	PressDown,
	LiftUp,
	Contact,
	NoEvent
}

#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Touch {
	id: TouchId,
	pos		: TouchPos,
	area	: TouchArea,
	weight  : TouchWeight,
	event	: TouchEvent,
}