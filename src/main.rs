#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![macro_use]
#![allow(dead_code)]


use defmt_rtt as _; // global logger

use embassy_executor::{Spawner, _export::StaticCell};
use embassy_lora::stm32wl::{SubGhzRadioConfig, SubGhzRadio};
use embassy_stm32::{gpio::{Level, Output, Speed, AnyPin, Pin}, usart::{UartTx, Config}, dma::NoDma, i2c::I2c, interrupt::{self}, time::hz, peripherals::{I2C2, PB5}, subghz::{SubGhz, CalibrateImage}};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_sync::blocking_mutex::{NoopMutex};
use embassy_time::{Duration, Timer, Delay};
use heapless::String;
use lorawan_device::async_device::{region, Device, JoinMode};
use core::{fmt::Write, cell::{ RefCell}};

use lorawan::default_crypto::DefaultFactory as Crypto;
use embassy_stm32::rng::Rng;
use embassy_lora::LoraTimer;
use embassy_stm32::pac;
use embassy_lora::stm32wl::*;

use cortex_m::interrupt::Mutex;

use lm75::{Lm75, Address};




// Dev profile: easier to debug panics when in debug
#[cfg(debug_assertions)]
use panic_probe as _; // panic handler


// Release profile: minimize the binary size of the application
#[cfg(not(debug_assertions))]
#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

struct RadioSwitch<'a> {
    ctrl1: Output<'a, AnyPin>,
    ctrl2: Output<'a, AnyPin>,
    ctrl3: Output<'a, AnyPin>,
}

impl<'a> RadioSwitch<'a> {
    fn new(ctrl1: Output<'a, AnyPin>, ctrl2: Output<'a, AnyPin>,  ctrl3: Output<'a, AnyPin>) -> Self {
        Self { ctrl1, ctrl2,ctrl3 }
    }
}

impl<'a> embassy_lora::stm32wl::RadioSwitch for RadioSwitch<'a> {
    fn set_rx(&mut self) {
        self.ctrl1.set_high();
        self.ctrl2.set_low();
        self.ctrl3.set_high();
 
    }

    fn set_tx(&mut self) {
        self.ctrl1.set_high();
        self.ctrl2.set_high();
        self.ctrl3.set_high();
    }
}
//static LED: Mutex<RefCell<Option<Output<'static, PB5>>>> = Mutex::new(RefCell::new(None));


#[embassy_executor::main]
async fn main(_spawner: Spawner) {

    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSE32;
    config.rcc.enable_lsi = true;
    let p = embassy_stm32::init(config);
    //log::log!("Hello World!");
    let mut msg: String<64> = String::new();
    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }
    

    let mut _delay = Delay;

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    let mut usart = UartTx::new(p.USART1, p.PB6, NoDma, Config::default());

    let mut pwr_spply = Output::new(p.PA9, Level::Low, Speed::Low);

    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<I2C2>>>> = StaticCell::new();
    let irq = interrupt::take!( I2C2_EV);
    let i2c = I2c::new(p.I2C2, p.PB15, p.PA15, irq, NoDma, NoDma, hz(10_000), Default::default());
    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    led.set_high();

    //LM75A
    let i2c_dev = I2cDevice::new(i2c_bus);  
    let address = Address::from(0x48);
    let mut lm75a = Lm75::new(i2c_dev, address);

    let ctrl1 = Output::new(p.PA4.degrade(), Level::High, Speed::High);
    let ctrl2 = Output::new(p.PA5.degrade(), Level::High, Speed::High);
    let ctrl3 = Output::new(p.PA6.degrade(), Level::High, Speed::High);

    let rfs = RadioSwitch::new(ctrl1, ctrl2, ctrl3);
    let radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);
    let int_sg = interrupt::take!(SUBGHZ_RADIO);
    let mut radio_config = SubGhzRadioConfig::default();
    radio_config.calibrate_image = CalibrateImage::ISM_863_870;
    let radio = SubGhzRadio::new(radio, rfs, int_sg, radio_config).unwrap();
    let mut  region: region::Configuration = region::EU868::default().into();
    region.set_receive_delay1(5000);
    let mut device: Device<_, Crypto, _, _> = Device::new(region, radio, LoraTimer::new(), Rng::new(p.RNG));
    device.set_datarate(region::DR::_0);
    defmt::info!("Joining LoRaWAN network");

    device.join(&JoinMode::OTAA {

        deveui: [46,00,00,00,00,00,00,10],
        appeui: [46,00,00,00,00,00,00,10],
        appkey: [46,00,00,00,00,00,00,00,00,00,00,00,00,00,00,50],
    
    }).await.ok().unwrap();

  /*let mut rx: [u8; 255] = [0; 255];
  //log::log!("Sending 'PING'");
  let len = device.send_recv(b"PING", &mut rx[..], 1, true).await.ok().unwrap();
  if len > 0 {
    //log::log!("Message sent, received downlink: {:?}");
    defmt::info!("Message sent, received downlink: {:?}", &rx[..len]);
} else {
    defmt::info!("Message sent!");
}*/

    core::writeln!(&mut msg, "***--- Starting App ---***\r").unwrap();
    usart.blocking_write(msg.as_bytes()).unwrap();
    msg.clear();

    loop {
        core::writeln!(&mut msg, "-> App Running...\r").unwrap();
        usart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();
        pwr_spply.set_high();
        
        //log::log!("high");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        //log::log!("low");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        let temp_celsius = lm75a.read_temperature().unwrap();
        core::writeln!(&mut msg, "Temp on board = {temp_celsius:}°C \r").unwrap();
        usart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();
        pwr_spply.set_low();

    }
}


