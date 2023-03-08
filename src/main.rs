#![no_std]
#![no_main]
#![macro_use]
#![allow(dead_code)]
#![feature(type_alias_impl_trait)]
#![feature(async_fn_in_trait)]
#![allow(incomplete_features)]



use defmt_rtt as _;
use heapless::String;


use core::{cell::RefCell, fmt::Write};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_executor::{Spawner, _export::StaticCell};
use embassy_lora::stm32wl::{SubGhzRadio, SubGhzRadioConfig};
use embassy_stm32::{
    dma::NoDma,
    gpio::{AnyPin, Level, Output, Pin, Speed},
    i2c::I2c,
    interrupt::{self},
    peripherals::{I2C2, RNG},
    subghz::{CalibrateImage, SubGhz},
    time::hz,
    usart::{Config, UartTx, Uart},
};
use embassy_sync::blocking_mutex::NoopMutex;
use embassy_time::{Duration, Timer};
use lorawan_device::{async_device::{region, Device, JoinMode}};

use embassy_lora::LoraTimer;
use embassy_stm32::pac;
use embassy_stm32::rng::Rng;
use lorawan::{default_crypto::DefaultFactory as Crypto, parser::DevAddr, keys::AES128};


//use cortex_m::interrupt::Mutex;

use lm75::{Address, Lm75};

type Radio = SubGhzRadio<'static, RadioSwitch<'static>>;

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
pub struct RadioSwitch<'a> {
    ctrl1: Output<'a, AnyPin>,
    ctrl2: Output<'a, AnyPin>,
}

impl<'a> RadioSwitch<'a> {
    fn new(ctrl1: Output<'a, AnyPin>, ctrl2: Output<'a, AnyPin>) -> Self {
        Self { ctrl1, ctrl2 }
    }
}

impl<'a> embassy_lora::stm32wl::RadioSwitch for RadioSwitch<'a> {
    fn set_rx(&mut self) {
        self.ctrl1.set_high();
        self.ctrl2.set_low();
    }

    fn set_tx(&mut self) {
        self.ctrl1.set_low();
        self.ctrl2.set_high();
    }
}

//static LED: Mutex<RefCell<Option<Output<'static, PB5>>>> = Mutex::new(RefCell::new(None));

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSI16;
    config.rcc.enable_lsi = true;
    let p = embassy_stm32::init(config);
 

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    //let mut _usart = UartTx::new(p.USART1, p.PB6, NoDma, Config::default());
    let mut uartconfig = Config::default();
    uartconfig.baudrate = 9600;
    let irquart = interrupt::take!(USART1);
    let mut uart = Uart::new(p.USART1, p.PB7,  p.PB6, irquart, NoDma, NoDma, uartconfig);
let irqusart2 = interrupt::take!(USART2);
    let mut usart2 = Uart::new(p.USART2, p.PA3, p.PA2, irqusart2, NoDma, NoDma, uartconfig);
    let mut msg: String<64> = String::new();

    let mut pwr_spply = Output::new(p.PA9, Level::Low, Speed::Low);

    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<I2C2>>>> = StaticCell::new();
    let irq = interrupt::take!(I2C2_EV);
    let i2c = I2c::new(
        p.I2C2,
        p.PB15,
        p.PA15,
        irq,
        NoDma,
        NoDma,
        hz(10_000),
        Default::default(),
    );
    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    led.set_high();

    //LM75A
    let i2c_dev = I2cDevice::new(i2c_bus);
    let address = Address::from(0x48);
    let mut lm75a = Lm75::new(i2c_dev, address);

    let ctrl1 = Output::new(p.PA4.degrade(), Level::High, Speed::High);
    let ctrl2 = Output::new(p.PA5.degrade(), Level::High, Speed::High);

    let rfs = RadioSwitch::new(ctrl1, ctrl2);

    let radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);

    let int_sg = interrupt::take!(SUBGHZ_RADIO);

    let mut radio_config = SubGhzRadioConfig::default();
   
    radio_config.calibrate_image = CalibrateImage::ISM_863_870;

    let radio = SubGhzRadio::new(radio, rfs, int_sg, radio_config).unwrap();

    let mut region: region::Configuration = region::EU868::default().into();

    region.set_receive_delay1(5000);

    let mut device: Device<_, Crypto, _, _> =
        Device::new(region, radio, LoraTimer::new(), Rng::new(p.RNG));

    device.set_datarate(region::DR::_0);


    defmt::info!("Joining LoRaWAN network");
/*
    let  devadd:DevAddr<[u8;4]> = DevAddr::new([26,0x0B,12,0x6F]).unwrap();
    let appskey:AES128 =  AES128([0xFC,0x01,0x98,0xB5,0xF5,0x11,0x3C,0xA7,0x0E,0xA8,0xF2,0xFF,0xEC,0xB8,0x9F,0x2A]);
    let nwkskey:AES128 = AES128([0xE1,0x14,0xD9,0x24,0x52,0xC6,0xB9,0x27,0x62,0x95,0x2D,0xB3,0xF4,0xD1,0x10,0xA9]);

    while let  Err(err) =  device.join(&JoinMode::ABP { newskey: nwkskey, appskey: appskey, devaddr: devadd }).await{
        match err{
            lorawan_device::async_device::Error::Radio(_) => defmt::error!("Join failed: Radio"),
            lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Join failed: NetworkNotJoined")}
            lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Join failed: UnableToPreparePayload")}
            lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Join failed: InvalidDevAddr")}
            lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Join failed: RxTimeout")}
            lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Join failed: SessionExpired")}
            lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Join failed: InvalidMic")}
            lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Join failed: UnableToDecodePayload")}
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
*/
   // 70B3D57ED005B3B6
   // 6EE78EF90E9DD333
   // 0507EACE6B5C10CA4D6F616170644BF1
   let mut deveui = [0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xB3, 0xB6];
   let mut appeui =  [0x6E, 0xE7, 0x8E, 0xF9, 0x0E, 0x9D, 0xD3, 0x33];
   let appkey = [0x05, 0x07, 0xEA, 0xCE, 0x6B, 0x5C, 0x10, 0xCA, 0x4D, 0x6F, 0x61, 0x61, 0x70, 0x64, 0x4B, 0xF1];
   deveui.reverse();
   appeui.reverse();

   while let Err(err) = device.join(&JoinMode::OTAA {deveui, appeui, appkey,}).await{

   match err {
            lorawan_device::async_device::Error::Radio(_) => defmt::error!("Join failed: Radio"),
            lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Join failed: NetworkNotJoined")}
            lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Join failed: UnableToPreparePayload")}
            lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Join failed: InvalidDevAddr")}
            lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Join failed: RxTimeout")}
            lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Join failed: SessionExpired")}
            lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Join failed: InvalidMic")}
            lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Join failed: UnableToDecodePayload")}
     }
    Timer::after(Duration::from_millis(300000)).await;
}   

defmt::info!("Lorawan joined<");

defmt::info!( "***--- Starting App ---***");
/*
core::write!(&mut msg, "AT\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+MODE=LWOTAA\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+DR=EU868\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+CH=NUM,0-2\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+CLASS=A\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+PORT=8\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+ID=DevEui,\"70B3D57ED005B040\"\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+ID=AppEui,\"3E46E423455675E4\"\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+KEY=APPKEY,\"BDF4CF3CFE40578737D0D4323E6D3982\"\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;

core::write!(&mut msg, "AT+JOIN\r\n").unwrap();
usart2.blocking_write(msg.as_bytes()).unwrap();
msg.clear();
Timer::after(Duration::from_millis(1000)).await;*/


    loop {
     
        
      
        pwr_spply.set_high();

        led.set_high();
        defmt::info!("Led On");
        Timer::after(Duration::from_millis(2000)).await;

        led.set_low();
        defmt::info!("led off");
        Timer::after(Duration::from_millis(2000)).await;

        

        let temp_celsius = lm75a.read_temperature().unwrap();
       let data:[u8;1] = [temp_celsius as u8];


       /* 
        core::write!(&mut msg, "AT+MSGHEX=\"{}\"\r\n",temp_celsius).unwrap();
        usart2.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();
        defmt::info!("Data sent");*/

        //device.send(&data, 1, false);
        sending(&mut device, &data).await;
        
        defmt::info!("Temp on board = {}°C",temp_celsius);
        
        pwr_spply.set_low(); 

        Timer::after(Duration::from_millis(300000)).await;
       

    }
}


async fn sending( device: &mut lorawan_device::async_device::Device<SubGhzRadio<'_, RadioSwitch<'_>>, Crypto, LoraTimer, Rng<'_, RNG>>, data: &[u8]){
 
while let Err(error) = device.send(&data, 1, false).await{
    match error {
        lorawan_device::async_device::Error::Radio(_) => defmt::error!("Sending failed: Radio"),
        lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Sending failed: NetworkNotJoined")}
        lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Sending failed: UnableToPreparePayload")}
        lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Sending failed: InvalidDevAddr")}
        lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Sending failed: RxTimeout"); break}
        lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Sending failed: SessionExpired")}
        lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Sending failed: InvalidMic")}
        lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Sending failed: UnableToDecodePayload")}
         }
        
    }

}
 

