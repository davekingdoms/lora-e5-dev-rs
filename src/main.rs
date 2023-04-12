#![no_std]
#![no_main]
#![macro_use]
#![allow(dead_code)]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_lora::stm32wl::*;
use embassy_lora::LoraTimer;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{AnyPin, Level, Output, Pin, Speed};
use embassy_stm32::rng::Rng;
use embassy_stm32::subghz::*;
use embassy_stm32::{interrupt, pac};

use embassy_time::{Duration, Timer};
use lorawan::default_crypto::DefaultFactory as Crypto;

use lorawan_device::async_device::{region, Device, JoinMode};
use {defmt_rtt as _, panic_probe as _};

struct RadioSwitch<'a> {
    ctrl1: Output<'a, AnyPin>,
    ctrl2: Output<'a, AnyPin>,
    ctrl3: Output<'a, AnyPin>,
}

impl<'a> RadioSwitch<'a> {
    fn new(ctrl1: Output<'a, AnyPin>, ctrl2: Output<'a, AnyPin>, ctrl3: Output<'a, AnyPin>) -> Self {
        Self { ctrl1, ctrl2, ctrl3 }
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

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSI16;
    config.rcc.enable_lsi = true;
    let p = embassy_stm32::init(config);

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }

    let ctrl1 = Output::new(p.PC3.degrade(), Level::High, Speed::High);
    let ctrl2 = Output::new(p.PC4.degrade(), Level::High, Speed::High);
    let ctrl3 = Output::new(p.PC5.degrade(), Level::High, Speed::High);
    let rfs = RadioSwitch::new(ctrl1, ctrl2, ctrl3);

    let radio = SubGhz::new(p.SUBGHZSPI, NoDma, NoDma);
    let irq = interrupt::take!(SUBGHZ_RADIO);

    let mut radio_config = SubGhzRadioConfig::default();
    radio_config.calibrate_image = CalibrateImage::ISM_863_870;
    let radio = SubGhzRadio::new(radio, rfs, irq, radio_config).unwrap();

  //  let mut region: region::Configuration = region::EU868::default().into();
    let  conf = region::Region::EU868;
    let mut region = region::Configuration::new(conf);
    // NOTE: This is specific for TTN, as they have a special RX1 delay
    region.set_receive_delay1(5000);

    let mut device: Device<_, Crypto, _, _> = Device::new(region, radio, LoraTimer::new(), Rng::new(p.RNG));

    // Depending on network, this might be part of JOIN
    device.set_datarate(region::DR::_0); // SF12

    // device.set_datarate(region::DR::_1); // SF11
    // device.set_datarate(region::DR::_2); // SF10
    // device.set_datarate(region::DR::_3); // SF9
    // device.set_datarate(region::DR::_4); // SF8
    // device.set_datarate(region::DR::_5); // SF7

    defmt::info!("Joining LoRaWAN network");
    let mut deveui = [0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xB3, 0x44];
   let mut appeui =  [0x6E, 0xE7, 0x8E, 0xF9, 0x0E, 0x9D, 0xD3, 0x44];
   let appkey = [0x05, 0x07, 0xEA, 0xCE, 0x6B, 0x5C, 0x10, 0xCA, 0x4D, 0x6F, 0x61, 0x61, 0x70, 0x64, 0x4B, 0x44];

   let mut sec:u64 = 5000;
   deveui.reverse();
   appeui.reverse();
   while let Err(err) = device.join(&JoinMode::OTAA {deveui, appeui, appkey,}).await{
        defmt::info!("{}",sec);
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
            Timer::after(Duration::from_millis(sec)).await;
            sec = sec*2;
    
}  
defmt::info!("Lorawan joined<");


loop {
   

//let mut rx: [u8; 255] = [0; 255];

/* 
defmt::info!("Sending>");
while let Err(error) =device.send_recv(&data,  &mut rx[..], 3, false).await{
    match error {
        lorawan_device::async_device::Error::Radio(_) => defmt::error!("Sending failed: Radio"),
        lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Sending failed: NetworkNotJoined")}
        lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Sending failed: UnableToPreparePayload")}
        lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Sending failed: InvalidDevAddr")}
        lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Sending failed: RxTimeout");break;}
        lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Sending failed: SessionExpired")}
        lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Sending failed: InvalidMic")}
        lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Sending failed: UnableToDecodePayload")}
         }
        Timer::after(Duration::from_millis(3000)).await;
    }

    Timer::after(Duration::from_millis(20000)).await;
    defmt::info!("Data sent>");
    defmt::info!("{}",temp_celsius);
    defmt::info!("rx: {}",rx);
    temp_celsius +=1;
  */
  defmt::info!("Sending>");
  while let Err(error) =device.send(b"$GPGLL , 3723.2475, N,12158.3416, W,161229.487, A*2C.", 2, false).await{
    match error {
        lorawan_device::async_device::Error::Radio(_) => defmt::error!("Sending failed: Radio"),
        lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Sending failed: NetworkNotJoined")}
        lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Sending failed: UnableToPreparePayload")}
        lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Sending failed: InvalidDevAddr")}
        lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Sending failed: RxTimeout");break;}
        lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Sending failed: SessionExpired")}
        lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Sending failed: InvalidMic")}
        lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Sending failed: UnableToDecodePayload")}
         }
        Timer::after(Duration::from_millis(3000)).await;
    }

    Timer::after(Duration::from_millis(5000)).await;
    defmt::info!("Data sent>");
    defmt::info!("$GPGLL,3723.2475, N,12158.3416, W,161229.487, A*2C");
 


    }
}

