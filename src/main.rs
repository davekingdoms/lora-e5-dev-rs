//! This example runs on a STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LoRaWAN join functionality.
#![no_std]
#![no_main]
#![macro_use]
#![feature(type_alias_impl_trait, async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(generic_arg_infer)]


use core::sync::atomic::{AtomicU32, Ordering};


use defmt::info;
use embassy_executor::Spawner;
use embassy_lora::iv::Stm32wlInterfaceVariant;
use embassy_lora::LoraTimer;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pin, Speed, Input, Pull, AnyPin};
use embassy_stm32::peripherals::{ PA0, PA1, PC6};
use embassy_stm32::rng::Rng;
use embassy_stm32::spi::Spi;
use embassy_stm32::{interrupt, into_ref, pac, Peripheral};
use embassy_time::{Delay, Timer, Duration};
use lora_phy::mod_params::*;
use lora_phy::sx1261_2::SX1261_2;
use lora_phy::LoRa;
use lorawan::default_crypto::DefaultFactory as Crypto;
use lorawan_device::async_device::lora_radio::LoRaRadio;
use lorawan_device::async_device::{region, Device, JoinMode};
use embassy_stm32::peripherals::RNG;
use {defmt_rtt as _, panic_probe as _};

const LORAWAN_REGION: region::Region = region::Region::EU868; // warning: set this appropriately for the region
static NR:AtomicU32 = AtomicU32::new(0);


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSE32;
    config.rcc.enable_lsi = true;
    let p = embassy_stm32::init(config);

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }
    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
 

    let irq = interrupt::take!(SUBGHZ_RADIO);
    into_ref!(irq);
    // Set CTRL1 and CTRL3 for high-power transmission, while CTRL2 acts as an RF switch between tx and rx
    let _ctrl1 = Output::new(p.PC4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5.degrade(), Level::High, Speed::High);
    let _ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);
    let iv = Stm32wlInterfaceVariant::new(irq, None, Some(ctrl2)).unwrap();

    let button = ExtiInput::new(Input::new(p.PA0, Pull::Up), p.EXTI0);
    let button2 = ExtiInput::new(Input::new(p.PA1, Pull::Up), p.EXTI1);
    let mut led1 = Output::new(p.PB15, Level::Low, Speed::High);
    let mut led2 = Output::new(p.PB9, Level::Low, Speed::High);
    let mut led3 = Output::new(p.PB11, Level::Low, Speed::High);

    let mut delay = Delay;

    let lora = {
        match LoRa::new(SX1261_2::new(BoardType::Stm32wlSx1262, spi, iv), true, &mut delay).await {
            Ok(l) => l,
            Err(err) => {
                info!("Radio error = {}", err);
                return;
            }
        }
    };
    let radio = LoRaRadio::new(lora);
    let mut region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
    region.set_receive_delay1(5000);

    let mut device: Device<_, Crypto, _, _> = Device::new(region, radio, LoraTimer::new(), Rng::new(p.RNG));

  
    device.set_datarate(region::DR::_0);
    defmt::info!("Joining LoRaWAN network");
    //eui-wl55jc-lora-phy
    //DEVEUI 70B3D57ED005D1A99
    //APPEUI 4D7E7E7DFE731129
    //APPKEY AFC2BF8B503D6104D599C31FDE99C249
    let mut deveui = [0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xD1, 0xA9];
    let mut appeui =  [0x4D, 0x7E, 0x7E, 0x7D, 0xFE, 0x73, 0x11, 0x29];
    let appkey = [0xAF, 0xC2, 0xBF, 0x8B, 0x50, 0x3D, 0x61, 0x04, 0xD5, 0x99, 0xC3, 0x1F, 0xDE, 0x99, 0xC2, 0x49];

   deveui.reverse();
   appeui.reverse();
  led1.set_high();
  led2.set_high();
  led3.set_high();
  let mut _time = 15000;
    // TODO: Adjust the EUI and Keys according to your network credentials
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
            Timer::after(Duration::from_millis(20000)).await;
    /*info!("Timer await: {}", time);
    Timer::after(Duration::from_millis(time)).await;
    if time < 3840000{
        time = time*2;
    }
    else {
        time = 15000;
    }*/
}  
;
 /*
match  device.join(&JoinMode::OTAA { deveui, appeui, appkey}).await{
    Ok(()) => defmt::info!("LoRaWAN network joined"),
    Err(err) => {
        defmt::error!("Radio error = {}", err);
        return; 
    }
};*/
defmt::info!("Lorawan joined<");
led2.set_low();
led3.set_low();
info!("Spawn");
spawner.spawn(button_waiter(button, spawner.clone())).unwrap();
spawner.spawn(button2_waiter(button2)).unwrap();
info!("Spawned");

loop {
    
defmt::info!("Sending>");
led2.set_high();
  while let Err(error) =device.send(b"N,12158.3416 W,161229.487", 2, false).await{
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
    led2.set_low();
    defmt::info!("Data sent>");
    let  fcount = device.get_session().as_ref().unwrap().fcnt_up();
    defmt::info!("Fcount: {}", fcount );
    defmt::info!("N,12158.3416 W,161229.487");
    Timer::after(Duration::from_millis(50000)).await;
}
}

#[embassy_executor::task]
async fn button_waiter(mut button: ExtiInput<'static, PA0>, spawner: Spawner) {
    loop {
 
    button.wait_for_falling_edge().await;
    spawner.spawn(blink()).unwrap();
    info!("BUTTON D1");
    let nr_old = NR.load(Ordering::Relaxed);

    if nr_old < 7{
            let nr_new = nr_old +1;
            NR.store(nr_new, Ordering::Relaxed);
            info!("NR: {}",nr_new);
         }
    else{
    info!("NR: 7");
        }
    }
}


#[embassy_executor::task]
async fn button2_waiter(mut button2: ExtiInput<'static, PA1>) {
    loop {
 
    button2.wait_for_falling_edge().await;
    info!("BUTTON D2");
    let nr_old = NR.load(Ordering::Relaxed);

    if nr_old >=1{
            let nr_new = nr_old -1;
            NR.store(nr_new, Ordering::Relaxed);
            info!("NR: {}",nr_new);
         }
    else{
        info!("NR: 0");
    }
         
    }
    
}

#[embassy_executor::task]
async fn blink(){
   let mut i = 0;
    while i < 2 {
        defmt::info!("{}",i);
        i += 1;
    }
}
