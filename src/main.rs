//! This example runs on a STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LoRaWAN join functionality.
#![no_std]
#![no_main]
#![macro_use]
#![feature(type_alias_impl_trait, async_fn_in_trait)]
#![allow(incomplete_features)]

use defmt::info;
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_lora::iv::Stm32wlInterfaceVariant;
use embassy_lora::LoraTimer;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pin, Speed, Input, Pull};
use embassy_stm32::peripherals::{SUBGHZSPI, PA0};
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::rng::Rng;
use embassy_stm32::spi::{BitOrder, Config as SpiConfig, Spi, MODE_0};
use embassy_stm32::time::Hertz;
use embassy_stm32::{interrupt, into_ref, pac, Peripheral};
use embassy_time::{Delay, Duration, Timer};
use lora_phy::mod_params::*;
use lora_phy::sx1261_2::SX1261_2;
use lora_phy::LoRa;
use lorawan::default_crypto::DefaultFactory as Crypto;
use lorawan_device::async_device::lora_radio::LoRaRadio;
use lorawan_device::async_device::{region, Device, JoinMode};
use {defmt_rtt as _, panic_probe as _};

const LORAWAN_REGION: region::Region = region::Region::EU868; // warning: set this appropriately for the region
static DEVICE: Mutex<RefCell<Device<LoRaRadio<SX1261_2<>>>>>


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSI16;
    config.rcc.enable_lsi = true;
    let p = embassy_stm32::init(config);

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }

    let clk = Hertz(core::cmp::min(SUBGHZSPI::frequency().0 / 2, 16_000_000));
    let mut spi_config = SpiConfig::default();
    spi_config.mode = MODE_0;
    spi_config.bit_order = BitOrder::MsbFirst;
    let spi = Spi::new_subghz(p.SUBGHZSPI, NoDma, NoDma, clk, spi_config);

    let spi = BlockingAsync::new(spi);

    let irq = interrupt::take!(SUBGHZ_RADIO);
    into_ref!(irq);
    // Set CTRL1 and CTRL3 for high-power transmission, while CTRL2 acts as an RF switch between tx and rx
    let _ctrl1 = Output::new(p.PC4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5.degrade(), Level::High, Speed::High);
    let _ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);
    let iv = Stm32wlInterfaceVariant::new(irq, None, Some(ctrl2)).unwrap();

    let button = ExtiInput::new(Input::new(p.PA0, Pull::Up), p.EXTI0);

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
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
    let mut device: Device<_, Crypto, _, _> = Device::new(region, radio, LoraTimer::new(), Rng::new(p.RNG));

    
    defmt::info!("Joining LoRaWAN network");
    //eui-wl55jc-lora-phy
    let mut deveui = [0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xD1, 0xA9];
    let mut appeui =  [0x4D, 0x7E, 0x7E, 0x7D, 0xFE, 0x73, 0x11, 0x2A];
    let appkey = [0xAF, 0xC2, 0xBF, 0x8B, 0x50, 0x3D, 0x61, 0x04, 0xD5, 0x99, 0xC3, 0x1F, 0xDE, 0x99, 0xC2, 0x4E];

   deveui.reverse();
   appeui.reverse();
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
            
           
    
}  
defmt::info!("Lorawan joined<");
info!("Spawn");
spawner.spawn(button_waiter(button)).unwrap();
info!("Spawned");
}

#[embassy_executor::task]
async fn button_waiter(mut button: ExtiInput<'static, PA0>) {
    button.wait_for_falling_edge().await;
    info!("BUTTON D1")
}