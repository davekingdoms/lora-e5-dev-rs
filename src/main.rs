//! This example runs on a STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LoRaWAN join functionality.
#![no_std]
#![no_main]
#![macro_use]
#![feature(type_alias_impl_trait, async_fn_in_trait)]
#![allow(incomplete_features)]
#![feature(generic_arg_infer)]



use core::str::from_utf8;
use core::sync::atomic::{ Ordering, AtomicU8};


use arrayvec::ArrayVec;
use defmt::*;
use embassy_executor::Spawner;
use embassy_lora::iv::{InterruptHandler,Stm32wlInterfaceVariant};
use embassy_lora::LoraTimer;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pin, Speed, Input, Pull, AnyPin};

use embassy_stm32::peripherals::{ PA0, PA1, SUBGHZSPI, DMA1_CH1, DMA1_CH2, RNG, PC6, USART2,USART1, DMA1_CH4, DMA1_CH3, DMA1_CH5, DMA1_CH6};
use embassy_stm32::rng::Rng;
use embassy_stm32::spi::Spi;
use embassy_stm32::usart::{Config, InterruptHandler as interrupt,Uart};
use embassy_stm32::{bind_interrupts, pac, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use nmea::Nmea;
use embassy_time::{Delay, Timer, Duration, with_timeout};
use lora_phy::mod_params::*;
use lora_phy::sx1261_2::SX1261_2;
use lora_phy::LoRa;
use lorawan::default_crypto::DefaultFactory as Crypto;

use lorawan_device::async_device::lora_radio::LoRaRadio;
use lorawan_device::async_device::{region, Device, JoinMode};
//use embassy_stm32::peripherals::RNG;
use {defmt_rtt as _, panic_probe as _};


struct Leds<'a> {
    leds: [Output<'a, AnyPin>; 3], 
    blue  : usize,
    green : usize,
    red   : usize
}

impl<'a> Leds<'a> {
    fn new(pins: [Output<'a, AnyPin>; 3])-> Self {
        Self {
            leds: pins,
            blue:  0,
            green: 1,
            red:   2
        }
    }
   
    async fn drflash(&mut self){

        if self.leds[self.green].is_set_low(){
            self.leds[self.green].set_high();
            Timer::after(Duration::from_millis(150)).await;
            self.leds[self.green].set_low();
            Timer::after(Duration::from_millis(150)).await;
        }
        else{
            self.leds[self.green].set_high();
            Timer::after(Duration::from_millis(150)).await;
            self.leds[self.green].set_low();
            Timer::after(Duration::from_millis(150)).await;
            self.leds[self.green].set_high();
        }
    }

    async fn idle(&mut self){
        self.leds[self.blue].set_high();
        self.leds[self.green].set_low();
    }
    async fn init(&mut self ){
        for _ in 0..3{
            self.leds[self.blue].set_high();
            self.leds[self.green].set_high();
            self.leds[self.red].set_high();
            Timer::after(Duration::from_millis(200)).await;
            self.leds[self.blue].set_low();
            self.leds[self.green].set_low();
            self.leds[self.red].set_low();
            Timer::after(Duration::from_millis(200)).await;
        }
    }

    async fn process_state(&mut self){
        let state_message = CHANNEL.recv().await;
        match state_message {
            AppState::Init     => {self.init().await}
            AppState::Joining  => {
                loop {
                    self.leds[self.blue].set_high();
                    if let Ok(state_message) = with_timeout(Duration::from_millis(250), CHANNEL.recv()).await {
                        CHANNEL.send(state_message).await;
                        break;
                    } else{
                        self.leds[self.blue].set_low();
                        if let Ok(state_message) = with_timeout(Duration::from_millis(250), CHANNEL.recv()).await {
                            CHANNEL.send(state_message).await;
                            break;
                        }else {
                            continue;
                        }
                    }
                }
            }
            AppState::Idle     => {self.idle().await}
            AppState::Sanding  => {self.leds[self.green].set_high();self.leds[self.red].set_low()}
            AppState::DrButton => {self.drflash().await}
            AppState::Error    => {self.leds[self.red].set_high();},
        }
    }

}
#[derive(Format,PartialEq)]
enum AppState {
    Init,
    Joining,
    Idle,
    Sanding,
    DrButton,
    Error
}

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;  
});

bind_interrupts!(struct Irq{
    USART2 => interrupt<peripherals::USART2>;
    USART1 => interrupt<peripherals::USART1>;
    
});
const LORAWAN_REGION: region::Region = region::Region::EU868; // warning: set this appropriately for the region
static NR:AtomicU8 = AtomicU8::new(0);
static CHANNEL: Channel<ThreadModeRawMutex, AppState, 1> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSE32;
    config.rcc.enable_lsi = true;
    let p = embassy_stm32::init(config);

    unsafe { pac::RCC.ccipr().modify(|w| w.set_rngsel(0b01)) }
    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
 
 
    // Set CTRL1 and CTRL3 for high-power transmission, while CTRL2 acts as an RF switch between tx and rx
    let _ctrl1 = Output::new(p.PC4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PC5.degrade(), Level::High, Speed::High);
    let _ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);
    let iv = Stm32wlInterfaceVariant::new(Irqs, None, Some(ctrl2)).unwrap();

    let mut uartconfig = Config::default();
    uartconfig.baudrate = 9600;

    let  usart2 = Uart::new(p.USART2, p.PA3, p.PA2, Irq, p.DMA1_CH3, p.DMA1_CH4, uartconfig);
    let mut usart1 = Uart::new(p.USART1,p.PB7, p.PB6, Irq, p.DMA1_CH5, p.DMA1_CH6, uartconfig);


    let button = ExtiInput::new(Input::new(p.PA0, Pull::Up), p.EXTI0);
    let button2 = ExtiInput::new(Input::new(p.PA1, Pull::Up), p.EXTI1);
    let button3 = ExtiInput::new(Input::new(p.PC6, Pull::Up), p.EXTI6);
    let leds = [
        Output::new(p.PB15.degrade(), Level::Low, Speed::Low),
        Output::new(p.PB9.degrade(), Level::Low, Speed::Low),
        Output::new(p.PB11.degrade(), Level::Low, Speed::Low),
    ];
    let leds = Leds::new(leds);
   
    spawner.spawn(led_manager(leds)).unwrap();
    CHANNEL.send(AppState::Init).await;

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
    Timer::after(Duration::from_millis(2000)).await;
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

  let mut _time = 15000;
   CHANNEL.send(AppState::Joining).await;
    // TODO: Adjust the EUI and Keys according to your network credentials
    while let Err(err) = device.join(&JoinMode::OTAA {deveui, appeui, appkey,}).await{

        match err {
                    lorawan_device::async_device::Error::Radio(_) => {defmt::error!("Join failed: Radio");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Join failed: NetworkNotJoined");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Join failed: UnableToPreparePayload");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Join failed: InvalidDevAddr");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Join failed: RxTimeout");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Join failed: SessionExpired");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Join failed: InvalidMic");CHANNEL.send(AppState::Error).await;}
                    lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Join failed: UnableToDecodePayload");CHANNEL.send(AppState::Error).await;}
            }  
            CHANNEL.send(AppState::Joining).await;
            Timer::after(Duration::from_millis(20000)).await;
            
         
    /*info!("Timer await: {}", time);
    Timer::after(Duration::from_millis(time)).await;
    if time < 3840000{
        time = time*2;
    }
    else {
        time = 15000;
    }*/
};  
usart1.write(b"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n").await.unwrap();
CHANNEL.send(AppState::Idle).await;
defmt::info!("Lorawan joined<");

info!("Spawn");
spawner.spawn(sending(button, usart1,usart2,  device)).unwrap();
spawner.spawn(button2_waiter(button2)).unwrap();
spawner.spawn(dr_up(button3)).unwrap();
info!("Spawned");


}

#[embassy_executor::task]
async fn sending(mut button: ExtiInput<'static, PA0>, mut uart: Uart<'static, USART1, DMA1_CH5, DMA1_CH6>,mut uart2: Uart<'static, USART2, DMA1_CH3, DMA1_CH4>,
 mut device:  Device<LoRaRadio<SX1261_2<Spi<'static, SUBGHZSPI, DMA1_CH1, DMA1_CH2>, Stm32wlInterfaceVariant< Output<'static, AnyPin>>>>, Crypto, LoraTimer, Rng<'static, RNG>>) {
    Timer::after(Duration::from_millis(3000)).await;
    info!("Ready for send");
    let mut nmea = Nmea::default();
    let mut buf = [0u8; 82];
    loop {

    button.wait_for_falling_edge().await;
    CHANNEL.send(AppState::Sanding).await;

loop{
    info!("...Reciving gps data...");
   match uart.read_until_idle(&mut buf).await{
        Ok(_) => {

                    uart2.write(&buf).await.unwrap();
                    info!("buf from uart {}",buf);
                    let sentence = from_utf8(&buf).unwrap();//truncate string. _until_idle not working
                    let gga:&str;
       
                    if sentence.starts_with("$GPGGA"){
                        gga = match  sentence.find('*') {
                            Some(index) =>&sentence[..index + 3],
                            None => &sentence,
                        };
                        info!("gga: {}", gga);
                         nmea.parse(gga).unwrap();
                        if nmea.fix_type.unwrap().is_valid(){
                            
                            break;
                         }
                         else{
                            info!("Fix not available");
                            Timer::after(Duration::from_millis(3000)).await;
                            continue;
                           
                        }
                    }else {
                        info!("Not GGA sentence: {}", sentence); 
                        Timer::after(Duration::from_millis(3000)).await;
                    }
            }
        
        Err(_err) => {
            //Ignore eg. framing errors
            }
        };
    }
       
                    let latitude =nmea.latitude().unwrap().to_le_bytes();
                    let longitude = nmea.longitude.unwrap().to_le_bytes();
                    let altitude = nmea.altitude().unwrap().to_le_bytes();
                

                    let mut combined: ArrayVec<u8, 20> = ArrayVec::new( );
                   if combined.try_extend_from_slice(&latitude).is_err(){
                    error!("Error extending combined with latitude");
                   }
                    if combined.try_extend_from_slice(&longitude).is_err(){
                        error!("Error extending combined with longitude");
                    }
                    if combined.try_extend_from_slice(&altitude).is_err(){
                        error!("Error extending combined with altitude")
                    }

                    let payload: [u8;20] = combined.into_inner().unwrap();

                    info!("Lat {}", f64::from_le_bytes(latitude));
                    info!("Long {}", f64::from_le_bytes(longitude));
                    info!("Alt {}", f32::from_le_bytes(altitude));

                    info!("Lat {}",latitude);
                    info!("Long {}", longitude);
                    info!("Alt {}",altitude);
 
                    info!("Payload {}", payload);
                    info!(">Sending");
                    Timer::after(Duration::from_millis(1000)).await;
                    let dr = NR.load(Ordering::Relaxed);
                    match dr {
                        0 => device.set_datarate(region::DR::_0),
                        1 => device.set_datarate(region::DR::_1),
                        2 => device.set_datarate(region::DR::_2),
                        3 => device.set_datarate(region::DR::_3),
                        4 => device.set_datarate(region::DR::_4),
                        5 => device.set_datarate(region::DR::_5),
                        6 => device.set_datarate(region::DR::_6),
                        _ => device.set_datarate(region::DR::_0)
                    }
                    while let Err(error) = device.send(&payload, 2, false).await{
                        match error {
                            lorawan_device::async_device::Error::Radio(_) => defmt::error!("Sending failed: Radio"),
                            lorawan_device::async_device::Error::NetworkNotJoined => {defmt::error!("Sending failed: NetworkNotJoined")}
                            lorawan_device::async_device::Error::UnableToPreparePayload(_) => {defmt::error!("Sending failed: UnableToPreparePayload")}
                            lorawan_device::async_device::Error::InvalidDevAddr => {defmt::error!("Sending failed: InvalidDevAddr")}
                            lorawan_device::async_device::Error::RxTimeout => {defmt::error!("Sending failed: RxTimeout");break}
                            lorawan_device::async_device::Error::SessionExpired => {defmt::error!("Sending failed: SessionExpired")}
                            lorawan_device::async_device::Error::InvalidMic => {defmt::error!("Sending failed: InvalidMic")}
                            lorawan_device::async_device::Error::UnableToDecodePayload(_) => {defmt::error!("Sending failed: UnableToDecodePayload")}
                             }
                            Timer::after(Duration::from_millis(3000)).await;
                        }
                        defmt::info!("Data sent>");
                        let  fcount = device.get_session().as_ref().unwrap().fcnt_up();
                        defmt::info!("Fcount: {}", fcount );
                        //defmt::info!("N,12158.3416 W,161229.487");
                        Timer::after(Duration::from_millis(20000)).await;
                        CHANNEL.send(AppState::Idle).await;
                        info!("...Ready for new send...")
     
    }
}


#[embassy_executor::task]
async fn button2_waiter(mut button2: ExtiInput<'static, PA1>) {
    loop {
 
    button2.wait_for_falling_edge().await;
 
    info!("BUTTON D2");
    let nr_old = NR.load(Ordering::Relaxed);
    CHANNEL.send(AppState::DrButton).await;
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
async fn dr_up(mut button3: ExtiInput<'static, PC6>) {
    loop {
        button3.wait_for_falling_edge().await;
        CHANNEL.send(AppState::DrButton).await;
        info!("BUTTON D3");
        let nr_old = NR.load(Ordering::Relaxed);
    
        if nr_old < 6{
                let nr_new = nr_old +1;
                NR.store(nr_new, Ordering::Relaxed);
                info!("NR: {}",nr_new);
             }
        else{
        info!("NR: 6");
            }
        }
    }

    #[embassy_executor::task]
    async fn led_manager(mut leds: Leds<'static>) {

        loop {
           leds.process_state().await;
            }
        }
    