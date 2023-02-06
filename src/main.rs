#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod log;
//use defmt::*;


use embassy_executor::{Spawner, _export::StaticCell};
use embassy_stm32::{gpio::{Level, Output, Speed}, usart::{UartTx, Config}, dma::NoDma, i2c::{I2c}, interrupt, time::hz, peripherals::I2C2,};
use embassy_embedded_hal::shared_bus::blocking::i2c::I2cDevice;
use embassy_sync::blocking_mutex::{NoopMutex};
use embassy_time::{Duration, Timer, Delay};
use heapless::String;
use core::{fmt::Write, cell::RefCell};

use lm75::{Lm75, Address};
use tsl256x::{Tsl2561, SlaveAddr};
//use bme680::*;


// Dev profile: easier to debug panics when in debug
#[cfg(debug_assertions)]
use panic_semihosting as _;
//use {defmt_rtt as _, panic_probe as _};

// Release profile: minimize the binary size of the application
#[cfg(not(debug_assertions))]
#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {

    let mut config = embassy_stm32::Config::default();
    config.rcc.mux = embassy_stm32::rcc::ClockSrc::HSE32;
    let p = embassy_stm32::init(config);
    log::log!("Hello World!");
    let mut msg: String<64> = String::new();

    let mut _delay = Delay;

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    let mut usart = UartTx::new(p.USART1, p.PB6, NoDma, Config::default());

    let mut pwr_spply = Output::new(p.PA9, Level::Low, Speed::Low);
   
    static I2C_BUS: StaticCell<NoopMutex<RefCell<I2c<I2C2>>>> = StaticCell::new();
    let irq = interrupt::take!( I2C2_EV);
    let i2c = I2c::new(p.I2C2, p.PB15, p.PA15, irq, NoDma, NoDma, hz(10_000), Default::default());
    let i2c_bus = NoopMutex::new(RefCell::new(i2c));
    let i2c_bus = I2C_BUS.init(i2c_bus);

    //LM75A
    let i2c_dev = I2cDevice::new(i2c_bus);  
    let address = Address::from(0x48);
    let mut lm75a = Lm75::new(i2c_dev, address);

    //TSL2561
    let mut i2c_tsl = I2cDevice::new(i2c_bus); 
    let tsl2561 = Tsl2561::new(&i2c_tsl, SlaveAddr::ADDR_0x29.addr()).unwrap();
    tsl2561.power_on(&mut i2c_tsl).unwrap();
 
    //let  i2c_bme = I2cDevice::new(i2c_bus); 

    //let mut _bme680 = Bme680::init(i2c_bme, &mut delay, I2CAddress::Primary).unwrap();
    /*let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_run_gas(false)
        .build();

        bme680.set_sensor_settings(&mut delay, settings).unwrap();
        let profile_duration = bme680.get_profile_dur(&settings.0).unwrap();
        let b:u64 = profile_duration.as_millis()as u64;
        Timer::after(Duration::from_millis(b)).await;*/

    core::writeln!(&mut msg, "***--- Starting App ---***\r").unwrap();
    usart.blocking_write(msg.as_bytes()).unwrap();
    msg.clear();

    loop {
        core::writeln!(&mut msg, "-> App Running...\r").unwrap();
        usart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();
        pwr_spply.set_high();
        
        log::log!("high");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        log::log!("low");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;

        let temp_celsius = lm75a.read_temperature().unwrap();
        core::writeln!(&mut msg, "Temp on board = {temp_celsius:}°C \r").unwrap();
        usart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();
        pwr_spply.set_low();

        /*bme680.set_sensor_mode(&mut delay, PowerMode::ForcedMode).unwrap();
        let (data, _state) = bme680.get_sensor_data(&mut delay).unwrap();
        core::writeln!(&mut msg, "Temperature {}°C\r",data.temperature_celsius()).unwrap();
        usart.blocking_write(msg.as_bytes()).unwrap();
        msg.clear();*/
       
       //Timer::after(Duration::from_millis(400)).await;
    
       
       /*
       let visible_ir_raw_light = tsl2561.visible_and_ir_raw(&mut i2c_tsl).unwrap();
       let ir_only_raw = tsl2561.ir_raw(&mut i2c_tsl).unwrap();
       core::writeln!(&mut msg, "Temp on board = {visible_ir_raw_light:}°C \n\rIR (raw): {ir_only_raw}\r").unwrap();
       usart.blocking_write(msg.as_bytes()).unwrap();
       msg.clear();
       */

    }
}

