#![no_std]
#![no_main]

mod log;

use core::{fmt::Write, time::Duration};



//use ::alloc::{format, fmt::format};
use cortex_m::{delay::Delay, interrupt};
use cortex_m_rt::entry;

use lora_e5_bsp::{
    hal::{gpio::{PortA, PortB, pins, Output, Input, }, pac::{self}, uart::{self, NoRx, Uart1}, util::new_delay, i2c::I2c2, adc::{Adc, self}},
    led,
    pb::{PushButton, D0},
};

use lm75::{Lm75, Address};
use tsl256x::{Tsl2561, SlaveAddr};
use bme680::*   ;

// Dev profile: easier to debug panics when in debug
#[cfg(debug_assertions)]
use panic_semihosting as _;

// Release profile: minimize the binary size of the application
#[cfg(not(debug_assertions))]
#[panic_handler]
fn panic_handler(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::nop();
    }
}

#[entry]
fn main() -> ! {
    let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

    let mut rcc = dp.RCC;

    //Enable GPIO
    let gpiob: PortB = PortB::split(dp.GPIOB, &mut rcc);
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut rcc);

    let usart1 = dp.USART1;
    let i2c2 = dp.I2C2;
    let adc = dp.ADC;

    let a0 = gpioa.a0;
    let b5 = gpiob.b5;
    let b6 = gpiob.b6;
   // let b3 = gpiob.b3;
    let b15 = gpiob.b15; // SCL
    let a15 = gpioa.a15; // SDA
    let a9 = gpioa.a9;
    //let a3 = gpioa.a3;


    rcc.cr.modify(|_, w| w.hsion().set_bit());
    while rcc.cr.read().hsirdy().is_not_ready() {}

    let mut delay: Delay = new_delay(cp.SYST, &rcc);

    //Enable ADC
    let mut adc_in2 = Adc::new(adc, adc::Clk::RccHsi, &mut rcc);
    adc_in2.calibrate(&mut delay);
    adc_in2.enable();
    adc_in2.set_max_sample_time();
    adc_in2.start_chsel(adc::Ch::In2.mask());
    while Adc::isr().ccrdy().is_not_complete() {}
    adc_in2.start_conversion();
    while Adc::isr().eoc().is_not_complete() {}


    
    //Enable 3v3 for LM75A
    let mut pwr_spply:Output<pins::A9> = cortex_m::interrupt::free(|cs| Output::default(a9, cs));
    pwr_spply.set_level_high();

    //Enable I2C
   // let soil:Input<pins::B3> = interrupt::free(|cs| Input::default(b3, cs));
    
    let i2c = interrupt::free(|cs|{I2c2::new(i2c2, (b15,a15), 100000, &mut rcc, false, cs)});
    let bus = shared_bus::BusManagerSimple::new(i2c);

    //Enable Led
    let mut led = interrupt::free(|cs| led::D5::new(b5, cs));
    led.set_off();

    //Enable Button
    let button = cortex_m::interrupt::free(|cs| D0::new(a0, cs));

   
    
    //Enable UART in transittion mode only
   
    let mut uart: Uart1<NoRx, pins::B6> = interrupt::free(|cs|{ Uart1::new(usart1, 115_200, uart::Clk::Hsi16, &mut rcc).enable_tx(b6, cs)});
    
    //Enable LM75A
    let all_pins_floating = 0x48;
    let address = Address::from(all_pins_floating);
    let mut lm75a = Lm75::new(bus.acquire_i2c(), address);

    //BME680
   
    let mut bme680 = Bme680::init(bus.acquire_i2c(), &mut delay, I2CAddress::Primary).unwrap();
    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_run_gas(true)
        .build();
    
    bme680.set_sensor_settings(&mut delay, settings).unwrap();
    let profile_duration = bme680.get_profile_dur(&settings.0).unwrap();

    bme680.set_sensor_mode(&mut delay, PowerMode::ForcedMode).unwrap();

    Delay::delay_ms(&mut delay, profile_duration.as_millis() as u32);
    let (data, _state) = bme680.get_sensor_data(&mut delay).unwrap();

    //TSL2561
 
    let tsl2561 = Tsl2561::new(&mut bus.acquire_i2c(), SlaveAddr::ADDR_0x29.addr()).unwrap();
   
    tsl2561.power_on(&mut bus.acquire_i2c()).unwrap(); 

  
    
    log::log!("Starting blinky"); 

    let mut n:u8 = 10;

    while n !=0{
        uart.write_str("***RL application starting***\n\r").unwrap();
        delay.delay_ms(100);
        led.set_on();
        log::log!("LED is on");

        delay.delay_ms(100);
        led.set_off();
        log::log!("LED is OFF");

        n -= 1;

    }

    led.set_on();
    log::log!("LED is on");

    loop {
       
        delay.delay_ms(1000);
        if button.is_pushed(){

            log::log!("Button is pushed");
            led.toggle();

        }

    uart.write_str(">App running..\n\r").unwrap();
   
    let temp_celsius = lm75a.read_temperature().unwrap();
    uart.write_fmt(format_args!("Temp on board: {}\n\r ",temp_celsius)).unwrap();

    uart.write_fmt(format_args!("Temperature {}°C\n\r",data.temperature_celsius())).unwrap(); 
    uart.write_fmt(format_args!("Pressure {}hPa\n\r",data.pressure_hpa())).unwrap();  
    uart.write_fmt(format_args!("Humidity {}%\n\r",data.humidity_percent())).unwrap();
    uart.write_fmt(format_args!("Gas Resistence {}Ω\n\r",data.gas_resistance_ohm())).unwrap();

    let visible_ir_raw_light = tsl2561.visible_and_ir_raw(&mut bus.acquire_i2c()).unwrap();
    let ir_only_raw = tsl2561.ir_raw(&mut bus.acquire_i2c()).unwrap();
    uart.write_fmt(format_args!("IR + visible (raw): {}\n\r",visible_ir_raw_light)).unwrap();
    uart.write_fmt(format_args!("IR (raw): {}\n\r",ir_only_raw)).unwrap();
    let soil_moisture=adc_in2.data();
    uart.write_fmt(format_args!("Soil Moisture (raw): {}\n\r",soil_moisture)).unwrap();

    

    
    }
}
