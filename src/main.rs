#![no_std]
#![no_main]

use core::{fmt::Write, time::Duration};
use defmt_rtt as _; // global logger
use cortex_m::{delay::Delay, interrupt, prelude::_embedded_hal_serial_Read};
use cortex_m_rt::entry;

use lora_e5_bsp::{
    hal::{gpio::{PortA, PortB, pins, Output, Analog}, pac::{self}, uart::{self, NoRx, Uart1,Uart2}, util::new_delay, i2c::I2c2, adc::{Adc, self, Ts},},
    led,
    pb::{PushButton, D0},
};

use lm75::{Lm75, Address};
use tsl256x::{Tsl2561, SlaveAddr};
use bme680::*   ;

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

#[entry]
fn main() -> ! {
    let dp: pac::Peripherals = pac::Peripherals::take().unwrap();
    let cp: pac::CorePeripherals = pac::CorePeripherals::take().unwrap();

    let mut rcc = dp.RCC;

    //Enable GPIO
    let gpiob: PortB = PortB::split(dp.GPIOB, &mut rcc);
    let gpioa: PortA = PortA::split(dp.GPIOA, &mut rcc);

    let usart1 = dp.USART1;
    let usart = dp.USART2;
    let i2c2 = dp.I2C2;
    let adc = dp.ADC;

    let a0 = gpioa.a0;
    let b3 = gpiob.b3;
    let b5 = gpiob.b5;
    let b6 = gpiob.b6;
    let b7 = gpiob.b7;
    let b15 = gpiob.b15; // SCL
    let a15 = gpioa.a15; // SDA
    let a2 = gpioa.a2;
    let a3 = gpioa.a3;
    let a9 = gpioa.a9;


    rcc.cr.modify(|_, w| w.hsion().set_bit());
    while rcc.cr.read().hsirdy().is_not_ready() {}

    let mut delay: Delay = new_delay(cp.SYST, &rcc);

    //Enable 3v3 for LM75
    let mut pwr_spply:Output<pins::A9> = cortex_m::interrupt::free(|cs| Output::default(a9, cs));
    pwr_spply.set_level_high();

    //Enable ADC
    let mut adc_in2 = Adc::new(adc, adc::Clk::RccHsi, &mut rcc);
    adc_in2.calibrate(&mut delay);
    adc_in2.set_sample_times(pins::B3::ADC_CH.mask() , Ts::MIN ,Ts::Cyc1);
    adc_in2.enable();
    let _analog_pin: Analog<pins::B3> = cortex_m::interrupt::free(|cs| Analog::new(b3, cs));
    
    //Enable I2C

    
    let i2c = interrupt::free(|cs|{I2c2::new(i2c2, (b15,a15), 100000, &mut rcc, false, cs)});
    let bus = shared_bus::BusManagerSimple::new(i2c);

    //Enable Led
    let mut led = interrupt::free(|cs| led::D5::new(b5, cs));
    led.set_off();

    //Enable Button
    let _button = cortex_m::interrupt::free(|cs| D0::new(a0, cs));
    
    //Enable UART in transittion mode only
   // let mut uart: Uart1<NoRx, pins::B6> = interrupt::free(|cs|{ Uart1::new(usart1, 115_200, uart::Clk::Hsi16, &mut rcc).enable_tx(b6, cs)});
   let mut uart:Uart1<pins::B7, pins::B6> = interrupt::free(|cs|{Uart1::new(usart1, 115_200,uart::Clk::Hsi16,&mut rcc).enable_tx(b6, cs).enable_rx(b7, cs)});
    let mut usart2:Uart2<pins::A3, pins::A2> = interrupt::free(|cs|{Uart2::new(usart, 115_200,uart::Clk::Hsi16,&mut rcc).enable_tx(a2, cs).enable_rx(a3, cs)});
    
    //Enable LM75A
    let all_pins_floating = 0x48;
    let address = Address::from(all_pins_floating);
    let mut lm75a = Lm75::new(bus.acquire_i2c(), address);
/*
    //BME680 
    let mut _bme680 = Bme680::init(bus.acquire_i2c(), &mut delay, I2CAddress::Primary).unwrap();
    let settings = SettingsBuilder::new()
        .with_humidity_oversampling(OversamplingSetting::OS2x)
        .with_pressure_oversampling(OversamplingSetting::OS4x)
        .with_temperature_oversampling(OversamplingSetting::OS8x)
        .with_temperature_filter(IIRFilterSize::Size3)
        .with_gas_measurement(Duration::from_millis(1500), 320, 25)
        .with_run_gas(true)
        .build();
    
    _bme680.set_sensor_settings(&mut delay, settings).unwrap();
    let _profile_duration = _bme680.get_profile_dur(&settings.0).unwrap();
    Delay::delay_ms(&mut delay, _profile_duration.as_millis() as u32);


    //TSL2561
   // let tsl2561 = Tsl2561::new(&bus.acquire_i2c(), SlaveAddr::ADDR_0x29.addr()).unwrap();
   // tsl2561.power_on(&mut bus.acquire_i2c()).unwrap(); 
  

*/
   
  
    loop {
    led.set_on();
    defmt::info!("joining");

    usart2.write_str("AT+UART=BR,115200\r\n").unwrap();
  //  let mut msg = usart2.read().unwrap();
    delay.delay_ms(1000);
    //uart.write_fmt(format_args!("{}",msg)).unwrap();


    usart2.write_str("AT+MODE=LWOTAA\r\n").unwrap();
  //  msg = usart2.read().unwrap();
  delay.delay_ms(1000);
   // uart.write_fmt(format_args!("{}",msg)).unwrap();

    usart2.write_str("AT+DR=EU868\r\n").unwrap();
    delay.delay_ms(1000);

    usart2.write_str("AT+CH=NUM,0-2\r\n").unwrap();
    delay.delay_ms(1000);

    usart2.write_str("AT+CLASS=A\r\n").unwrap();
    delay.delay_ms(1000);

    usart2.write_str("AT+PORT=8\r\n").unwrap();
    delay.delay_ms(1000);

    usart2.write_str("AT+ID=DevEui,\"70B3D57ED005B040\"\r\n").unwrap();
    delay.delay_ms(1000);

    usart2.write_str("AT+ID=AppEui,\"3E46E423455675E4\"\r\n").unwrap();
    delay.delay_ms(1000);

    usart2.write_str("AT+KEY=APPKEY,\"BDF4CF3CFE40578737D0D4323E6D3982\"\r\n").unwrap();
    delay.delay_ms(1000);
    
    usart2.write_str("AT+JOIN\r\n").unwrap();
    led.set_off();
    delay.delay_ms(3000);


   
    let _temp_celsius = lm75a.read_temperature().unwrap();
    //uart.write_fmt(format_args!("Temp on board: {temp_celsius}\n\r ")).unwrap();

   


    }
}
