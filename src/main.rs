#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]


//use defmt::*;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};


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
    let p = embassy_stm32::init(Default::default());
    //info!("Hello World!");

    let mut led = Output::new(p.PB5, Level::High, Speed::Low);

    loop {
        //info!("high");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        //info!("low");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

