//! This example runs on a STM32WL board, which has a builtin Semtech Sx1262 radio.
//! It demonstrates LoRaWAN join functionality.
#![no_std]
#![no_main]

#[path = "iv.rs"]
mod iv;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Pin, Speed};
use embassy_stm32::rng::{self, Rng};
use embassy_stm32::spi::Spi;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_time::{Delay, Duration, Timer};
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx126xVariant, TcxoCtrlVoltage, Sx126x};
use lora_phy::LoRa;
use lorawan_device::async_device::{region, Device, EmbassyTimer, JoinMode, JoinResponse};
use lorawan_device::default_crypto::DefaultFactory as Crypto;
use lorawan_device::{AppEui, AppKey, DevEui};
use {defmt_rtt as _, panic_probe as _};

use self::iv::{InterruptHandler, Stm32wlInterfaceVariant, SubghzSpiDevice};

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;

bind_interrupts!(struct Irqs{
    SUBGHZ_RADIO => InterruptHandler;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_stm32::Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(32_000_000),
            mode: HseMode::Bypass,
            prescaler: HsePrescaler::DIV1,
        });
        config.rcc.mux = ClockSrc::PLL1_R;
        config.rcc.pll = Some(Pll {
            source: PllSource::HSE,
            prediv: PllPreDiv::DIV2,
            mul: PllMul::MUL6,
            divp: None,
            divq: Some(PllQDiv::DIV2), // PLL1_Q clock (32 / 2 * 6 / 2), used for RNG
            divr: Some(PllRDiv::DIV2), // sysclk 48Mhz clock (32 / 2 * 6 / 2)
        });
    }
    let p = embassy_stm32::init(config);

    // Set CTRL1 and CTRL3 for high-power transmission, while CTRL2 acts as an RF switch between tx and rx
    let _ctrl1 = Output::new(p.PA4.degrade(), Level::Low, Speed::High);
    let ctrl2 = Output::new(p.PA5.degrade(), Level::High, Speed::High);
    //let _ctrl3 = Output::new(p.PC3.degrade(), Level::High, Speed::High);

    let spi = Spi::new_subghz(p.SUBGHZSPI, p.DMA1_CH1, p.DMA1_CH2);
    let spi = SubghzSpiDevice(spi);

    let config = sx126x::Config {
        chip: Sx126xVariant::Stm32wl,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: true,
        use_dio2_as_rfswitch: true,
    };
    let iv = Stm32wlInterfaceVariant::new(Irqs, None, Some(ctrl2)).unwrap();
    let lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay).await.unwrap();

    let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
    let mut device: Device<_, Crypto, _, _> = Device::new(region, radio, EmbassyTimer::new(), Rng::new(p.RNG, Irqs));

    defmt::info!("Joining LoRaWAN network");

    let mut deveui = [0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xD1, 0xA9];
    let mut appeui = [0x4D, 0x7E, 0x7E, 0x7D, 0xFE, 0x73, 0x11, 0x29];
    let appkey = [
        0xAF, 0xC2, 0xBF, 0x8B, 0x50, 0x3D, 0x61, 0x04, 0xD5, 0x99, 0xC3, 0x1F, 0xDE, 0x99, 0xC2,
        0x49,
    ];

    deveui.reverse();
    appeui.reverse();

    // TODO: Adjust the EUI and Keys according to your network credentials
    let join_mode = JoinMode::OTAA {
        deveui: DevEui::from(deveui),
        appeui: AppEui::from(appeui),
        appkey: AppKey::from(appkey),
    };

    info!("Joining LoRaWAN network");
    loop {
        let join_result = device.join(&join_mode).await;
        if let Ok(JoinResponse::JoinSuccess) = join_result {
            info!("LoRaWAN network joined");
            break;
        }
        Timer::after(Duration::from_millis(1000)).await;
        info!("Join failed: {:?}. Retrying...", join_result);
        Timer::after(Duration::from_millis(1000)).await;
    }
}