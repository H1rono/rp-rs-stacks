#![no_std]
#![no_main]

// Logging support
use defmt::*;
use defmt_rtt as _;

// Panic handler
use panic_probe as _;

// alias for the BSP
use rp_pico as bsp;

// Entry point
use bsp::entry;

// Peripherals
use bsp::hal::{pac, sio::Sio, watchdog::Watchdog};

// Clocks
use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use cortex_m::delay::Delay;

// GPIO
use bsp::hal::gpio::{bank0::Gpio25, Output, Pin, PushPull};
use embedded_hal::digital::v2::OutputPin;

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

// Struct used in the main loop
struct Machine {
    pub delay: Option<Delay>,
    pub led_pin: Pin<Gpio25, Output<PushPull>>,
}

// Global state
static MACHINE: Lazy<Mutex<RefCell<Machine>>> = Lazy::new(|| {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let delay = Some(Delay::new(core.SYST, clocks.system_clock.freq().to_Hz()));

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let led_pin = pins.led.into_push_pull_output();
    Mutex::new(RefCell::new(Machine { delay, led_pin }))
});

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut delay = free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        m.delay.take().unwrap()
    });

    loop {
        info!("on!");
        free(|cs| {
            let mut m = MACHINE.borrow(cs).borrow_mut();
            m.led_pin.set_high().unwrap();
        });
        delay.delay_ms(500);
        info!("off!");
        free(|cs| {
            let mut m = MACHINE.borrow(cs).borrow_mut();
            m.led_pin.set_low().unwrap();
        });
        delay.delay_ms(500);
    }
}
