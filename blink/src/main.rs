//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

use core::cell::RefCell;

use core::ops::Deref;

use bsp::{entry, hal::gpio::bank0::Gpio25};
use cortex_m::interrupt::{free, Mutex};
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio::{Output, Pin, PushPull},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

use once_cell::sync::Lazy;

struct Machine {
    pub delay: Option<cortex_m::delay::Delay>,
    pub led_pin: Pin<Gpio25, Output<PushPull>>,
}

static MACHINE: Lazy<Mutex<RefCell<Machine>>> = Lazy::new(|| {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
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

    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());
    let delay = Some(delay);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // This is the correct pin on the Raspberry Pico board. On other boards, even if they have an
    // on-board LED, it might need to be changed.
    // Notably, on the Pico W, the LED is not connected to any of the RP2040 GPIOs but to the cyw43 module instead. If you have
    // a Pico W and want to toggle a LED with a simple GPIO output pin, you can connect an external
    // LED to one of the GPIO pins, and reference that pin here.
    let led_pin = pins.led.into_push_pull_output();
    Mutex::new(RefCell::new(Machine { delay, led_pin }))
});

#[entry]
fn main() -> ! {
    info!("Program start");
    let machine = MACHINE.deref();
    let mut delay = free(|cs| {
        let mut m = machine.borrow(cs).borrow_mut();
        m.delay.take().unwrap()
    });

    loop {
        info!("on!");
        free(|cs| {
            let mut m = machine.borrow(cs).borrow_mut();
            m.led_pin.set_high().unwrap();
        });
        delay.delay_ms(500);
        info!("off!");
        free(|cs| {
            let mut m = machine.borrow(cs).borrow_mut();
            m.led_pin.set_low().unwrap();
        });
        delay.delay_ms(500);
    }
}

// End of file
