#![no_std]
#![no_main]

// Logging support
use defmt::*;
use defmt_rtt as _;

use embedded_hal::PwmPin;
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
use bsp::hal::pwm::Slices;
use bsp::Pins;

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

// Struct used in the main loop
struct Machine {
    pub delay: Delay,
    pub pins: Pins,
    pub pwm_slices: Slices,
}

// Global state
static MACHINE: Lazy<Mutex<RefCell<Option<Machine>>>> = Lazy::new(|| {
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

    let delay = Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);
    Mutex::new(RefCell::new(Some(Machine {
        delay,
        pwm_slices,
        pins,
    })))
});

// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;

// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 25000;

#[entry]
fn main() -> ! {
    info!("Program start");
    let Machine {
        mut delay,
        mut pwm_slices,
        pins,
    } = free(|cs| MACHINE.borrow(cs).borrow_mut().take().unwrap());

    // Configure PWM4
    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    // Infinite loop, fading LED up and down
    loop {
        info!("brightness up");
        // Ramp brightness up
        for i in (LOW..=HIGH).skip(100) {
            delay.delay_us(8);
            channel.set_duty(i);
        }

        info!("brightness down");
        // Ramp brightness down
        for i in (LOW..=HIGH).rev().skip(100) {
            delay.delay_us(8);
            channel.set_duty(i);
        }

        delay.delay_ms(500);
    }
}
