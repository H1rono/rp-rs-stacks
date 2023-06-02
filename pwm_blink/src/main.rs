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
use bsp::hal::pwm::{Channel, FreeRunning, Pwm4, Slices};

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

// Struct used in the main loop
struct Machine {
    pub delay: Option<Delay>,
    pub channel: Channel<Pwm4, FreeRunning, bsp::hal::pwm::B>,
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
    // Init PWMs
    let pwm_slices = Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM4
    let mut pwm = pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    // Output channel B on PWM4 to the LED pin
    let mut channel = pwm.channel_b;
    channel.output_to(pins.led);
    Mutex::new(RefCell::new(Machine { delay, channel }))
});

impl Machine {
    pub fn set_duty(&mut self, duty: u16) {
        self.channel.set_duty(duty);
    }
}

// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;

// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 25000;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut delay = free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take().unwrap());

    // Infinite loop, fading LED up and down
    loop {
        info!("brightness up");
        // Ramp brightness up
        for i in (LOW..=HIGH).skip(100) {
            delay.delay_us(8);
            free(|cs| MACHINE.borrow(cs).borrow_mut().set_duty(i));
        }

        info!("brightness down");
        // Ramp brightness down
        for i in (LOW..=HIGH).rev().skip(100) {
            delay.delay_us(8);
            free(|cs| MACHINE.borrow(cs).borrow_mut().set_duty(i));
        }

        delay.delay_ms(500);
    }
}
