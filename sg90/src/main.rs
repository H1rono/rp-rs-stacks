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
use bsp::{entry, XOSC_CRYSTAL_FREQ};

// Peripherals
use bsp::hal::{pac, sio::Sio, watchdog::Watchdog};

// Clocks
use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use cortex_m::delay::Delay;

// GPIO
use bsp::hal::pwm::{Pwm0, Slices};

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

// SG90
use sg90::Sg90;

// Struct used in the main loop
struct Machine {
    pub delay: Option<Delay>,
    pub servo: Sg90<Pwm0, bsp::hal::pwm::B>,
}

// Global state
static MACHINE: Lazy<Mutex<RefCell<Machine>>> = Lazy::new(|| {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    let clocks = init_clocks_and_plls(
        XOSC_CRYSTAL_FREQ,
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

    let mut servo = Sg90::<Pwm0, bsp::hal::pwm::B>::from_slice(pwm_slices.pwm0);
    servo.output_to(pins.gpio1);
    Mutex::new(RefCell::new(Machine { delay, servo }))
});

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut delay = free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take()).unwrap();
    let mut deg = 0i32;
    let mut d_deg = 10;
    loop {
        info!("{}", deg);
        free(|cs| MACHINE.borrow(cs).borrow_mut().servo.set_degree(deg as u8));
        deg += d_deg;
        if deg <= 0 || 180 <= deg {
            d_deg *= -1;
        }
        delay.delay_ms(500);
    }
}
