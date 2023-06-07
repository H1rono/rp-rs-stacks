#![no_std]
#![no_main]

// Logging support
use defmt::{info, warn};
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
use bsp::hal::timer::{Alarm, Alarm0, Timer};
use cortex_m::delay::Delay;
use fugit::{ExtU32, MicrosDurationU32};

// interrupt macro
use bsp::hal::pac::interrupt;

// GPIO
use bsp::hal::gpio::{bank0::Gpio25, Pin, PushPullOutput};
use embedded_hal::digital::v2::ToggleableOutputPin;

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

// Struct used in the main loop
struct Machine {
    pub delay: Option<Delay>,
    pub alarm_0: Alarm0,
    pub led_pin: Pin<Gpio25, PushPullOutput>,
}

impl Machine {
    fn reset_timer(&mut self) {
        self.alarm_0.enable_interrupt();
        if self.alarm_0.schedule(*TIMER_DURATION).is_err() {
            warn!("Error while initializing timer");
        }
    }
}

static TIMER_DURATION: Lazy<MicrosDurationU32> = Lazy::new(|| 1.secs());

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

    let mut timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    let mut alarm_0 = timer.alarm_0().unwrap();
    alarm_0.enable_interrupt();
    if alarm_0.schedule(*TIMER_DURATION).is_err() {
        warn!("Error while initializing timer");
    }

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let led_pin = pins.led.into_push_pull_output();
    let mut m = Machine {
        delay,
        led_pin,
        alarm_0,
    };
    m.reset_timer();
    Mutex::new(RefCell::new(m))
});

#[entry]
fn main() -> ! {
    let mut delay = free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take().unwrap());
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::TIMER_IRQ_0);
    }
    info!("setup done");

    loop {
        delay.delay_ms(1500);
        free(|cs| {
            info!("toggle!");
            MACHINE.borrow(cs).borrow_mut().led_pin.toggle().unwrap();
        })
    }
}

#[interrupt]
fn TIMER_IRQ_0() {
    free(|cs| {
        info!("interrupt!");
        let mut m = MACHINE.borrow(cs).borrow_mut();
        m.alarm_0.clear_interrupt();
        if let Err(e) = m.led_pin.toggle() {
            warn!("Error while interrupt: {:?}", e);
        }
        m.reset_timer();
    });
}
