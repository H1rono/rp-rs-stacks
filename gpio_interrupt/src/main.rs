#![no_std]
#![no_main]

// Logging support
use defmt::info;
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
use bsp::hal::clocks::init_clocks_and_plls;

// interrupt macro
use bsp::hal::pac::interrupt;

// GPIO
use bsp::hal::gpio::bank0::Gpio26;
use bsp::hal::gpio::PullUpInput;
use bsp::hal::gpio::{bank0::Gpio25, Interrupt::EdgeLow, Pin, Pins, PushPullOutput};
use embedded_hal::digital::v2::ToggleableOutputPin;

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

struct Machine {
    led: Pin<Gpio25, PushPullOutput>,
    input: Pin<Gpio26, PullUpInput>,
}

static MACHINE: Lazy<Mutex<RefCell<Machine>>> = Lazy::new(|| {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let external_xtal_freq_hz = 12_000_000u32;
    let _clocks = init_clocks_and_plls(
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

    // The single-cycle I/O block controls our GPIO pins
    let sio = Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO 25 as an output to drive our LED.
    // we can use reconfigure() instead of into_pull_up_input()
    // since the variable we're pushing it into has that type
    let led = pins.gpio25.into_mode();

    // Set up the GPIO pin that will be our input
    let input = pins.gpio26.into_mode();

    // Trigger on the 'falling edge' of the input pin.
    // This will happen as the button is being pressed
    input.set_interrupt_enabled(EdgeLow, true);

    Mutex::new(RefCell::new(Machine { led, input }))
});

#[entry]
fn main() -> ! {
    // initialize certainly
    free(|cs| {
        let _ = MACHINE.borrow(cs);
    });
    // Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
    // will jump to the interrupt function when the interrupt occurs.
    // We do this last so that the interrupt can't go off while
    // it is in the middle of being configured
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    info!("setup done");

    loop {
        // interrupts handle everything else in this example.
        cortex_m::asm::wfi();
    }
}

#[interrupt]
fn IO_IRQ_BANK0() {
    info!("interrupt");
    free(|cs| {
        let mut m = MACHINE.borrow(cs).borrow_mut();
        // Check if the interrupt source is from the pushbutton going from high-to-low.
        // Note: this will always be true in this example, as that is the only enabled GPIO interrupt source
        if m.input.interrupt_status(EdgeLow) {
            // toggle can't fail, but the embedded-hal traits always allow for it
            // we can discard the return value by assigning it to an unnamed variable
            let _ = m.led.toggle();

            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            m.input.clear_interrupt(EdgeLow);
        }
    })
}
