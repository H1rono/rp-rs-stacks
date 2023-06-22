#![no_std]
#![no_main]

extern crate alloc;

use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use bsp::entry;
use bsp::hal;
use rp_pico as bsp;

// Peripherals
use bsp::hal::{pac, sio::Sio, watchdog::Watchdog};

// Clocks
use bsp::hal::clocks::{init_clocks_and_plls, Clock};
use cortex_m::delay::Delay;

// Concurrency
use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use once_cell::sync::Lazy;

// memory allocator
use embedded_alloc::Heap;

// GPIO
use bsp::hal::gpio::{bank0::Gpio25, Pin, PinId, PushPullOutput};
use embedded_hal::digital::v2::OutputPin;

// interrupt macro
use bsp::hal::pac::interrupt;

// USB Device support
use hal::usb::UsbBus;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

// To construct self-referential struct
use self_cell::self_cell;
type UsbOwner = UsbBusAllocator<UsbBus>;
struct UsbDependent<'a> {
    device: UsbDevice<'a, UsbBus>,
    serial: SerialPort<'a, UsbBus>,
}
self_cell!(
    struct Usb {
        owner: UsbOwner,
        #[covariant]
        dependent: UsbDependent,
    }
);

struct Machine<I: PinId> {
    delay: Option<Delay>,
    led: Pin<I, PushPullOutput>,
    usb: Usb,
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

static MACHINE: Lazy<Mutex<RefCell<Machine<Gpio25>>>> = Lazy::new(|| {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let clocks = init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
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
    let sio = Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let led_pin = pins.led.into_push_pull_output();

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    Mutex::new(RefCell::new(Machine {
        delay: Some(delay),
        led: led_pin,
        usb: Usb::new(usb_bus, |bus| UsbDependent {
            device: UsbDeviceBuilder::new(bus, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Fake company")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(2) // from: https://www.usb.org/defined-class-codes
                .build(),
            serial: SerialPort::new(bus),
        }),
    }))
});

#[entry]
fn main() -> ! {
    info!("Program start");

    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    // take `delay` from the machine
    let mut delay = free(|cs| MACHINE.borrow(cs).borrow_mut().delay.take()).unwrap();
    // Enable the USB interrupt
    unsafe {
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };

    // Blink the LED at 1 Hz
    loop {
        free(|cs| MACHINE.borrow(cs).borrow_mut().led.set_high().unwrap());
        delay.delay_ms(500);
        free(|cs| MACHINE.borrow(cs).borrow_mut().led.set_high().unwrap());
        delay.delay_ms(500);
    }
}

#[allow(non_snake_case)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    info!("interrupt");
    free(|cs| {
        MACHINE
            .borrow(cs)
            .borrow_mut()
            .usb
            .with_dependent_mut(|_, dep| usbctrl_irq(dep));
    });
}

fn usbctrl_irq(dep: &mut UsbDependent) {
    use core::sync::atomic::{AtomicBool, Ordering};

    /// Note whether we've already printed the "hello" message.
    static SAID_HELLO: AtomicBool = AtomicBool::new(false);

    // Say hello exactly once on start-up
    if !SAID_HELLO.load(Ordering::Relaxed) {
        SAID_HELLO.store(true, Ordering::Relaxed);
        let _ = dep.serial.write(b"Hello, World!\r\n");
    }

    // Poll the USB driver with all of our supported USB Classes
    if dep.device.poll(&mut [&mut dep.serial]) {
        let mut buf = [0u8; 64];
        match dep.serial.read(&mut buf) {
            Err(_e) => {
                // Do nothing
            }
            Ok(0) => {
                // Do nothing
            }
            Ok(count) => {
                // Convert to upper case
                buf.iter_mut().take(count).for_each(|b| {
                    b.make_ascii_uppercase();
                });

                // Send back to the host
                let mut wr_ptr = &buf[..count];
                while !wr_ptr.is_empty() {
                    let _ = dep.serial.write(wr_ptr).map(|len| {
                        wr_ptr = &wr_ptr[len..];
                    });
                }
            }
        }
    }
}
