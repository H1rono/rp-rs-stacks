#![no_std]
#![no_main]

use cortex_m::interrupt::{free, Mutex};
use defmt::*;
use defmt_rtt as _;
use once_cell::sync::Lazy;
use panic_probe as _;

use bsp::entry;
use rp_pico as bsp;

use bsp::hal::{self, clocks::init_clocks_and_plls, pac, watchdog::Watchdog, Timer};

// USB Device support
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDeviceBuilder, UsbVidPid},
};

// USB Communications Class Device support
use usbd_serial::SerialPort;

// Used to demonstrate writing formatted strings
use core::{cell::RefCell, fmt::Write};
use heapless::String;

struct Machine {
    pub usb_bus: UsbBusAllocator<hal::usb::UsbBus>,
    pub timer: Timer,
}

static MACHINE: Lazy<Mutex<RefCell<Option<Machine>>>> = Lazy::new(|| {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    Mutex::new(RefCell::new(Some(Machine { usb_bus, timer })))
});

#[entry]
fn main() -> ! {
    info!("Program start");
    let Machine { usb_bus, timer } = free(|cs| MACHINE.borrow(cs).take().unwrap());

    // Set up the USB Communications Class Device driver
    let mut serial = SerialPort::new(&usb_bus);

    // Create a USB device with a fake VID and PID
    let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
        .manufacturer("Fake company")
        .product("Serial port")
        .serial_number("TEST")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    let mut said_hello = false;
    loop {
        // A welcome message at the beginning
        if !said_hello && timer.get_counter().ticks() >= 2_000_000 {
            said_hello = true;
            let _ = serial.write(b"Hello, World!\r\n");

            let time = timer.get_counter().ticks();
            let mut text: String<64> = String::new();
            writeln!(&mut text, "Current timer ticks: {}", time).unwrap();

            // This only works reliably because the number of bytes written to
            // the serial port is smaller than the buffers available to the USB
            // peripheral. In general, the return value should be handled, so that
            // bytes not transferred yet don't get lost.
            if let Err(e) = serial.write(text.as_bytes()) {
                warn!("Error writing to serial port: {:?}", e);
            }
        }

        // Check for new data
        if usb_dev.poll(&mut [&mut serial]) {
            let mut buf = [0u8; 64];
            match serial.read(&mut buf) {
                Err(e) => {
                    warn!("Error reading from serial port: {:?}", e);
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
                        match serial.write(wr_ptr) {
                            Ok(len) => wr_ptr = &wr_ptr[len..],
                            // On error, just drop unwritten data.
                            // One possible error is Err(WouldBlock), meaning the USB
                            // write buffer is full.
                            Err(_) => break,
                        };
                    }
                }
            }
        }
    }
}

// End of file
