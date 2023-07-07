#![no_std]

use defmt::info;
use embedded_hal::PwmPin;
use rp_pico as bsp;

use bsp::hal::gpio::{bank0::BankPinId, FunctionPwm, Pin, PinId, PinMode, ValidPinMode};
use bsp::hal::pwm::{self, Channel, ChannelId, FreeRunning, Slice, SliceId, ValidPwmOutputPin};

fn setup_slice<S>(slice: &mut Slice<S, FreeRunning>)
where
    S: SliceId,
{
    // https://www.i-programmer.info/programming/hardware/14849-the-pico-in-c-basic-pwm.html?start=2
    // TODO: check if this implementation is correct
    slice.set_ph_correct();
    slice.enable();
    slice.set_top(u16::MAX);
    // f_c = 125e6, f_pwm = 50, top = 65535
    // ceil(16 * f_c / (top * f_pwm)) = 0x263
    // so I set divider as 0x26.3
    slice.set_div_int(0x26);
    slice.set_div_frac(0x3);
}

fn degree_to_duty(deg: u8) -> u16 {
    // TODO: check this
    let deg = (deg % 181) as u16;
    // ** degree to pulse width **
    // 0deg <-> 0.5ms, 180deg <-> 2.4ms
    // => pulse_width_ms = degree * (2.4 - 0.5) / 180 + 0.5
    // ** pulse width to duty **
    // 20ms <-> 65535 (u16::MAX)
    // => duty = pulse_width_ms * u16::MAX / 20
    // ** degree to duty **
    // duty = (degree * (2.4 - 0.5) / 180 + 0.5) * u16::MAX / 20
    //      = degree * 1.9 * 65535 / 3600 + 65535 / 40
    // in python, these two are the same within `deg in range(181)`:
    //     int(deg * 1.9 * 65535 / 3600 + 65535 / 40)
    //     deg * int(1.9 * 65535 / 3600) + int(65535 / 40)
    // so I compute as:
    deg * 34 + 1638
}

// https://recruit.cct-inc.co.jp/tecblog/rust/template-rust/

pub struct Sg90A<S: SliceId> {
    channel: Channel<S, FreeRunning, pwm::A>,
}

impl<S: SliceId> Sg90A<S> {
    pub fn from_slice(mut slice: Slice<S, FreeRunning>) -> Self {
        setup_slice(&mut slice);
        Self {
            channel: slice.channel_a,
        }
    }

    pub fn output_to<G, M>(&mut self, pin: Pin<G, M>) -> Pin<G, FunctionPwm>
    where
        G: PinId + BankPinId + ValidPwmOutputPin<S, pwm::A>,
        M: PinMode + ValidPinMode<G>,
    {
        self.channel.output_to(pin)
    }

    pub fn set_duty(&mut self, duty: u16) {
        self.channel.set_duty(duty);
    }

    pub fn set_degree(&mut self, deg: u8) {
        let duty = degree_to_duty(deg);
        self.set_duty(duty);
    }

    pub fn set_radian(&mut self, rad: f32) {
        let deg = rad.to_degrees() as u8;
        self.set_degree(deg);
    }
}

pub struct Sg90B<S: SliceId> {
    channel: Channel<S, FreeRunning, pwm::B>,
}

impl<S: SliceId> Sg90B<S> {
    pub fn from_slice(mut slice: Slice<S, FreeRunning>) -> Self {
        setup_slice(&mut slice);
        Self {
            channel: slice.channel_b,
        }
    }

    pub fn output_to<G, M>(&mut self, pin: Pin<G, M>) -> Pin<G, FunctionPwm>
    where
        G: PinId + BankPinId + ValidPwmOutputPin<S, pwm::B>,
        M: PinMode + ValidPinMode<G>,
    {
        self.channel.output_to(pin)
    }

    pub fn set_duty(&mut self, duty: u16) {
        info!("duty is {}", duty);
        self.channel.set_duty(duty);
    }

    pub fn set_degree(&mut self, deg: u8) {
        let duty = degree_to_duty(deg);
        self.set_duty(duty);
    }

    pub fn set_radian(&mut self, rad: f32) {
        let deg = rad.to_degrees() as u8;
        self.set_degree(deg);
    }
}

pub trait ChannelExt<S: SliceId, C: ChannelId> {
    type Sg90;
}

impl<S: SliceId> ChannelExt<S, pwm::A> for Channel<S, FreeRunning, pwm::A> {
    type Sg90 = Sg90A<S>;
}

impl<S: SliceId> ChannelExt<S, pwm::B> for Channel<S, FreeRunning, pwm::B> {
    type Sg90 = Sg90B<S>;
}

pub type Sg90<S, C> = <Channel<S, FreeRunning, C> as ChannelExt<S, C>>::Sg90;
