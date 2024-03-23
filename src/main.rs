#![no_std]
#![cfg_attr(not(test), no_main)]
#![feature(abi_avr_interrupt)]
#![deny(unsafe_op_in_unsafe_fn)]
#![feature(lang_items)]

use core::panic::PanicInfo;

use arduino_hal::pac::PLL;
use arduino_hal::port::mode::Floating;
use arduino_hal::{
    delay_ms, entry, pins,
    port::{
        mode::{Input, Output},
        Pin,
    },
    Peripherals, Pins,
};
use atmega_usbd::{SuspendNotifier, UsbBus};
use avr_device::{asm::sleep, interrupt};
use usb_device::{
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_hid::{
    descriptor::{KeyboardReport, SerializedDescriptor},
    hid_class::HIDClass,
};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let pins: Pins = pins!(peripherals);
    let indicator = pins.led_tx.into_output();

    let pll = peripherals.PLL;
    let usb = peripherals.USB_DEVICE;

    // Configure pll
    // Set to 8MHz
    pll.pllcsr.write(|w| w.pindiv().set_bit());
    // Run 64MHz timers
    pll.pllfrq
        .write(|w| w.pdiv().mhz96().plltm().factor_15().pllusb().set_bit());
    // And enable
    pll.pllcsr.modify(|_, w| w.plle().set_bit());
    // Wait until the bit is set
    while pll.pllcsr.read().plock().bit_is_clear() {}

    let usb_bus = unsafe {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus<PLL>>> = None;
        &*USB_BUS.insert(UsbBus::with_suspend_notifier(usb, pll))
    };

    let hid_class = HIDClass::new(usb_bus, KeyboardReport::desc(), 60);
    let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x1209, 0x0001))
        .manufacturer("nz")
        .product("keyball")
        .build();

    let trigger = pins.a2.into_floating_input();

    unsafe {
        USB_CTX = Some(UsbContext {
            usb_device,
            hid_class,
            trigger: trigger.downgrade(),
            indicator: indicator.downgrade(),
        });

        interrupt::enable()
    }

    loop {
        sleep();
    }
}

static mut USB_CTX: Option<UsbContext<PLL>> = None;

#[interrupt(atmega32u4)]
fn USB_GEN() {
    unsafe { poll_usb() };
}

#[interrupt(atmega32u4)]
fn USB_COM() {
    unsafe { poll_usb() };
}

unsafe fn poll_usb() {
    let ctx = unsafe { USB_CTX.as_mut().unwrap() };
    ctx.poll();
}

struct UsbContext<S: SuspendNotifier> {
    usb_device: UsbDevice<'static, UsbBus<S>>,
    hid_class: HIDClass<'static, UsbBus<S>>,
    trigger: Pin<Input<Floating>>,
    indicator: Pin<Output>,
}

impl<S: SuspendNotifier> UsbContext<S> {
    fn poll(&mut self) {
        if self.trigger.is_low() {
            let report = ascii_to_report(b'a').unwrap();
            self.hid_class.push_input(&report).ok();
            self.indicator.set_high();
        } else {
            self.hid_class.push_input(&BLANK_REPORT).ok();
            self.indicator.set_low();
        }

        if self.usb_device.poll(&mut [&mut self.hid_class]) {
            let mut report_buf = [0u8; 1];

            if self.hid_class.pull_raw_output(&mut report_buf).is_ok() {
                if report_buf[0] & 2 != 0 {
                    self.indicator.set_high();
                } else {
                    self.indicator.set_low();
                }
            }
        }
    }
}

const BLANK_REPORT: KeyboardReport = KeyboardReport {
    modifier: 0,
    reserved: 0,
    leds: 0,
    keycodes: [0; 6],
};

fn ascii_to_report(c: u8) -> Option<KeyboardReport> {
    let (keycode, shift) = if c.is_ascii_alphabetic() {
        (c.to_ascii_lowercase() - b'a' + 0x04, c.is_ascii_uppercase())
    } else {
        match c {
            b' ' => (0x2c, false),
            _ => return None,
        }
    };

    let mut report = BLANK_REPORT;
    if shift {
        report.modifier |= 0x2;
    }
    report.keycodes[0] = keycode;
    Some(report)
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let peripherals = unsafe { Peripherals::steal() };
    let pins = pins!(peripherals);

    let ctx = unsafe { USB_CTX.as_mut().unwrap() };
    let mut rx = pins.led_rx.into_output();
    let mut tx = pins.led_tx.into_output();
    loop {
        for _ in 0..2 {
            rx.set_high();
            tx.set_high();
            delay_ms(300);
            rx.set_low();
            tx.set_low();
            delay_ms(300);
        }
        for _ in 0..2 {
            rx.set_high();
            tx.set_high();
            delay_ms(100);
            rx.set_low();
            tx.set_low();
            delay_ms(100);
        }
    }
}

#[lang = "eh_personality"]
#[no_mangle]
pub unsafe extern "C" fn rust_eh_personality() -> () {}
