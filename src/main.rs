#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use esp_backtrace as _;
use esp_hal::{clock::ClockControl, delay::Delay, gpio::IO, i2c::I2C, peripherals::{Peripherals, I2C0}, prelude::*, timer::TimerGroup, Async, Blocking};
use vl53l0x::VL53L0x;

extern crate alloc;
use core::mem::MaybeUninit;

#[global_allocator]
static ALLOCATOR: esp_alloc::EspHeap = esp_alloc::EspHeap::empty();

fn init_heap() {
    const HEAP_SIZE: usize = 32 * 1024;
    static mut HEAP: MaybeUninit<[u8; HEAP_SIZE]> = MaybeUninit::uninit();

    unsafe {
        ALLOCATOR.init(HEAP.as_mut_ptr() as *mut u8, HEAP_SIZE);
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let delay = Delay::new(&clocks);
    init_heap();

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    esp_println::logger::init_logger_from_env();

    let i2c = I2C::new(peripherals.I2C0, io.pins.gpio8, io.pins.gpio9, 1000_u32.kHz(), &clocks, None);
    let mut sensor: VL53L0x<I2C<I2C0,Blocking>>  = VL53L0x::new(i2c).unwrap();

    sensor.set_measurement_timing_budget(20000).unwrap();
    sensor.start_continuous(0).unwrap();

    loop {
        if let Some(distance) = sensor.read_range_mm().ok() {
            log::info!("Distance: {}",distance);
        } else {
            log::info!("oops");
        }
        delay.delay_millis(100);
    }
}
