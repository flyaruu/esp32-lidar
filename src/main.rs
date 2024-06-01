#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use alloc::boxed::Box;
use embassy_executor::{task, Spawner};
use embassy_sync::{blocking_mutex::raw::NoopRawMutex, pubsub::{PubSubChannel, Publisher, Subscriber}};
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{clock::ClockControl, delay::Delay, embassy::init, gpio::IO, i2c::I2C, peripherals::{Peripherals, I2C0}, prelude::*, timer::TimerGroup, Async, Blocking};
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

#[main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    // let delay = Delay::new(&clocks);
    init_heap();

    let io = IO::new(peripherals.GPIO,peripherals.IO_MUX);

    esp_println::logger::init_logger_from_env();

    let i2c = I2C::new_async(peripherals.I2C0, io.pins.gpio8, io.pins.gpio9, 1000_u32.kHz(), &clocks);

    init(&clocks, TimerGroup::new_async(peripherals.TIMG0, &clocks));

    let sensor: VL53L0x<I2C<I2C0,Async>>  = VL53L0x::new(i2c).unwrap();

    let sensor_channel: PubSubChannel<NoopRawMutex,u16,10,5,5> = PubSubChannel::new();
    let sensor_channel = Box::leak(Box::new(sensor_channel));

    spawner.spawn(measure_distance(sensor,sensor_channel.publisher().unwrap())).unwrap();
    spawner.spawn(log_distance(sensor_channel.subscriber().unwrap())).unwrap();
    loop {
        Timer::after_millis(1000).await;
    }
}

#[task]
async fn log_distance(mut subscriber: Subscriber<'static, NoopRawMutex,u16,10,5,5>) {
    loop {
        let distance = subscriber.next_message_pure().await;
        log::info!("Distance: {}",distance);
    }
}

#[task]
async fn measure_distance(mut sensor: VL53L0x<I2C<'static, I2C0,Async>>, publisher: Publisher<'static,NoopRawMutex,u16,10,5,5>) {
    sensor.set_measurement_timing_budget(20000).unwrap();
    sensor.start_continuous(0).unwrap();
    loop {
        if let Some(distance) = sensor.read_range_mm().ok() {
            publisher.publish(distance).await;
            // sensor.stop_continuous().unwrap();
            // sensor.start_continuous(100).unwrap()
        }
        // delay.delay(100.millis());
        Timer::after_millis(100).await;

    }
}