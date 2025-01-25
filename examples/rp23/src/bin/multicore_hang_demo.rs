//! This example demonstrates a potential bug that I believe is related to the
//! critical section implementation for the RP2350. When using the second core
//! of the processor, behavior is sometimes observed that looks a lot like
//! non-mutually exclusive access to data stored inside a mutex guarded by a
//! critical section. This can manifest itself in a couple of ways including
//! channels hanging on the sender side and DMA Transfer futures hanging. This
//! example uses async SPI to initiate DMA transfers as this seemed to be the
//! most consistent way that I found to cause a hang.
//!
//! When the bug is triggered, core 1 hangs and we detect this by the watchdog
//! timer initiating a chip reset. In this instance we hold the status LED high
//! and then panic to halt.
//!
//! If core 1 does not hang after 2 seconds, we manually trigger a watchdog
//! reset so that we can try again. We do this because I noticed that the bug
//! almost always gets triggered fairly early on in execution and if we survive
//! for more than 2 seconds it is unlikely that we will see it happen.
//!
//! If you couldn't already tell, this behavior is pretty cursed.
//!
//! The watchdog scratch registers are used to persist some metrics between
//! attempts. Registers 0 and 1 are used to hold the core 1 loop counter and
//! register 2 is used to track the number of attempts we've made so far (reboot
//! count).

#![no_std]
#![no_main]

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Executor;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::multicore::{spawn_core1, Stack};
use embassy_rp::peripherals::SPI0;
use embassy_rp::spi::{Async, Spi};
use embassy_rp::watchdog::Watchdog;
use embassy_rp::{pac, peripherals};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::spi::SpiDevice as _;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

static mut CORE1_STACK: Stack<4096> = Stack::new();
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    let mut watchdog_led = Output::new(p.PIN_25, Level::Low);
    let mut watchdog = Watchdog::new(p.WATCHDOG);

    // If the watchdog timer was triggered that means we caught the bug so stop
    // here and light up the status LED
    if pac::WATCHDOG.reason().read().timer() {
        watchdog_led.set_high();

        let loop_high = watchdog.get_scratch(1) as u64;
        let loop_low = watchdog.get_scratch(0) as u64;
        let loop_counter = (loop_high << 32) | loop_low;

        let reboot_counter = watchdog.get_scratch(2);

        defmt::panic!(
            "Watchdog triggered (loop_counter = {}, reboot_counter = {})!",
            loop_counter,
            reboot_counter
        );
    }

    // On the first boot, set the reboot counter to 0. On subsequent boots,
    // increment the counter by 1.
    if !pac::WATCHDOG.reason().read().force() {
        watchdog.set_scratch(2, 0);

        // Just a way to sanity check that the code is running when the debugger
        // is not attached
        for _ in 0..3 {
            watchdog_led.set_high();
            cortex_m::asm::delay(10_000_000);
            watchdog_led.set_low();
            cortex_m::asm::delay(10_000_000);
        }
    } else {
        let reboot_counter = watchdog.get_scratch(2);
        watchdog.set_scratch(2, reboot_counter + 1)
    }

    // Reset the scratch registers that track our loop counter
    watchdog.set_scratch(0, 0);
    watchdog.set_scratch(1, 0);
    watchdog.pause_on_debug(false);

    // If the watchdog does not get fed for more than 500ms, then we know that
    // core 1 is definitely stuck
    watchdog.start(Duration::from_micros(500_000));

    spawn_core1(
        p.CORE1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR1.init(Executor::new());
            executor1.run(|spawner| {
                let spi = Spi::new(p.SPI0, p.PIN_6, p.PIN_7, p.PIN_4, p.DMA_CH0, p.DMA_CH1, {
                    let mut cfg = embassy_rp::spi::Config::default();
                    cfg.frequency = 20_000_000;
                    cfg
                });
                let cs = Output::new(p.PIN_5, Level::High);

                spawner.must_spawn(core1_task(spi, cs, watchdog));
            });
        },
    );

    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(|spawner| {
        spawner.must_spawn(core0_task());
    });
}

/// After 2 seconds if we didn't get stuck then this task will force a watchdog
/// reset to try again
#[embassy_executor::task]
async fn core0_task() {
    defmt::info!("core0_task started");

    let start = Instant::now();

    loop {
        // If 2 seconds have passed since we started, the bug was not triggered
        // so reboot and try again
        if start.elapsed() > Duration::from_millis(2_000) {
            Watchdog::new(unsafe { peripherals::WATCHDOG::steal() }).trigger_reset();
        } else {
            // Since the embassy_rp time driver uses critical_section, this
            // simulates some contention on core 0. I found that 2µs worked best
            // and still did not starve core 1.
            Timer::after_micros(2).await;
        }
    }
}

pub type Spi0<'a> = Spi<'a, SPI0, Async>;
pub type Spi0Bus = Mutex<CriticalSectionRawMutex, Spi0<'static>>;
static SPI0_BUS: StaticCell<Spi0Bus> = StaticCell::new();

/// This task continuously tries to perform SPI transfers through an SpiDevice.
#[embassy_executor::task]
async fn core1_task(spi: Spi0<'static>, cs: Output<'static>, mut watchdog: Watchdog) {
    defmt::info!("core1_task started");

    // We use an SpiDevice instead of the raw SpiBus to add reliance on the
    // critical section for all operations
    let spi_bus = SPI0_BUS.init(Mutex::new(spi));
    let mut dev = SpiDevice::new(spi_bus, cs);

    let mut loop_counter = 0u64;

    loop {
        // This helps to vary the timing a bit and makes the hang more consistent
        for _ in 0..(loop_counter % 37) {
            // If mutual exclusion is not guaranteed, AtomicWaker might fail to
            // work as expected and the DMA Transfer future will not be woken up
            // causing core 1 to hang right here.
            dev.transfer_in_place(&mut [0u8; 16])
                .await
                .expect("Failed to complete SPI transfer");
        }

        // Keep track of the number of iterations of the loop
        loop_counter += 1;
        watchdog.set_scratch(0, loop_counter as u32);
        watchdog.set_scratch(1, (loop_counter >> 32) as u32);

        // If the SPI call hangs, the watchdog will not be fed and the chip will
        // reset confirming that we triggered the bug
        watchdog.feed();
    }
}
