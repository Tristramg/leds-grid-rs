#![deny(unsafe_code)]
#![no_main]
#![no_std]
mod animations;

use core::panic::PanicInfo;
use rtic::app;
use rtt_target::{rprintln, rtt_init_print};
use stm32f1xx_hal::gpio::{gpiob, Alternate, PushPull};
use stm32f1xx_hal::{pac, prelude::*, spi, timer};

use smart_leds::{SmartLedsWrite, RGB8};
use ws2812_spi::Ws2812;

const FREQ_HZ: u32 = 50;
type PINS = (spi::NoSck, spi::NoMiso, gpiob::PB15<Alternate<PushPull>>);
type SPI = spi::Spi<pac::SPI2, spi::Spi2NoRemap, PINS, u8>;

#[app(device = stm32f1xx_hal::stm32, peripherals = true)]
const APP: () = {
    // This are the resources shared accross all the “threads”
    struct Resources {
        timer: timer::CountDownTimer<pac::TIM3>,
        leds: [RGB8; 256],
        ws: Ws2812<SPI>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!(); // cargo embed
        rprintln!("Hello, world!");

        // Take ownership over the raw flash and rcc devices and convert them into the corresponding
        // HAL structs
        let mut flash = cx.device.FLASH.constrain();
        let mut rcc = cx.device.RCC.constrain();

        // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
        // `clocks`
        let clocks = rcc
            .cfgr
            .sysclk(64.mhz())
            .pclk1(32.mhz())
            .freeze(&mut flash.acr);

        // This timer is used to refresh the display
        let mut timer = timer::Timer::tim3(cx.device.TIM3, &clocks, &mut rcc.apb1)
            .start_count_down(FREQ_HZ.hz());
        timer.listen(timer::Event::Update);

        // Acquire the GPIOB peripheral: we’ll use it for the SPI output sent to ws2812 leds
        let mut gpiob = cx.device.GPIOB.split(&mut rcc.apb2);
        let pb15 = gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh);
        let spi = spi::Spi::spi2(
            cx.device.SPI2,
            (spi::NoSck, spi::NoMiso, pb15),
            ws2812_spi::MODE,
            3.mhz(),
            clocks,
            &mut rcc.apb1,
        );
        let ws = Ws2812::new(spi);

        init::LateResources {
            timer,
            leds: [RGB8::default(); 256],
            ws,
        }
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        rprintln!("Starting idle");
        loop {}
    }

    #[task(binds = TIM3, resources=[timer, leds, ws])]
    fn tim3(cx: tim3::Context) {
        static mut ELLAPSED: f32 = 0.0;
        cx.resources.timer.clear_update_interrupt_flag();
        *ELLAPSED += 1.0 / FREQ_HZ as f32;

        let leds = cx.resources.leds;

        for x in 0..16 {
            for y in 0..16 {
                let i = idx(x, y);
                leds[i] = animations::tixy(*ELLAPSED, i, x, y)
            }
        }
        cx.resources.ws.write(leds.iter().cloned()).unwrap();
    }
};

fn idx(x: usize, y: usize) -> usize {
    if y % 2 == 0 {
        y * 16 + x
    } else {
        (y + 1) * 16 - x - 1
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}
