//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]
// cargo embed stuff
use core::panic::PanicInfo;
//use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};


struct Leds<REMAP, PINS, const COUNT: usize>
{
    spi: Spi<pac::SPI2, REMAP, PINS, u8>,
    pub buffer: [u8; COUNT],
}

// Each bit of an RGB pixel is expanded into three bits (1, X, 0)
// Where X is the one we want to set or unset
// This functions returns
fn get_pos(byte: usize, bit: usize) -> (usize, usize) {
    let pos = (byte * 8 + bit) * 3 + 1;
    (pos / 8, pos % 8)
}

fn make_buffer<const COUNT: usize>() -> [u8; COUNT] {
    /* On ws2812b led (with a 0,15μs tollerance):
    0 bit is sent as 0,40μs high, 0,85μs low
    1 bit is sent as 0,85μs high, 0,40μs low
    So, actually, we are sending 3 bits (1, 0, 0) for 0 and (1, 1 , 0) for 1
    Only the “middle bit” actually matters
    We initialize everything every pixel at black (0, 0, 0) in RGB, hence (1, 0, 0) of every one of the 3 × 8 bits per pixel
    */

    let mut buffer = [0u8; COUNT];
    for i in 0..COUNT / 3 {
        buffer[i] = 0b100_100_10u8;
        buffer[i+1] = 0b0_100_100_1u8;
        buffer[i+1] = 0b00_100_100u8;
    }
    buffer
}

impl<REMAP, PINS, const COUNT: usize> Leds<REMAP, PINS, COUNT>
{
    fn new(spi: Spi<pac::SPI2, REMAP, PINS, u8>, buffer: [u8; COUNT]) -> Self {
        Self {
            spi,
            buffer,
        }
    }

    fn set(&mut self, idx: usize, r: u8, g: u8, b: u8) {
        self.set_byte(idx*3, g);
        self.set_byte(idx*3 + 1, r);
        self.set_byte(idx*3 + 2, b);
    }

    fn set_byte(&mut self, byte: usize, data: u8) {
        // The strongest bit is written first:
        // 1 would be 0000_0001 in data, but we want to send 1000_0000
        // Hence we compare with the strongest bit (0x80 = 128 = 1000_0000)
        // And then shift everything to the left and look at the next bit
        for bit in 0..8 {
            let (byte, bit) = get_pos(byte, bit);
            let mask = 1u8 << (7 - bit);
            if (data << bit) & 0x80 == 0 {
                self.buffer[byte] = self.buffer[byte] & !mask;
            } else {
                self.buffer[byte] = self.buffer[byte] | mask;
            }
        }
    }

    fn write(&mut self) {
        // A reset is 50μs on low bit, so 125 bits at 0,4μs/bit
        self.spi.write(&[0; 125]).ok().unwrap();
        self.spi.write(&self.buffer).ok().unwrap()
    }
}

#[entry]
fn main() -> ! {
    rtt_init_print!(); // cargo embed
    rprintln!("Hello, world!");

    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc
        .cfgr
        .sysclk(72.mhz())
        .pclk1(32.mhz())
        .freeze(&mut flash.acr);

    // Acquire the GPIOB peripheral
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let pins = (
        gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
        gpiob.pb14.into_floating_input(&mut gpiob.crh),
        gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
    );

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, 2800.khz(), clocks, &mut rcc.apb1);

    const NB_LED : usize = 10;
    let mut led = Leds::new(spi, make_buffer::<{9 * NB_LED}>());

    led.set(0, 0, 0, 255);
    led.set(1, 0, 255, 0);
    led.set(2, 255, 0, 0);

    loop {
       led.write();
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    rprintln!("{}", info);
    loop {} // You might need a compiler fence in here.
}

mod test {
    //use super::*;

    #[test]
    fn test_get_pos() {
        assert_eq!(get_pos(0, 0), (0, 1));
        assert_eq!(get_pos(0, 1), (0, 4));
        assert_eq!(get_pos(1, 0), (3, 1));
    }
}
