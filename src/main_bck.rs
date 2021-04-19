//#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

// carge embed stuff
use core::panic::PanicInfo;
//use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;
use stm32f1xx_hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};

struct Leds<REMAP, PINS> {
    spi: Spi<pac::SPI2, REMAP, PINS, u8>,
    dma: stm32f1xx_hal::dma::dma1::C5,
    pub buffer: [u8; 3 * 3 * 4], // 3 bytes per color, 3 colours, 4 leds
}

fn get_pos(byte: usize, bit: usize) -> (usize, usize) {
    let pos = (byte * 8 + bit) * 3 + 1;
    (pos / 8, pos % 8)
}

impl<REMAP, PINS> Leds<REMAP, PINS>
{
    fn new(spi: Spi<pac::SPI2, REMAP, PINS, u8>, dma: stm32f1xx_hal::dma::dma1::C5) -> Self {
        /* On ws2812b led (with a 0,15μs tollerance):
        0 bit is sent as 0,40μs high, 0,85μs low
        1 bit is sent as 0,85μs high, 0,40μs low
        So, actually, we are sending 3 bits (1, 0, 0) for 0 or (1, 1 , 0) for 1
        Only the “middle bit” actually matters
        */
        let a = 0b100_100_10u8;
        let b = 0b0_100_100_1u8;
        let c = 0b00_100_100u8;
        Self {
            spi,
            dma,
            buffer: [a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c, a, b, c],
        }
    }

    fn set_bit(&mut self, byte: usize, bit: usize) {
        let (byte, bit) = get_pos(byte, bit);
        let mask = 1u8 << (7 - bit);
        self.buffer[byte] = self.buffer[byte] | mask;
    }

    fn unset_bit(&mut self, byte: usize, bit: usize) {
        let (byte, bit) = get_pos(byte, bit);
        let mask = 1u8 << (7 - bit);
        self.buffer[byte] = self.buffer[byte] & !mask;
    }

    fn set(&mut self, idx: usize, r: u8, g: u8, b: u8) {
        self.set_byte(idx, g);
        self.set_byte(idx + 1, r);
        self.set_byte(idx + 2, b);
    }

    fn set_byte(&mut self, byte: usize, data: u8) {
        // The strongest bit is written first:
        // 1 would be 0000_0001 in data, but we want to send 1000_0000
        // Hence we compare with the strongest bit (0x80 = 128 = 1000_0000)
        // And then shift everything to the left and look at the next bit
        for bit in 0..8 {
            rprintln!(
                "setting byte {:b} at index {} bit {} {:b}, shifted {:b}, &0x80: {:b}",
                data,
                byte,
                bit,
                0x80,
                data << bit,
                data << bit & 0x80
            );
            if (data << bit) & 0x80 == 0 {
                rprintln!(
                    "unsetting byte {} at index {} bit {} {:b}",
                    data,
                    byte,
                    bit,
                    0x80
                );
                self.unset_bit(byte, bit)
            } else {
                rprintln!("setting byte {} at index {} bit {}", data, byte, bit);
                self.set_bit(byte, bit)
            }
        }
    }

    fn reset(&mut self) {
        // A reset is 50μs on low bit, so 125 bits at 0,4μs/bit
        self.spi.write(&[0; 125]).ok().unwrap();
    }

    fn write(&mut self) {
        self.reset();
  //      let spi_dma = self.spi.with_tx_dma(self.dma);

   //     let clone = self.buffer.clone();
    //    let transfer = spi_dma.write(&clone);
     //   let (_, _) = transfer.wait();

    // Start a DMA transfer
    //let transfer = spi_dma.write(b"hello, world");

    // Wait for it to finnish. The transfer takes ownership over the SPI device
    // and the data being sent anb those things are returned by transfer.wait
    //let (_buffer, _spi_dma) = transfer.wait();

       //

       self.spi.write(&self.buffer).ok().unwrap()
      // self
    }
}

/*impl<T, REMAP, PINS, FRAMESIZE> Leds<T, REMAP, PINS, FRAMESIZE> {
    fn new<DMA, SPI>(dma: DMA, spi: Spi<SPI, REMAP, PINS, FRAMESIZE>) -> Self {
        Self {
            spi_dma: spi.with_tx_dma(dma),
        }
    }
}
*/
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
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, 1000.khz(), clocks, &mut rcc.apb1);

    // Set up the DMA device
    let dma = dp.DMA1.split(&mut rcc.ahb);

    let mut led = Leds::new(spi, dma.5);
    /*
        // Connect the SPI device to the DMA
        let spi_dma = spi.with_tx_dma(dma.5);

        // Start a DMA transfer
        let transfer = spi_dma.write(b"hello, world");

        // Wait for it to finnish. The transfer takes ownership over the SPI device
        // and the data being sent anb those things are returned by transfer.wait
        let (_buffer, spi_dma) = transfer.wait();

        let transfer = spi_dma.write(b"hello, world");
        //let (_buffer, _spi_dma) = transfer.wait();
    */
    led.set(0, 0, 0, 255);
    led.set(1, 0, 255, 0);
    led.set(2, 255, 0, 0);
    rprintln!(
        "Green: {:08b} {:08b} {:08b}",
        led.buffer[0],
        led.buffer[1],
        led.buffer[2]
    );
    rprintln!(
        "Red:   {:08b} {:08b} {:08b}",
        led.buffer[3],
        led.buffer[4],
        led.buffer[5]
    );
    rprintln!(
        "Blue:  {:08b} {:08b} {:08b}",
        led.buffer[6],
        led.buffer[7],
        led.buffer[8]
    );
    //led.write();
    loop {
       //led.write();
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
