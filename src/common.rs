use embassy_time::{Duration, Timer};
use esp_hal::{
    dma::*,
    peripherals::SPI2,
    spi::{master::SpiDmaBus, FullDuplexMode},
    Async,
};

enum DeviceState {
    Ok,    // I don't have to do anything, the system is running
    Idle,  // the system is idle, I may have to do something, but I don't have to do it now
    Warn,  // I should do something, but it's not urgent
    Error, // I have to do something now
}

enum SystemState {
    Ok,
    Warn,
    Error,
}

pub struct AndonLight {
    device_state: DeviceState,
    system_state: SystemState,
    leds_amount: u8,
}

impl AndonLight {
    pub fn new(leds_amount: u8) -> Self {
        Self {
            device_state: DeviceState::Ok,
            system_state: SystemState::Ok,
            leds_amount,
        }
    }

    pub async fn run_test_procedure<'a>(
        &mut self,
        spi: &mut SpiDmaBus<'a, SPI2, DmaChannel0, FullDuplexMode, Async>,
    ) {
        let green = Color { r: 0, g: 16, b: 0 };
        let red = Color { r: 16, g: 0, b: 0 };
        let blue = Color { r: 0, g: 0, b: 16 };
        let yellow = Color { r: 8, g: 8, b: 0 };
        let cyan = Color { r: 0, g: 8, b: 8 };
        let magenta = Color { r: 8, g: 0, b: 8 };
        let empty = Color { r: 0, g: 0, b: 0 };

        // First empty row also "primes" spi, it has more data, as it eats first two bytes ¯\_(ツ)_/¯
        let reset = [empty; 17];
        for item in reset.iter() {
            self.send_color(spi, item).await;
        }
        embedded_hal_async::spi::SpiBus::flush(spi).await.unwrap();

        let send = [
            [empty; 16],
            [red; 16],
            [green; 16],
            [blue; 16],
            [yellow; 16],
            [cyan; 16],
            [magenta; 16],
            [empty; 16],
        ];

        loop {
            for row in send.iter() {
                for item in row.iter() {
                    self.send_color(spi, item).await;
                }
                embedded_hal_async::spi::SpiBus::flush(spi).await.unwrap();
                Timer::after(Duration::from_millis(300)).await;
            }
        }
    }

    async fn send_color<'a>(
        &mut self,
        spi: &mut SpiDmaBus<'a, SPI2, DmaChannel0, FullDuplexMode, Async>,
        color: &Color,
    ) {
        let data = to_bytes(color.g);
        embedded_hal_async::spi::SpiBus::write(spi, &data)
            .await
            .unwrap();
        let data = to_bytes(color.r);
        embedded_hal_async::spi::SpiBus::write(spi, &data)
            .await
            .unwrap();
        let data = to_bytes(color.b);
        embedded_hal_async::spi::SpiBus::write(spi, &data)
            .await
            .unwrap();
    }
}

fn to_bytes(mut data: u8) -> [u8; 4] {
    // This function is copied from smart-leds crate.
    //
    // Send two bits in one spi byte. High time first, then the low time
    // The maximum for T0H is 500ns, the minimum for one bit 1063 ns.
    // These result in the upper and lower spi frequency limits
    let mut result = [0b0; 4];
    let patterns = [0b1000_1000, 0b1000_1110, 0b11101000, 0b11101110];
    for i in 0..4 {
        let bits = (data & 0b1100_0000) >> 6;
        result[i] = patterns[bits as usize];
        data <<= 2;
    }
    result
}

#[derive(Debug, Clone, Copy)]
pub struct Color {
    /// Red
    pub r: u8,
    /// Green
    pub g: u8,
    /// Blue
    pub b: u8,
}
