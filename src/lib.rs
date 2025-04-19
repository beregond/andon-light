#![no_std]

use andon_light_core::OutputSpiDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::{NoopRawMutex, RawMutex};
use embassy_sync::mutex::Mutex;
use embedded_hal_async::spi::SpiDevice as _;
use embedded_sdmmc::SdCard;
use esp_hal::{delay::Delay, gpio::Output, spi::master::SpiDmaBus, Async};

pub struct SpiDev<'d> {
    pub device: SpiDevice<'static, NoopRawMutex, SpiDmaBus<'static, Async>, Output<'d>>,
}

impl<'d> OutputSpiDevice for SpiDev<'d> {
    type Error = core::convert::Infallible;

    async fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        let _ = self.device.write(buf).await;
        Ok(())
    }
}
