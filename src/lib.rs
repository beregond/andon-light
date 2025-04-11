#![no_std]

use andon_light_core::OutputSpiDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embedded_hal_async::spi::SpiDevice as _;
use esp_hal::{
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    spi::{
        master::{Config, Spi, SpiDmaBus},
        Mode,
    },
    time::Rate,
    Async,
};

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
