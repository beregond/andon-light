use andon_light_core::OutputSpiDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_1::digital::{ErrorType, OutputPin};
use embedded_hal_async::spi::SpiDevice as _;
use esp_hal::gpio::Output;
use esp_hal::{
    spi::{master::SpiDmaBus, FullDuplexMode},
    Async,
};

pub struct ControlPin<'d, T> {
    pub pin: Output<'d, T>,
}

impl<'d, T> OutputPin for ControlPin<'d, T>
where
    T: esp_hal::gpio::OutputPin,
{
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low();
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high();
        Ok(())
    }
}

impl<'d, T> ErrorType for ControlPin<'d, T> {
    type Error = core::convert::Infallible;
}

pub struct SpiDev<'d, T: esp_hal::gpio::OutputPin> {
    pub device: SpiDevice<
        'static,
        NoopRawMutex,
        SpiDmaBus<
            'static,
            esp_hal::peripherals::SPI2,
            esp_hal::dma::DmaChannel0,
            FullDuplexMode,
            Async,
        >,
        ControlPin<'d, T>,
    >,
}

impl<'d, T: esp_hal::gpio::OutputPin> OutputSpiDevice for SpiDev<'d, T> {
    type Error = core::convert::Infallible;

    async fn write(&mut self, buf: &[u8]) -> Result<(), Self::Error> {
        let _ = self.device.write(buf).await;
        Ok(())
    }
}
