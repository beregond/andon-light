use core::ops::DerefMut;

use andon_light_core::OutputSpiDevice;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_async::spi::SpiDevice as _;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::SdCard;
use esp_hal::gpio::Output;
use esp_hal::{
    delay::Delay,
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

pub struct TestTimeSource {
    fixed: embedded_sdmmc::Timestamp,
}

impl embedded_sdmmc::TimeSource for TestTimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        self.fixed
    }
}

pub fn make_time_source() -> TestTimeSource {
    TestTimeSource {
        fixed: embedded_sdmmc::Timestamp {
            year_since_1970: 33,
            zero_indexed_month: 3,
            zero_indexed_day: 3,
            hours: 13,
            minutes: 30,
            seconds: 5,
        },
    }
}

pub struct SdReader<'a, M: RawMutex, BUS: embedded_hal::spi::SpiBus, CS: OutputPin> {
    bus: &'a Mutex<M, BUS>,
    cs: CS,
    delay: Delay,
}

impl<'a, M: RawMutex, BUS: embedded_hal::spi::SpiBus, CS: OutputPin> SdReader<'a, M, BUS, CS> {
    pub fn new(bus: &'a Mutex<M, BUS>, cs: CS, delay: Delay) -> Self {
        Self { bus, cs, delay }
    }

    pub async fn read_config<'de, T>(
        self,
        filename: &str,
        buffer: &'de mut [u8],
    ) -> Result<Option<T>, ()>
    where
        T: Default + serde::Deserialize<'de>,
    {
        let mut guard = self.bus.lock().await;
        let mut bus = guard.deref_mut();
        let sd_spi_device = ExclusiveDevice::new(&mut bus, self.cs, self.delay).unwrap();
        let sd_card = SdCard::new(sd_spi_device, self.delay);

        let mut volume_mgr = embedded_sdmmc::VolumeManager::new(sd_card, make_time_source());
        if let Ok(mut volume0) = volume_mgr.open_volume(embedded_sdmmc::VolumeIdx(0)) {
            let mut root_dir = volume0.open_root_dir().unwrap();
            let mut my_file = root_dir
                .open_file_in_dir(filename, embedded_sdmmc::Mode::ReadOnly)
                .unwrap();
            let num_read = my_file.read(buffer).unwrap();
            match serde_json_core::de::from_slice::<T>(&buffer[0..num_read]) {
                Ok((result, _)) => {
                    return Ok(Some(result));
                }
                Err(e) => {
                    log::error!("{:?}", e);
                    return Err(());
                }
            }
        };
        Ok(None)
    }
}
