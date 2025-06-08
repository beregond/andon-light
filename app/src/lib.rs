#![no_std]

use core::ops::DerefMut;
use embassy_sync::blocking_mutex::raw::RawMutex;
use embassy_sync::mutex::Mutex;
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::SdCard;
use esp_hal::delay::Delay;

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

pub struct SdReader<'a, M: RawMutex, BUS: embedded_hal::spi::SpiBus> {
    bus: &'a Mutex<M, BUS>,
    cs: esp_hal::gpio::Output<'static>,
    delay: Delay,
}

impl<'a, M: RawMutex, BUS: embedded_hal::spi::SpiBus> SdReader<'a, M, BUS> {
    pub fn new(bus: &'a Mutex<M, BUS>, cs: esp_hal::gpio::Output<'static>, delay: Delay) -> Self {
        Self { bus, cs, delay }
    }

    pub async fn read_config(self, filename: &str, buffer: &mut [u8]) -> Result<usize, ()> {
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
            return Ok(num_read);
        };
        Err(())
    }
}
