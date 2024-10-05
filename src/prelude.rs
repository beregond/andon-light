use embedded_hal_1::digital::{ErrorType, OutputPin};
use esp_hal::gpio::Output;

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
