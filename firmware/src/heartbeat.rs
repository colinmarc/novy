use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use esp_hal::{
    gpio::{AnyPin, DriveMode},
    ledc::{
        LSGlobalClkSource, Ledc, LowSpeed,
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
    },
    peripherals::LEDC,
    time::Rate,
};

pub(crate) static ENABLED: Watch<CriticalSectionRawMutex, bool, 2> = Watch::new();

#[embassy_executor::task]
pub(crate) async fn run(ledc: LEDC<'static>, pin: AnyPin<'static>) {
    let mut ledc = Ledc::new(ledc);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);

    let mut hb_timer = ledc.timer::<LowSpeed>(timer::Number::Timer0);
    hb_timer
        .configure(timer::config::Config {
            duty: timer::config::Duty::Duty14Bit,
            clock_source: timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(26),
        })
        .unwrap();

    let mut hb_channel = ledc.channel(channel::Number::Channel0, pin);

    let mut rx = ENABLED.receiver().unwrap();
    let mut enabled = false;
    loop {
        let duty = if enabled { 25 } else { 0 };
        hb_channel
            .configure(channel::config::Config {
                timer: &hb_timer,
                duty_pct: duty,
                drive_mode: DriveMode::PushPull,
            })
            .unwrap();

        enabled = rx.changed().await;
    }
}
