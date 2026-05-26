//! Drives the seven front-panel LEDs through a PCA9685 PWM controller
//! (U3) on the shared I2C bus.
//!
//! Layout
//!
//! | LED   | Meaning            | Channel(s)           |
//! |-------|--------------------|----------------------|
//! | PAN1  | pan, left burner   | 7                    |
//! | PAN2  | pan, middle burner | 2                    |
//! | PAN3  | pan, right burner  | 10                   |
//! | LINK1 | link mode          | 6                    |
//! | ST2   | status, left       | 5 (amber) / 4 (green)|
//! | ST1   | status, middle     | 1 (amber) / 0 (green)|
//! | ST3   | status, right      | 9 (amber) / 8 (green)|

use defmt::error;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Ticker};
use esp_hal::i2c;

use crate::I2cBus;

const ADDR: u8 = 0x40;

// PCA9685 registers.
const MODE1: u8 = 0x00;
const MODE2: u8 = 0x01;
const LED0_ON_L: u8 = 0x06; // LEDn block starts here, 4 bytes per channel.
const PRESCALE: u8 = 0xFE;

const MODE1_SLEEP: u8 = 0x10;
const MODE1_AI: u8 = 0x20; // Register auto-increment.
const MODE2_OUTDRV: u8 = 0x04; // Totem-pole outputs.

// ~1 kHz PWM. prescale = round(25MHz / (4096 * freq)) - 1.
const PRESCALE_1KHZ: u8 = 5;

pub(crate) static LED_STATE: Watch<CriticalSectionRawMutex, LedState, 2> = Watch::new();

const TICK: Duration = Duration::from_millis(50);

// Channel assignments, indexed by burner where applicable.
const PAN_CH: [u8; 3] = [7, 2, 10];
const LINK_CH: u8 = 6;

const STATUS_AK_CH: [u8; 3] = [5, 1, 9]; // amber
const STATUS_KA_CH: [u8; 3] = [4, 0, 8]; // green

// The blinking pattern for an LED.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum Pattern {
    Off,
    On,
    // On for one second, off for one second.
    Slow,
    // On for 333ms and off for 333ms.
    Fast,
}

impl Pattern {
    fn is_on_at(self, tick: u32) -> bool {
        match self {
            Pattern::Off => false,
            Pattern::On => true,
            Pattern::Slow => (tick / 10).is_multiple_of(2), // ~1 Hz.
            Pattern::Fast => (tick / 3).is_multiple_of(2),  // ~3.3 Hz.
        }
    }
}

/// Full-scale 12-bit duty for the PCA9685.
const FULL_DUTY: u16 = 4095;

/// Which mode the bidirectional status LED is in.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum Status {
    Green(Pattern),
    Amber(Pattern, u8),
}

/// The complete desired state of all seven LEDs. Recomputed by the
/// state machine whenever anything changes and pushed through
/// `LED_STATE`.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct LedState {
    /// Pan-detection LEDs: left, middle, right.
    pub(crate) pan: [Pattern; 3],
    /// Link-mode LED.
    pub(crate) link_mode: Pattern,
    /// Status LEDs (on / hot): left, middle, right.
    pub(crate) status: [Status; 3],
}

impl LedState {
    pub fn fault() -> Self {
        Self {
            pan: [Pattern::Off; 3],
            link_mode: Pattern::Off,
            status: [Status::Amber(Pattern::Slow, 255); 3],
        }
    }
}

impl Default for LedState {
    fn default() -> Self {
        Self {
            pan: [Pattern::Off; 3],
            link_mode: Pattern::Off,
            status: [Status::Green(Pattern::Off); 3],
        }
    }
}

#[embassy_executor::task]
pub(crate) async fn run(i2c: &'static I2cBus) {
    if let Err(e) = init(i2c).await {
        error!("PCA9685 init failed: {:?}", e);
        return;
    }

    // Wait for the oscillator to stabilize.
    embassy_time::Timer::after(Duration::from_micros(500)).await;

    let mut rx = LED_STATE.receiver().unwrap();
    let mut state = LedState::default();
    let mut ticker = Ticker::every(TICK);
    let mut tick: u32 = 0;

    loop {
        match select(ticker.next(), rx.changed()).await {
            Either::First(_) => tick = tick.wrapping_add(1),
            Either::Second(latest) => state = latest,
        }

        let mut buf = [0u8; 65];
        buf[0] = LED0_ON_L;

        for (i, pat) in state.pan.iter().enumerate() {
            set_led(
                &mut buf,
                PAN_CH[i],
                if pat.is_on_at(tick) { FULL_DUTY } else { 0 },
            );
        }

        for (i, s) in state.status.iter().enumerate() {
            let (ak, ka) = match s {
                Status::Green(pat) if pat.is_on_at(tick) => (0, FULL_DUTY),
                Status::Amber(pat, pct) if pat.is_on_at(tick) => {
                    ((*pct as u32 * FULL_DUTY as u32 / 255) as u16, 0)
                }
                _ => (0, 0),
            };

            set_led(&mut buf, STATUS_AK_CH[i], ak);
            set_led(&mut buf, STATUS_KA_CH[i], ka);
        }

        if state.link_mode.is_on_at(tick) {
            set_led(&mut buf, LINK_CH, FULL_DUTY);
        } else {
            set_led(&mut buf, LINK_CH, 0);
        }

        let mut guard = i2c.lock().await;
        if let Err(e) = guard.write_async(ADDR, &buf).await {
            error!("PCA9685 write failed: {:?}", e);
        }
    }
}

async fn init(i2c: &'static I2cBus) -> Result<(), i2c::master::Error> {
    let mut guard = i2c.lock().await;

    // The prescaler can only be set while in sleep mode.
    guard.write_async(ADDR, &[MODE1, MODE1_SLEEP]).await?;
    guard.write_async(ADDR, &[PRESCALE, PRESCALE_1KHZ]).await?;
    guard.write_async(ADDR, &[MODE2, MODE2_OUTDRV]).await?;
    guard.write_async(ADDR, &[MODE1, MODE1_AI]).await?;

    Ok(())
}

fn set_led(buf: &mut [u8; 65], ch: u8, duty: u16) {
    let base = 1 + ch as usize * 4;
    if duty == 0 {
        buf[base + 3] = 0x10; // Full off.
    } else if duty >= FULL_DUTY {
        buf[base + 1] = 0x10; // Full on.
    } else {
        // On at count 0, off at count `duty`.
        buf[base + 2] = (duty & 0xFF) as u8;
        buf[base + 3] = (duty >> 8) as u8;
    }
}
