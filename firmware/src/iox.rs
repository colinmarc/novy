//! Helpers for the three mcp23017s, which are connected to rotary switches.

use defmt::{debug, error};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Timer};
use esp_hal::{gpio::Input, i2c};

use crate::I2cBus;

const GPINTENA: u8 = 0x04;
// const GPINTENB: u8 = 0x05;
// const DEFVALA: u8 = 0x06;
// const DEFVALB: u8 = 0x07;
// const INTCONA: u8 = 0x08;
// const INTCONB: u8 = 0x09;
const IOCON: u8 = 0x0A;
const GPPUA: u8 = 0x0C;
// const GPPUB: u8 = 0x0D;
const GPIOA: u8 = 0x12;
// const GPIOB: u8 = 0x13;

struct Device(u8);
const SW1: Device = Device(0x20);
const SW2: Device = Device(0x21);
const SW3: Device = Device(0x22);

pub(crate) async fn init(i2c: &'static I2cBus) -> Result<(), i2c::master::Error> {
    let devices = [SW1, SW2, SW3];
    let mut guard = i2c.lock().await;

    for dev in devices {
        // Enable MIRROR and ODR for INTA
        // MIRROR=1, ODR=1.
        guard.write_async(dev.0, &[IOCON, 0b01000100]).await?;

        // Enable pullups.
        guard.write_async(dev.0, &[GPPUA, 0xFF, 0xFF]).await?;

        // Enable interrupts. We trigger on both make and break.
        guard
            .write_async(dev.0, &[GPINTENA, 0b00001111, 0b11111111])
            .await?;
    }

    Ok(())
}

/// Returns the level of each switch (0-11);
#[derive(Debug, Default, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub(crate) struct SwitchState {
    pub(crate) left: u8,
    pub(crate) middle: u8,
    pub(crate) right: u8,
}

impl SwitchState {
    pub(crate) const OFF: Self = SwitchState {
        left: 0,
        middle: 0,
        right: 0,
    };
}

pub(crate) static SWITCH_STATE: Watch<CriticalSectionRawMutex, SwitchState, 2> =
    Watch::new_with(SwitchState::OFF);

#[embassy_executor::task]
pub(crate) async fn monitor_switches(i2c: &'static super::I2cBus, mut inta: Input<'static>) {
    init(i2c).await.unwrap();

    // Read the initial switch positions.
    if let Err(e) = read_switches(i2c).await {
        error!("initial switch read failed: {:?}", e);
    }

    loop {
        // let _ = inta.wait_for_low().await;
        embassy_time::Timer::after_secs(1).await;

        if let Err(e) = read_switches(i2c).await {
            error!("failed to read switches: {:?}", e);
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

async fn read_switches(i2c: &'static super::I2cBus) -> Result<(), i2c::master::Error> {
    let left_raw = read_active(i2c, SW1).await?;
    let middle_raw = read_active(i2c, SW2).await?;
    let right_raw = read_active(i2c, SW3).await?;

    debug!(
        "interrupt fired sw1={:b} sw2={:b} sw3={:b}",
        left_raw, middle_raw, right_raw
    );

    let tx = SWITCH_STATE.sender();
    let prev = tx.try_get().unwrap_or(SwitchState::OFF);
    let state = SwitchState {
        left: decode_switch(left_raw).unwrap_or(prev.left),
        middle: decode_switch(middle_raw).unwrap_or(prev.middle),
        right: decode_switch(right_raw).unwrap_or(prev.right),
    };

    if state != prev {
        tx.send(state);
    }
    Ok(())
}

// Read a switch and return the active pins as a bitmask.
async fn read_active(i2c: &'static I2cBus, dev: Device) -> Result<u16, i2c::master::Error> {
    let mut guard = i2c.lock().await;

    let mut buf = [0u8; 2];
    guard.write_read_async(dev.0, &[GPIOA], &mut buf).await?;
    debug!("buf: {:?}", buf);

    // The order is:
    // 0->7: GPB0->7
    // 8->11: GPA0->3
    //
    // The inputs have a pullup by default, so we have to invert.
    let [a, b] = buf;
    let state = !(((a as u16 & 0x0F) << 8) | (b as u16)) & 0x0FFF;

    Ok(state)
}

fn decode_switch(raw: u16) -> Option<u8> {
    if raw.count_ones() != 1 {
        return None;
    }

    Some(raw.trailing_zeros() as u8)
}
