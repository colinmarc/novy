//! Helpers for the three mcp23017s, which are connected to rotary switches.

use defmt::{debug, error, info};
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

// Proposed mapping of dial (0->11) to level (1 -> 14):
//  - 0 to off/standby
//  - 1 to 3
//  - 2 to 5
//  - 3 to 6
//  - 4 to 7
//  - 5 to 8
//  - 6 to 9
//  - 7 to 10
//  - 8 to 12
//  - 9 to 14
//  - 10 to boost
//  - 11 NC

struct Device(u8);
const SW1: Device = Device(0x20);
const SW2: Device = Device(0x21);
const SW3: Device = Device(0x22);

pub(crate) async fn init(i2c: &'static I2cBus) -> Result<(), i2c::master::Error> {
    let devices = [SW1]; // TODO
    let mut guard = i2c.lock().await;

    for dev in devices {
        // Enable MIRROR and ODR for INTA
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
    pub(crate) sw1: u8,
    pub(crate) sw2: u8,
    pub(crate) sw3: u8,
}

pub(crate) static SWITCH_STATE: Watch<CriticalSectionRawMutex, SwitchState, 2> = Watch::new();

#[embassy_executor::task]
pub(crate) async fn monitor_switches(i2c: &'static super::I2cBus, mut inta: Input<'static>) {
    init(i2c).await.unwrap();

    loop {
        if let Err(e) = monitor_switches_inner(i2c, &mut inta).await {
            error!("failed to read switches: {:?}", e);
            Timer::after(Duration::from_secs(1)).await;
        }
    }
}

async fn monitor_switches_inner(
    i2c: &'static super::I2cBus,
    inta: &mut Input<'static>,
) -> Result<(), i2c::master::Error> {
    let _ = inta.wait_for_low().await;
    let sw1_state = read_active(i2c, SW1).await?;
    // let sw2_state = read_active(i2c, DEV2.0).await.unwrap();
    // let sw3_state = read_active(i2c, DEV3.0).await.unwrap();

    debug!(
        "interrupt fired sw0={:b} sw1={:b} sw2={:b}",
        sw1_state, 0, 0
    );

    // Filter out states where 2 or 0 pins are low.
    if sw1_state.count_ones() == 1 {
        SWITCH_STATE.sender().send(SwitchState {
            sw1: sw1_state.trailing_zeros() as _,
            sw2: 0,
            sw3: 0,
        });
    }

    Ok(())
}

// Read a switch and return the active pins as a bitmask.
async fn read_active(i2c: &'static I2cBus, dev: Device) -> Result<u16, i2c::master::Error> {
    let mut guard = i2c.lock().await;

    let mut buf = [0u8; 2];
    guard.write_read_async(dev.0, &[GPIOA], &mut buf).await?;

    // The order is:
    // 0->7: GPB0->7
    // 8->11: GPA0->3
    //
    // The inputs have a pullup by default, so we have to invert.
    let [a, b] = buf;
    let state = !(((a as u16 & 0x0F) << 8) | (b as u16)) & 0x0FFF;

    Ok(state)
}
