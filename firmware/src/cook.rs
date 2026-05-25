use core::sync::atomic::{AtomicU16, Ordering};

use defmt::{debug, error, info};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_can::{Frame, Id};
use esp_hal::{Async, time, twai};

const TX_TIMEOUT: Duration = Duration::from_millis(100);
const ACK_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Debug)]
pub(crate) enum Error {
    TxTimeout,
    AckTimeout(u16),
    Twai(twai::EspTwaiError),
}

impl From<twai::EspTwaiError> for Error {
    fn from(value: twai::EspTwaiError) -> Self {
        Self::Twai(value)
    }
}

impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::TxTimeout => defmt::write!(f, "TxTimeout"),
            Error::AckTimeout(id) => defmt::write!(f, "AckTimeout(waiting for {:03x})", id),
            Error::Twai(e) => defmt::write!(f, "Twai({})", e),
        }
    }
}

async fn wait_signal(signal: &Signal<CriticalSectionRawMutex, ()>, id: u16) -> Result<(), Error> {
    with_timeout(ACK_TIMEOUT, signal.wait())
        .await
        .map_err(|_| Error::AckTimeout(id))
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub(crate) enum Device {
    LeftCoil = 0x09,
    RightCoil = 0x13,
    Psu = 0x19,
}

pub(crate) fn all_zones() -> &'static [(Device, Zone)] {
    &[
        (Device::LeftCoil, Zone::Zone1),
        (Device::RightCoil, Zone::Zone1),
        (Device::RightCoil, Zone::Zone2),
    ]
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub(crate) enum Zone {
    Zone1 = 0,
    Zone2 = 1,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Power {
    Off,
    Standby,
    Level(u8),
    Boost,
}

impl Power {
    fn as_raw(self) -> u8 {
        match self {
            Power::Off => 0x00,
            Power::Standby => 0x61,
            Power::Level(n @ 1..=14) => n,
            Power::Level(_) => 0x0E,
            Power::Boost => 0xF1,
        }
    }
}

struct State {
    next_resp_id: AtomicU16,
    next_resp: Signal<CriticalSectionRawMutex, [u8; 8]>,

    // Ready signals (0x500 + device, empty frame).
    psu_ready: Signal<CriticalSectionRawMutex, ()>,
    left_ready: Signal<CriticalSectionRawMutex, ()>,
    right_ready: Signal<CriticalSectionRawMutex, ()>,

    // Init signals (0x580 + device, empty frame).
    left_init: Signal<CriticalSectionRawMutex, ()>,
    right_init: Signal<CriticalSectionRawMutex, ()>,
    psu_init: Signal<CriticalSectionRawMutex, ()>,

    // Announce acks (0x600 + device, [00 0B 00 01]).
    psu_ack: Signal<CriticalSectionRawMutex, ()>,
    left_ack: Signal<CriticalSectionRawMutex, ()>,
    right_ack: Signal<CriticalSectionRawMutex, ()>,
}

static STATE: State = State {
    next_resp_id: AtomicU16::new(0),
    next_resp: Signal::new(),
    psu_ready: Signal::new(),
    left_ready: Signal::new(),
    right_ready: Signal::new(),
    left_init: Signal::new(),
    right_init: Signal::new(),
    psu_init: Signal::new(),
    psu_ack: Signal::new(),
    left_ack: Signal::new(),
    right_ack: Signal::new(),
};

pub(crate) struct Cook<'a> {
    can_tx: twai::TwaiTx<'a, Async>,
}

impl<'a> Cook<'a> {
    pub(crate) fn new(can_tx: twai::TwaiTx<'a, Async>) -> Self {
        Self { can_tx }
    }

    pub(crate) async fn startup(&mut self) -> Result<(), Error> {
        info!("cooktop system init");

        STATE.psu_ready.reset();
        STATE.left_ready.reset();
        STATE.right_ready.reset();

        self.send(0x680, &[0x00, 0x01, 0x00, 0x06]).await?;
        Timer::after(Duration::from_millis(40)).await;
        self.send(0x581, &[]).await?;

        wait_signal(&STATE.left_ready, 0x509).await?;
        wait_signal(&STATE.right_ready, 0x513).await?;
        wait_signal(&STATE.psu_ready, 0x519).await?;
        info!("all devices ready");

        // Query serial + config from each device.
        for device in [Device::LeftCoil, Device::RightCoil, Device::Psu] {
            let serial = self.read_serial(device).await?;
            info!("{}: serial {:X}", device, serial);
            let config = self.read_config(device).await?;
            info!("{}: config {:X}", device, config);
        }

        // Controller announce.
        STATE.psu_ack.reset();
        STATE.left_ack.reset();
        STATE.right_ack.reset();
        self.send(0x681, &[0x00, 0x0B, 0x00, 0x00]).await?;
        wait_signal(&STATE.left_ack, 0x609).await?;
        wait_signal(&STATE.right_ack, 0x613).await?;
        wait_signal(&STATE.psu_ack, 0x619).await?;
        info!("all devices acked announce");

        // Match the OEM sequence: device_on once per coil, then
        // set all zones to standby.
        self.device_on(Device::LeftCoil).await?;
        self.set_power(Device::LeftCoil, Zone::Zone1, Power::Standby)
            .await?;

        self.device_on(Device::RightCoil).await?;
        self.set_power(Device::RightCoil, Zone::Zone1, Power::Standby)
            .await?;
        self.set_power(Device::RightCoil, Zone::Zone2, Power::Standby)
            .await?;
        self.set_power(Device::RightCoil, Zone::Zone1, Power::Standby)
            .await?;

        // Configure both coils.
        for device in [Device::LeftCoil, Device::RightCoil] {
            self.roundtrip(0x680 | device as u16, &[0x01, 0x06, 0x00, 0x0E, 0x74])
                .await?;
        }

        // Final announce.
        STATE.psu_ack.reset();
        STATE.left_ack.reset();
        STATE.right_ack.reset();
        self.send(0x681, &[0x00, 0x0B, 0x00, 0x00]).await?;
        wait_signal(&STATE.left_ack, 0x609).await?;
        wait_signal(&STATE.right_ack, 0x613).await?;
        wait_signal(&STATE.psu_ack, 0x619).await?;
        info!("startup complete");

        Ok(())
    }

    pub(crate) async fn shutdown(&mut self) -> Result<(), Error> {
        info!("cooktop shutdown");

        // Reset coils.
        self.roundtrip(0x680 | Device::LeftCoil as u16, &[0x01, 0x01, 0x00, 0x00])
            .await?;
        self.roundtrip(0x680 | Device::RightCoil as u16, &[0x01, 0x01, 0x00, 0x00])
            .await?;

        // PSU shutdown.
        self.roundtrip(0x680 | Device::Psu as u16, &[0x01, 0x03, 0x00, 0x02])
            .await?;
        self.roundtrip(0x680 | Device::Psu as u16, &[0x01, 0x02, 0x00, 0x00])
            .await?;

        // Final announce.
        self.send(0x681, &[0x00, 0x0B, 0x00, 0x00]).await?;
        info!("shutdown complete");
        Ok(())
    }

    /// Send init command to a coil (0x01 0x01 0x00 0x01).
    pub(crate) async fn device_on(&mut self, device: Device) -> Result<(), Error> {
        self.roundtrip(0x680 | device as u16, &[0x01, 0x01, 0x00, 0x01])
            .await?;
        Ok(())
    }

    /// Send reset command to a coil (0x01 0x01 0x00 0x00).
    pub(crate) async fn device_off(&mut self, device: Device) -> Result<(), Error> {
        self.roundtrip(0x680 | device as u16, &[0x01, 0x01, 0x00, 0x00])
            .await?;
        Ok(())
    }

    /// Set power level for a coil zone.
    /// Zone 1 = sub_cmd 0x04, zone 2 = sub_cmd 0x05 (coil B only).
    pub(crate) async fn set_power(
        &mut self,
        device: Device,
        zone: Zone,
        power: Power,
    ) -> Result<(), Error> {
        self.roundtrip(
            0x680 | device as u16,
            &[0x01, 0x04 + zone as u8, 0x00, power.as_raw()],
        )
        .await?;
        Ok(())
    }

    /// Send a status query and return the raw response.
    pub(crate) async fn query(&mut self, device: Device, reg: &[u8]) -> Result<[u8; 8], Error> {
        self.roundtrip(0x780 | device as u16, reg).await
    }

    pub(crate) async fn read_serial(&mut self, device: Device) -> Result<[u8; 4], Error> {
        let resp = self.query(device, &[0x00, 0x03, 0x00]).await?;
        Ok([resp[4], resp[5], resp[6], resp[7]])
    }

    pub(crate) async fn read_power_state(&mut self, device: Device) -> Result<u8, Error> {
        let resp = self.query(device, &[0x02, 0x05, 0x00]).await?;
        Ok(resp[4])
    }

    pub(crate) async fn read_config(&mut self, device: Device) -> Result<[u8; 4], Error> {
        let resp = self.query(device, &[0x00, 0x04, 0x00]).await?;
        Ok([resp[4], resp[5], resp[6], resp[7]])
    }

    pub(crate) async fn read_pcb_temp(&mut self, device: Device, zone: Zone) -> Result<u8, Error> {
        let reg = 0x03 + zone as u8;
        let resp = self.query(device, &[reg, 0x02, 0x00]).await?;
        Ok(resp[5])
    }

    pub(crate) async fn read_surface_temp(
        &mut self,
        device: Device,
        zone: Zone,
    ) -> Result<u8, Error> {
        let reg = 0x03 + zone as u8;
        let resp = self.query(device, &[reg, 0x04, 0x00]).await?;
        Ok(resp[5])
    }

    pub(crate) async fn read_reg_x3(
        &mut self,
        device: Device,
        zone: Zone,
    ) -> Result<[u8; 4], Error> {
        let reg = 0x03 + zone as u8;
        let resp = self.query(device, &[reg, 0x03, 0x00]).await?;
        Ok([resp[4], resp[5], resp[6], resp[7]])
    }

    pub(crate) async fn read_reg_x5(&mut self, device: Device, zone: Zone) -> Result<u8, Error> {
        let reg = 0x03 + zone as u8;
        let resp = self.query(device, &[reg, 0x05, 0x00]).await?;
        Ok(resp[4])
    }

    async fn roundtrip(&mut self, id: u16, data: &[u8]) -> Result<[u8; 8], Error> {
        STATE.next_resp.reset();
        STATE.next_resp_id.store(id - 0x80, Ordering::Release);

        if let Err(e) = self.send(id, data).await {
            STATE.next_resp_id.store(0, Ordering::Release);
            return Err(e);
        }

        let expected = id - 0x80;
        with_timeout(ACK_TIMEOUT, STATE.next_resp.wait())
            .await
            .map_err(|_| Error::AckTimeout(expected))
    }

    async fn send(&mut self, id: u16, data: &[u8]) -> Result<(), Error> {
        debug!("CAN tx: {:x} {:X}", id, data);

        let frame = twai::EspTwaiFrame::new(twai::StandardId::new(id).unwrap(), data).unwrap();
        match with_timeout(TX_TIMEOUT, self.can_tx.transmit_async(&frame)).await {
            Ok(Ok(())) => Ok(()),
            Ok(Err(e)) => Err(e.into()),
            Err(_) => Err(Error::TxTimeout),
        }
    }
}

#[embassy_executor::task]
pub(crate) async fn can_dispatch(mut rx: twai::TwaiRx<'static, Async>) {
    loop {
        let frame = match rx.receive_async().await {
            Ok(f) => f,
            Err(e) => {
                error!("CAN rx error: {:?}", e);
                continue;
            }
        };

        let id = match frame.id() {
            Id::Standard(v) => v.as_raw(),
            Id::Extended(v) => {
                error!("unexpected extended ID: {:x}", v.as_raw());
                continue;
            }
        };

        debug!("CAN rx: {:x} {:X}", id, frame.data());

        let mut resp = [0_u8; 8];
        resp[0..frame.data().len()].copy_from_slice(frame.data());

        if STATE
            .next_resp_id
            .compare_exchange(id, 0, Ordering::AcqRel, Ordering::Relaxed)
            .is_ok()
        {
            STATE.next_resp.signal(resp);
            continue;
        }

        // Ready signals (0x500 + device).
        match id {
            0x509 => STATE.left_ready.signal(()),
            0x513 => STATE.right_ready.signal(()),
            0x519 => STATE.psu_ready.signal(()),
            _ => {}
        }

        // Init signals (0x580 + device).
        match id {
            0x589 => STATE.left_init.signal(()),
            0x593 => STATE.right_init.signal(()),
            0x599 => STATE.psu_init.signal(()),
            _ => {}
        }

        // Announce acks (0x600 + device, byte 1 == 0x0B).
        if frame.data().len() >= 2 && frame.data()[1] == 0x0B {
            match id {
                0x609 => STATE.left_ack.signal(()),
                0x613 => STATE.right_ack.signal(()),
                0x619 => STATE.psu_ack.signal(()),
                _ => {}
            }
        }
    }
}
