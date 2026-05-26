use core::sync::atomic::{AtomicU16, Ordering};

use defmt::{debug, error, info};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal, watch::Watch};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_can::{Frame, Id};
use esp_hal::{Async, twai};

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

async fn wait_ack(signal: &Signal<CriticalSectionRawMutex, ()>, id: u16) -> Result<(), Error> {
    with_timeout(ACK_TIMEOUT, signal.wait())
        .await
        .map_err(|_| Error::AckTimeout(id))
}

/// A CAN node on the bus. The right coil board is a single node
/// that drives two burners; the PSU is a node that heats nothing.
#[repr(u8)]
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub(crate) enum Node {
    LeftCoil = 0x09,
    RightCoil = 0x13,
    Psu = 0x19,
}

/// A user-facing heating element. Each has its own dial, pan
/// detection, and surface temperature.
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
#[repr(usize)]
pub(crate) enum Burner {
    Left = 0,
    Middle = 1,
    Right = 2,
}

impl Burner {
    pub(crate) const ALL: [Burner; 3] = [Burner::Left, Burner::Middle, Burner::Right];

    fn node(self) -> Node {
        match self {
            Burner::Left => Node::LeftCoil,
            Burner::Middle | Burner::Right => Node::RightCoil,
        }
    }

    fn power_subcmd(self) -> u8 {
        match self {
            Burner::Left | Burner::Middle => 0x04,
            Burner::Right => 0x05,
        }
    }

    fn temperature_subcmd(self) -> u8 {
        match self {
            Burner::Left | Burner::Middle => 0x03,
            Burner::Right => 0x04,
        }
    }
}

#[derive(Debug, Clone, Copy, Eq, PartialEq)]
pub(crate) enum Power {
    Off,
    Standby,
    Level(u8),
    Boost,
}

impl Power {
    /// Map a raw dial position (0–11) to a power command.
    pub(crate) fn from_dial(position: u8) -> Self {
        match position {
            0 => Power::Standby,
            1 => Power::Level(3),
            2 => Power::Level(5),
            3 => Power::Level(6),
            4 => Power::Level(7),
            5 => Power::Level(8),
            6 => Power::Level(9),
            7 => Power::Level(10),
            8 => Power::Level(12),
            9 => Power::Level(14),
            10 => Power::Boost,
            _ => Power::Standby,
        }
    }

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

type Sig = Signal<CriticalSectionRawMutex, ()>;

/// Per-burner pan detection state.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum Pan {
    // The pan has been present since this time.
    Present(Instant),
    // The pan is absent.
    Absent,
}

impl Pan {
    fn update(&mut self, detected: bool, now: Instant) {
        *self = match (*self, detected) {
            (Pan::Present(t), true) => Pan::Present(t),
            (Pan::Absent, true) => Pan::Present(now),
            (_, false) => Pan::Absent,
        };
    }
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct PanState {
    pub(crate) left: Pan,
    pub(crate) middle: Pan,
    pub(crate) right: Pan,
}

impl PanState {
    pub(crate) const ABSENT: Self = Self {
        left: Pan::Absent,
        middle: Pan::Absent,
        right: Pan::Absent,
    };
}

impl Default for PanState {
    fn default() -> Self {
        Self::ABSENT
    }
}

pub(crate) static PAN_STATE: Watch<CriticalSectionRawMutex, PanState, 2> =
    Watch::new_with(PanState::ABSENT);

/// Per-burner status readings.
#[derive(Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub(crate) struct BurnerStatus {
    /// PCB / ambient temperature in °C.
    pub(crate) pcb: u8,
    /// Cooking surface temperature in °C.
    pub(crate) surface: u8,
    /// Unknown register (byte 1 = 0x03), 4 bytes. Always 0 in
    /// captures so far.
    pub(crate) reg_x3: [u8; 4],
    /// Unknown register (byte 1 = 0x05), 1 byte. Always 0 in
    /// captures so far.
    pub(crate) reg_x5: u8,
}

static NEXT_RESP_ID: AtomicU16 = AtomicU16::new(0);
static NEXT_RESP: Signal<CriticalSectionRawMutex, [u8; 8]> = Signal::new();

// Ready signals (0x500 + device, empty frame).
static PSU_READY: Sig = Signal::new();
static LEFT_READY: Sig = Signal::new();
static RIGHT_READY: Sig = Signal::new();

// Announce acks (0x600 + device, [00 0B 00 01]).
static PSU_ACK: Sig = Signal::new();
static LEFT_ACK: Sig = Signal::new();
static RIGHT_ACK: Sig = Signal::new();

pub(crate) struct Stove<'a> {
    can_tx: twai::TwaiTx<'a, Async>,
}

impl<'a> Stove<'a> {
    pub(crate) fn new(can_tx: twai::TwaiTx<'a, Async>) -> Self {
        Self { can_tx }
    }

    pub(crate) async fn startup(&mut self) -> Result<(), Error> {
        info!("cooktop system init");

        PSU_READY.reset();
        LEFT_READY.reset();
        RIGHT_READY.reset();

        self.send(0x680, &[0x00, 0x01, 0x00, 0x06]).await?;
        Timer::after(Duration::from_millis(40)).await;
        self.send(0x581, &[]).await?;

        wait_ack(&LEFT_READY, 0x509).await?;
        wait_ack(&RIGHT_READY, 0x513).await?;
        wait_ack(&PSU_READY, 0x519).await?;
        debug!("all devices ready");

        // Controller announce.
        PSU_ACK.reset();
        LEFT_ACK.reset();
        RIGHT_ACK.reset();
        self.send(0x681, &[0x00, 0x0B, 0x00, 0x00]).await?;
        wait_ack(&LEFT_ACK, 0x609).await?;
        wait_ack(&RIGHT_ACK, 0x613).await?;
        wait_ack(&PSU_ACK, 0x619).await?;
        debug!("all devices acked announce");

        self.node_on(Node::LeftCoil).await?;
        self.set_power(Burner::Left, Power::Standby).await?;

        self.node_on(Node::RightCoil).await?;
        self.set_power(Burner::Middle, Power::Standby).await?;
        self.set_power(Burner::Right, Power::Standby).await?;

        // Configure maximum power (3700W?).
        for node in [Node::LeftCoil, Node::RightCoil] {
            self.roundtrip(0x680 | node as u16, &[0x01, 0x06, 0x00, 0x0E, 0x74])
                .await?;
        }

        // Final announce.
        PSU_ACK.reset();
        LEFT_ACK.reset();
        RIGHT_ACK.reset();

        self.send(0x681, &[0x00, 0x0B, 0x00, 0x00]).await?;
        wait_ack(&LEFT_ACK, 0x609).await?;
        wait_ack(&RIGHT_ACK, 0x613).await?;
        wait_ack(&PSU_ACK, 0x619).await?;
        info!("startup complete");

        Ok(())
    }

    pub(crate) async fn shutdown(&mut self) -> Result<(), Error> {
        info!("cooktop shutdown");

        // Set power off.
        self.set_power(Burner::Left, Power::Off).await?;
        self.set_power(Burner::Middle, Power::Off).await?;
        self.set_power(Burner::Right, Power::Off).await?;

        // Reset coil boards.
        self.roundtrip(0x680 | Node::LeftCoil as u16, &[0x01, 0x01, 0x00, 0x00])
            .await?;
        self.roundtrip(0x680 | Node::RightCoil as u16, &[0x01, 0x01, 0x00, 0x00])
            .await?;

        // PSU shutdown.
        self.roundtrip(0x680 | Node::Psu as u16, &[0x01, 0x03, 0x00, 0x02])
            .await?;
        self.roundtrip(0x680 | Node::Psu as u16, &[0x01, 0x02, 0x00, 0x00])
            .await?;

        // Final announce.
        self.send(0x681, &[0x00, 0x0B, 0x00, 0x00]).await?;
        info!("shutdown complete");
        Ok(())
    }

    /// Send init command to a coil board (0x01 0x01 0x00 0x01).
    pub(crate) async fn node_on(&mut self, node: Node) -> Result<(), Error> {
        self.roundtrip(0x680 | node as u16, &[0x01, 0x01, 0x00, 0x01])
            .await?;
        Ok(())
    }

    /// Send reset command to a coil board (0x01 0x01 0x00 0x00).
    pub(crate) async fn node_off(&mut self, node: Node) -> Result<(), Error> {
        self.roundtrip(0x680 | node as u16, &[0x01, 0x01, 0x00, 0x00])
            .await?;
        Ok(())
    }

    /// Set the power level for a burner.
    pub(crate) async fn set_power(&mut self, burner: Burner, power: Power) -> Result<(), Error> {
        self.roundtrip(
            0x680 | burner.node() as u16,
            &[0x01, burner.power_subcmd(), 0x00, power.as_raw()],
        )
        .await?;
        Ok(())
    }

    /// Send a status query to a node and return the raw response.
    pub(crate) async fn query(&mut self, node: Node, reg: &[u8]) -> Result<[u8; 8], Error> {
        self.roundtrip(0x780 | node as u16, reg).await
    }

    pub(crate) async fn read_serial(&mut self, node: Node) -> Result<[u8; 4], Error> {
        let resp = self.query(node, &[0x00, 0x03, 0x00]).await?;
        Ok([resp[4], resp[5], resp[6], resp[7]])
    }

    pub(crate) async fn read_power_state(&mut self, node: Node) -> Result<u8, Error> {
        let resp = self.query(node, &[0x02, 0x05, 0x00]).await?;
        Ok(resp[4])
    }

    pub(crate) async fn read_config(&mut self, node: Node) -> Result<[u8; 4], Error> {
        let resp = self.query(node, &[0x00, 0x04, 0x00]).await?;
        Ok([resp[4], resp[5], resp[6], resp[7]])
    }

    pub(crate) async fn read_pcb_temp(&mut self, burner: Burner) -> Result<u8, Error> {
        let resp = self
            .query(burner.node(), &[burner.temperature_subcmd(), 0x02, 0x00])
            .await?;
        Ok(resp[5])
    }

    pub(crate) async fn read_surface_temp(&mut self, burner: Burner) -> Result<u8, Error> {
        let resp = self
            .query(burner.node(), &[burner.temperature_subcmd(), 0x04, 0x00])
            .await?;
        Ok(resp[5])
    }

    pub(crate) async fn read_reg_x3(&mut self, burner: Burner) -> Result<[u8; 4], Error> {
        let resp = self
            .query(burner.node(), &[burner.temperature_subcmd(), 0x03, 0x00])
            .await?;
        Ok([resp[4], resp[5], resp[6], resp[7]])
    }

    pub(crate) async fn read_reg_x5(&mut self, burner: Burner) -> Result<u8, Error> {
        let resp = self
            .query(burner.node(), &[burner.temperature_subcmd(), 0x05, 0x00])
            .await?;
        Ok(resp[4])
    }

    async fn roundtrip(&mut self, id: u16, data: &[u8]) -> Result<[u8; 8], Error> {
        NEXT_RESP.reset();
        NEXT_RESP_ID.store(id - 0x80, Ordering::Release);

        if let Err(e) = self.send(id, data).await {
            NEXT_RESP_ID.store(0, Ordering::Release);
            return Err(e);
        }

        let expected = id - 0x80;
        with_timeout(ACK_TIMEOUT, NEXT_RESP.wait())
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

        if NEXT_RESP_ID
            .compare_exchange(id, 0, Ordering::AcqRel, Ordering::Relaxed)
            .is_ok()
        {
            NEXT_RESP.signal(resp);
            continue;
        }

        // Pan detection (0x480 + node offset).
        //
        // 0x489 (left coil):  byte 2 = 0x01 → Left has pan.
        // 0x493 (right coil): byte 2 = 0x01 → Middle has pan,
        //                     byte 2 = 0x02 → Right has pan.
        if (id == 0x489 || id == 0x493)
            && frame.data().len() >= 3
            && frame.data()[0] == 0x02
            && frame.data()[1] == 0x05
        {
            let pan_val = frame.data()[2];
            let now = Instant::now();
            let tx = PAN_STATE.sender();
            let mut state = tx.try_get().unwrap_or_default();

            if id == 0x489 {
                state.left.update(pan_val == 0x01, now);
            } else {
                state.middle.update(pan_val == 0x01, now);
                state.right.update(pan_val == 0x02, now);
            }

            tx.send(state);
        }

        // Ready signals (0x500 + device).
        match id {
            0x509 => LEFT_READY.signal(()),
            0x513 => RIGHT_READY.signal(()),
            0x519 => PSU_READY.signal(()),
            _ => (),
        }

        // Announce acks (0x600 + device, byte 1 == 0x0B).
        if frame.data().len() >= 2 && frame.data()[1] == 0x0B {
            match id {
                0x609 => LEFT_ACK.signal(()),
                0x613 => RIGHT_ACK.signal(()),
                0x619 => PSU_ACK.signal(()),
                _ => (),
            }
        }
    }
}
