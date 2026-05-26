#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![allow(clippy::large_stack_frames)]

use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, Either3, select, select3};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex},
    mutex::Mutex,
};
use embassy_time::{Duration, Instant, Timer};
use esp_hal::{
    Async, clock::CpuClock, gpio::Input, i2c::master::I2c, timer::timg::TimerGroup, twai,
};
use panic_rtt_target as _;
use static_cell::StaticCell;

use crate::{
    cook::{Burner, Pan, PanState, Power},
    iox::SwitchState,
    leds::{LedState, Pattern, Status},
};

esp_bootloader_esp_idf::esp_app_desc!();

type I2cBus = Mutex<NoopRawMutex, I2c<'static, Async>>;

static I2C_BUS: static_cell::StaticCell<I2cBus> = StaticCell::new();

mod cook;
mod heartbeat;
mod iox;
mod leds;

enum State {
    Fault { since: Instant },
    Off { last_input: SwitchState },
    On,
}

struct Controller<'a> {
    stove: cook::Stove<'a>,
    state: State,
    link_mode: bool,
    last_toggle: Instant,
    last_activity: Instant,
}

impl<'a> Controller<'a> {
    fn new(stove: cook::Stove<'a>, initial_switch_pos: SwitchState) -> Self {
        Self {
            stove,
            state: State::Off {
                last_input: initial_switch_pos,
            },
            link_mode: false,
            last_toggle: Instant::from_ticks(0),
            last_activity: Instant::now(),
        }
    }

    async fn startup(
        &mut self,
        switches: SwitchState,
        pans_rx: &mut embassy_sync::watch::Receiver<'static, CriticalSectionRawMutex, PanState, 2>,
    ) -> Result<(), cook::Error> {
        Timer::at(self.last_toggle + Duration::from_secs(1)).await;
        self.last_toggle = Instant::now();
        self.last_activity = Instant::now();
        self.stove.startup().await?;

        // Wait briefly for pan detection.
        let pans = match select(
            Timer::after_secs(1),
            pans_rx.changed_and(|&v| v != PanState::ABSENT),
        )
        .await
        {
            Either::First(_) => pans_rx.get().await,
            Either::Second(pans) => pans,
        };

        self.update_link_mode(pans);
        self.set_power(switches).await?;

        leds::LED_STATE
            .sender()
            .send(led_state(switches, pans, self.link_mode));

        self.state = State::On;
        Ok(())
    }

    async fn shutdown(&mut self, switches: SwitchState) {
        Timer::at(self.last_toggle + Duration::from_secs(1)).await;
        self.last_toggle = Instant::now();
        self.link_mode = false;

        if let Err(e) = self.stove.shutdown().await {
            error!("shutdown failed: {:?}", e);
            self.fault();
            return;
        }

        leds::LED_STATE
            .sender()
            .send(led_state(switches, PanState::ABSENT, false));
        self.state = State::Off {
            last_input: switches,
        };
    }

    fn fault(&mut self) {
        leds::LED_STATE.sender().send(LedState::fault());
        self.link_mode = false;
        self.state = State::Fault {
            since: Instant::now(),
        };

        // This disables pan detection and power.
        heartbeat::ENABLED.sender().send(false);
    }

    fn fault_reset(&mut self, switches: SwitchState) {
        info!("fault reset");
        leds::LED_STATE.sender().send(LedState::default());
        self.state = State::Off {
            last_input: switches,
        };

        heartbeat::ENABLED.sender().send(true);
    }

    /// Update link mode based on pan detection. Link mode activates
    /// when both the middle and right pans are placed within one
    /// second of each other.
    fn update_link_mode(&mut self, pans: PanState) {
        let (Pan::Present(middle), Pan::Present(right)) = (pans.middle, pans.right) else {
            self.link_mode = false;
            return;
        };

        let diff = if middle < right {
            right - middle
        } else {
            middle - right
        };

        self.link_mode = diff < Duration::from_secs(1);
    }

    async fn set_power(&mut self, switches: SwitchState) -> Result<(), cook::Error> {
        self.stove
            .set_power(Burner::Left, Power::from_dial(switches.left))
            .await?;
        self.stove
            .set_power(Burner::Right, Power::from_dial(switches.right))
            .await?;

        let middle_power = if self.link_mode {
            Power::from_dial(switches.right)
        } else {
            Power::from_dial(switches.middle)
        };

        self.stove.set_power(Burner::Middle, middle_power).await?;

        Ok(())
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // These GPIO pins are in use by the module.
    let _ = peripherals.GPIO24;
    let _ = peripherals.GPIO25;
    let _ = peripherals.GPIO26;
    let _ = peripherals.GPIO27;
    let _ = peripherals.GPIO28;
    let _ = peripherals.GPIO29;
    let _ = peripherals.GPIO30;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_interrupt =
        esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_interrupt.software_interrupt0);

    // i2c on pins (21, 7).
    let i2c = I2c::new(peripherals.I2C0, Default::default())
        .unwrap()
        .with_sda(peripherals.GPIO21)
        .with_scl(peripherals.GPIO7)
        .into_async();
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));

    let inta = Input::new(peripherals.GPIO6, Default::default());
    spawner.spawn(iox::monitor_switches(i2c_bus, inta).unwrap());
    spawner.spawn(leds::run(i2c_bus).unwrap());
    spawner.spawn(heartbeat::run(peripherals.LEDC, peripherals.GPIO23.into()).unwrap());

    // twai/CAN on pins (4, 5).
    let can_config = twai::TwaiConfiguration::new(
        peripherals.TWAI0,
        peripherals.GPIO5, // RXD
        peripherals.GPIO4, // TXD
        twai::BaudRate::B125K,
        twai::TwaiMode::Normal,
    );

    let (can_rx, can_tx) = can_config.into_async().start().split();
    spawner.spawn(cook::can_dispatch(can_rx).unwrap());

    let stove = cook::Stove::new(can_tx);
    let mut pans_rx = cook::PAN_STATE.receiver().unwrap();
    let mut sw_rx = iox::SWITCH_STATE.receiver().unwrap();
    let last_input = sw_rx.get().await;
    let mut ctrl = Controller::new(stove, last_input);

    info!("setup completed, querying cooktop");

    if let Err(e) = ctrl.stove.read_serial(cook::Node::Psu).await {
        error!("failed to query cooktop: {:?}", e);
        ctrl.fault();
    } else {
        heartbeat::ENABLED.sender().send(true);
    }

    loop {
        match ctrl.state {
            State::Fault { since } => {
                let st = match select(sw_rx.changed(), Timer::after_secs(1)).await {
                    Either::First(st) => st,
                    Either::Second(_) => sw_rx.get().await,
                };

                if since.elapsed() > Duration::from_secs(30) && st == SwitchState::OFF {
                    ctrl.fault_reset(st);
                }
            }
            State::Off { last_input } => {
                let switches = match select(sw_rx.changed(), Timer::after_secs(1)).await {
                    Either::First(switches) => switches,
                    Either::Second(_) => {
                        // todo poll temp?
                        let res = ctrl.stove.read_surface_temp(Burner::Left).await;
                        debug!("read_surface_temp: {:?}", res);
                        continue;
                    }
                };

                // todo safety switch?
                if switches != last_input && switches != SwitchState::OFF {
                    info!("powering on");
                    if let Err(e) = ctrl.startup(switches, &mut pans_rx).await {
                        error!("startup failed: {:?}", e);

                        // Attempt a normal shutdown.
                        if let Err(e) = ctrl.stove.shutdown().await {
                            error!("shutdown failed: {:?}", e);
                        }

                        ctrl.fault();
                        continue;
                    }
                }
            }
            State::On => {
                // 30m without activity kills the stove.
                const ACTIVITY_TIMEOUT: Duration = Duration::from_secs(30 * 60);

                let timeout = Timer::at(ctrl.last_activity + ACTIVITY_TIMEOUT);
                let (pans, switches) =
                    match select3(pans_rx.changed(), sw_rx.changed(), timeout).await {
                        Either3::First(pans) => (pans, sw_rx.get().await),
                        Either3::Second(switches) => {
                            ctrl.last_activity = Instant::now();
                            (pans_rx.get().await, switches)
                        }
                        Either3::Third(_) => {
                            info!("activity timeout, powering off");
                            let switches = sw_rx.try_get().unwrap_or_default();
                            ctrl.shutdown(switches).await;
                            continue;
                        }
                    };

                ctrl.update_link_mode(pans);
                debug!(
                    "pans: {:?}, switches: {:?}, link_mode: {}",
                    pans, switches, ctrl.link_mode
                );

                if switches == SwitchState::OFF {
                    info!("powering off");
                    ctrl.shutdown(switches).await;
                    continue;
                }

                if let Err(e) = ctrl.set_power(switches).await {
                    error!("set_power failed: {:?}", e);

                    // Attempt a normal shutdown.
                    if let Err(e) = ctrl.stove.shutdown().await {
                        error!("shutdown failed: {:?}", e);
                    }

                    ctrl.fault();
                    continue;
                }

                leds::LED_STATE
                    .sender()
                    .send(led_state(switches, pans, ctrl.link_mode));
            }
        }
    }
}

fn led_state(switches: SwitchState, pans: PanState, link_mode: bool) -> LedState {
    let (pan_l, st_l) = burner_led(switches.left, pans.left);
    let (pan_r, st_r) = burner_led(switches.right, pans.right);

    let (pan_m, st_m) = if switches.right > 0 && link_mode {
        (pan_r, st_r)
    } else {
        burner_led(switches.middle, pans.middle)
    };

    LedState {
        pan: [pan_l, pan_m, pan_r],
        status: [st_l, st_m, st_r],
        link_mode: if link_mode { Pattern::On } else { Pattern::Off },
    }
}

fn burner_led(sw: u8, pan: Pan) -> (Pattern, Status) {
    let pan_led = if matches!(pan, Pan::Present(_)) {
        Pattern::On
    } else if sw > 0 {
        Pattern::Slow
    } else {
        Pattern::Off
    };

    let status = if sw > 0 {
        Status::Green(Pattern::On)
    } else {
        // TODO
        // Status::Amber(Pattern::On, 128)
        Status::Green(Pattern::Off)
    };

    (pan_led, status)
}
