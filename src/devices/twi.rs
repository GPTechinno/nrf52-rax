//! TWI0 and TWI1: the legacy I2C master, a byte at a time through TXD and
//! RXD (no EasyDMA), on the same buses and devices as the TWIM. It shares
//! peripheral ID 3 or 4 with the TWIM and is the instance while ENABLE is 5.
//!
//! STARTTX sends a START and the address for writing, then the byte in TXD
//! if one was written, and each byte written after; every byte gives
//! TXDSENT. STARTRX sends a START and the address for reading, then
//! receives bytes, each giving RXDREADY with RXD. Every byte also gives BB
//! (byte boundary), whose shorts BB_SUSPEND and BB_STOP are how a driver
//! holds the bus after a byte (RESUME goes on) or ends it there. STOP sends
//! the STOP condition after the byte in flight and gives STOPPED; SUSPEND
//! gives SUSPENDED. An address or data byte not acknowledged gives ERROR
//! with ERRORSRC (ANACK, DNACK) and holds the bus until STOP. A byte takes
//! nine clocks.
//!
//! Not modelled: clock stretching, arbitration, OVERRUN.

use std::collections::BTreeMap;

use super::events::{self, Events};
use super::twim::{I2cBus, I2cDevice};
use super::Ctx;

const STOPPED: u32 = 1;
const RXDREADY: u32 = 2;
const TXDSENT: u32 = 7;
const ERROR: u32 = 9;
const BB: u32 = 14;
const SUSPENDED: u32 = 18;

const SHORT_BB_SUSPEND: u32 = 0;
const SHORT_BB_STOP: u32 = 1;

const ERRORSRC: u32 = 0x4C4;
const ENABLE: u32 = 0x500;
const PSEL_SCL: u32 = 0x508;
const PSEL_SDA: u32 = 0x50C;
const RXD: u32 = 0x518;
const TXD: u32 = 0x51C;
const FREQUENCY: u32 = 0x524;
const ADDRESS: u32 = 0x588;

/// ENABLE for the legacy TWI.
pub const ENABLED: u32 = 5;
const ANACK: u32 = 1 << 1;
const DNACK: u32 = 1 << 2;
const PS_PER_S: u128 = 1_000_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    /// A START and the address, until `end`.
    Address { read: bool, end: u64 },
    /// A byte to the device, until `end`.
    Sending { byte: u8, end: u64 },
    /// A byte from the device, until `end`.
    Receiving { end: u64 },
    /// Transmitting, with no byte in TXD: the bus waits for one.
    Waiting,
    /// The bus is held: after an error, a SUSPEND or a BB_SUSPEND.
    Held { read: bool },
    /// The STOP condition, until `end`.
    Stopping { end: u64 },
}

/// The device at `address` on the bus whose SCL is `scl`.
fn device<'a>(
    buses: &'a mut BTreeMap<u32, I2cBus>,
    scl: u32,
    address: u8,
) -> Option<&'a mut Box<dyn I2cDevice>> {
    buses.get_mut(&scl).and_then(|bus| bus.get_mut(&address))
}

pub struct Twi {
    pub events: Events,
    pub enable: u32,
    psel: [u32; 2],
    frequency: u32,
    errorsrc: u32,
    rxd: u8,
    /// A byte written to TXD and not sent yet.
    txd: Option<u8>,
    address: u32,
    phase: Phase,
    /// STOP was triggered during a byte: the STOP follows it.
    stop_after: bool,
    /// The address of the device the last START reached.
    addressed: Option<u8>,
}

impl Default for Twi {
    fn default() -> Self {
        Self::new()
    }
}

impl Twi {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            enable: 0,
            psel: [u32::MAX; 2],
            frequency: 0x0668_0000,
            errorsrc: 0,
            rxd: 0,
            txd: None,
            address: 0,
            phase: Phase::Idle,
            stop_after: false,
            addressed: None,
        }
    }

    pub fn enabled(&self) -> bool {
        self.enable == ENABLED
    }

    /// The SCL pin (PSEL.SCL), which names the bus.
    pub fn scl(&self) -> u32 {
        self.psel[0]
    }

    /// Nine clocks: a byte and its acknowledge. FREQUENCY is the rate ×
    /// 2^32 / 16 MHz, as for the TWIM.
    fn byte_time(&self) -> u64 {
        let hz = ((u128::from(self.frequency) * 16_000_000) >> 32).max(1);
        9 * (PS_PER_S / hz) as u64
    }

    fn task(&mut self, index: u32, now: u64) {
        if !self.enabled() {
            return;
        }
        let end = now + self.byte_time();
        match index {
            // STARTRX, STARTTX
            0 | 2 => {
                self.stop_after = false;
                self.phase = Phase::Address { read: index == 0, end };
            }
            // STOP
            5 => match self.phase {
                Phase::Idle => self.events.fire(STOPPED),
                Phase::Address { .. } | Phase::Sending { .. } | Phase::Receiving { .. } => {
                    self.stop_after = true;
                }
                _ => self.phase = Phase::Stopping { end: now + self.byte_time() / 9 },
            },
            // SUSPEND
            7 => {
                self.events.fire(SUSPENDED);
                let read = matches!(self.phase, Phase::Receiving { .. } | Phase::Held { read: true });
                if !matches!(self.phase, Phase::Idle | Phase::Stopping { .. }) {
                    self.phase = Phase::Held { read };
                }
            }
            // RESUME
            8 => match self.phase {
                Phase::Held { read: true } => self.phase = Phase::Receiving { end },
                Phase::Held { read: false } => self.next_byte(now),
                _ => {}
            },
            _ => {}
        }
    }

    /// Transmitting: send the byte in TXD, or wait for one.
    fn next_byte(&mut self, now: u64) {
        self.phase = match self.txd.take() {
            Some(byte) => Phase::Sending { byte, end: now + self.byte_time() },
            None => Phase::Waiting,
        };
    }

    /// A byte is done: BB and its shorts. Returns whether the bus goes on.
    fn boundary(&mut self, end: u64, read: bool) -> bool {
        self.events.fire(BB);
        if self.stop_after || self.events.short(SHORT_BB_STOP) {
            self.stop_after = false;
            self.phase = Phase::Stopping { end: end + self.byte_time() / 9 };
            return false;
        }
        if self.events.short(SHORT_BB_SUSPEND) {
            self.events.fire(SUSPENDED);
            self.phase = Phase::Held { read };
            return false;
        }
        true
    }

    fn fail(&mut self, source: u32, read: bool) {
        self.errorsrc |= source;
        self.events.fire(ERROR);
        self.phase = Phase::Held { read };
        if self.stop_after {
            self.stop_after = false;
            self.phase = Phase::Stopping { end: 0 };
        }
    }

    /// Advances the bus to `now`; returns the time of its next step.
    pub fn update(&mut self, ctx: &Ctx, buses: &mut BTreeMap<u32, I2cBus>) -> Option<u64> {
        loop {
            let address = (self.address & 0x7F) as u8;
            let scl = self.scl();
            match self.phase {
                Phase::Address { read, end } if end <= ctx.now => {
                    if device(buses, scl, address).is_some_and(|d| d.start(read)) {
                        self.addressed = Some(address);
                        if self.stop_after {
                            self.stop_after = false;
                            self.phase = Phase::Stopping { end };
                        } else if read {
                            self.phase = Phase::Receiving { end: end + self.byte_time() };
                        } else {
                            self.next_byte(end);
                        }
                    } else {
                        self.addressed = None;
                        self.fail(ANACK, read);
                    }
                }
                Phase::Sending { byte, end } if end <= ctx.now => {
                    let acked = device(buses, scl, address).is_some_and(|d| d.write(byte));
                    if !acked {
                        self.fail(DNACK, false);
                        continue;
                    }
                    self.events.fire(TXDSENT);
                    if self.boundary(end, false) {
                        self.next_byte(end);
                    }
                }
                Phase::Receiving { end } if end <= ctx.now => {
                    self.rxd = device(buses, scl, address).map_or(0xFF, |d| d.read());
                    self.events.fire(RXDREADY);
                    if self.boundary(end, true) {
                        self.phase = Phase::Receiving { end: end + self.byte_time() };
                    }
                }
                Phase::Stopping { end } if end <= ctx.now => {
                    if let Some(address) = self.addressed.take()
                        && let Some(d) = buses.get_mut(&scl).and_then(|bus| bus.get_mut(&address))
                    {
                        d.stop();
                    }
                    self.events.fire(STOPPED);
                    self.phase = Phase::Idle;
                }
                Phase::Address { end, .. }
                | Phase::Sending { end, .. }
                | Phase::Receiving { end }
                | Phase::Stopping { end } => return Some(end),
                Phase::Idle | Phase::Waiting | Phase::Held { .. } => return None,
            }
        }
    }

    pub fn read(&mut self, offset: u32) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            ERRORSRC => self.errorsrc,
            ENABLE => self.enable,
            PSEL_SCL | PSEL_SDA => self.psel[((offset - PSEL_SCL) / 4) as usize],
            RXD => u32::from(self.rxd),
            TXD => u32::from(self.txd.unwrap_or(0)),
            FREQUENCY => self.frequency,
            ADDRESS => self.address,
            _ => 0,
        }
    }

    pub fn write(&mut self, offset: u32, value: u32, ctx: &Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        if let Some(index) = events::task(offset, value) {
            self.task(index, ctx.now);
            return;
        }
        match offset {
            // write 1 to clear
            ERRORSRC => self.errorsrc &= !value,
            ENABLE => self.enable = value & 0xF,
            PSEL_SCL | PSEL_SDA => self.psel[((offset - PSEL_SCL) / 4) as usize] = value,
            TXD => {
                self.txd = Some(value as u8);
                if self.phase == Phase::Waiting {
                    self.next_byte(ctx.now);
                }
            }
            FREQUENCY => self.frequency = value,
            ADDRESS => self.address = value & 0x7F,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::twim::I2cDevice;
    use std::sync::{Arc, Mutex};

    /// A register file behind a pointer, as most sensors are.
    struct Regs {
        at: u8,
        mem: [u8; 4],
        log: Arc<Mutex<Vec<String>>>,
    }

    impl I2cDevice for Regs {
        fn write(&mut self, byte: u8) -> bool {
            self.log.lock().unwrap().push(format!("w{byte:02x}"));
            self.at = byte & 3;
            true
        }
        fn read(&mut self) -> u8 {
            let b = self.mem[self.at as usize];
            self.at = (self.at + 1) & 3;
            b
        }
        fn stop(&mut self) {
            self.log.lock().unwrap().push("stop".into());
        }
    }

    fn setup() -> (Twi, BTreeMap<u32, I2cBus>, Arc<Mutex<Vec<String>>>) {
        let log = Arc::new(Mutex::new(Vec::new()));
        let mut bus = I2cBus::new();
        bus.insert(0x76, Box::new(Regs { at: 0, mem: [0x11, 0x22, 0x33, 0x44], log: log.clone() }));
        let mut buses = BTreeMap::new();
        buses.insert(26, bus);
        let mut twi = Twi::new();
        let ctx = Ctx { now: 0 };
        twi.write(ENABLE, ENABLED, &ctx);
        twi.write(PSEL_SCL, 26, &ctx);
        twi.write(PSEL_SDA, 25, &ctx);
        twi.write(ADDRESS, 0x76, &ctx);
        (twi, buses, log)
    }

    /// Run until nothing is scheduled; returns the time reached.
    fn settle(twi: &mut Twi, buses: &mut BTreeMap<u32, I2cBus>, mut now: u64) -> u64 {
        while let Some(t) = twi.update(&Ctx { now }, buses) {
            now = t;
        }
        now
    }

    /// nrf_drv_twi's write: TXD then STARTTX, TXD again on TXDSENT, STOP.
    #[test]
    fn a_write_sends_each_byte_written() {
        let (mut twi, mut buses, log) = setup();
        let ctx = Ctx { now: 0 };
        twi.write(TXD, 0x02, &ctx);
        twi.write(0x008, 1, &ctx); // STARTTX
        let t = settle(&mut twi, &mut buses, 0);
        assert!(twi.events.is_set(TXDSENT));
        twi.write(EVENT_TXDSENT, 0, &Ctx { now: t });
        twi.write(0x014, 1, &Ctx { now: t }); // STOP
        settle(&mut twi, &mut buses, t);
        assert!(twi.events.is_set(STOPPED));
        assert_eq!(*log.lock().unwrap(), ["w02", "stop"]);
    }

    /// nrf_drv_twi's read: BB_SUSPEND, STARTRX, RXD then RESUME; BB_STOP
    /// for the last byte.
    #[test]
    fn a_read_suspends_at_each_byte_and_stops_on_the_last() {
        let (mut twi, mut buses, log) = setup();
        let mut now = 0;
        twi.write(0x200, 1 << SHORT_BB_SUSPEND, &Ctx { now });
        twi.write(0x000, 1, &Ctx { now }); // STARTRX
        now = settle(&mut twi, &mut buses, now);
        assert!(twi.events.is_set(RXDREADY) && twi.events.is_set(SUSPENDED));
        assert_eq!(twi.read(RXD), 0x11);
        twi.write(EVENT_RXDREADY, 0, &Ctx { now });
        twi.write(0x200, 1 << SHORT_BB_STOP, &Ctx { now });
        twi.write(0x020, 1, &Ctx { now }); // RESUME
        settle(&mut twi, &mut buses, now);
        assert_eq!(twi.read(RXD), 0x22);
        assert!(twi.events.is_set(STOPPED));
        assert_eq!(*log.lock().unwrap(), ["stop"]);
    }

    #[test]
    fn nobody_at_the_address_is_an_error() {
        let (mut twi, mut buses, _) = setup();
        twi.write(ADDRESS, 0x50, &Ctx { now: 0 });
        twi.write(0x008, 1, &Ctx { now: 0 });
        settle(&mut twi, &mut buses, 0);
        assert!(twi.events.is_set(ERROR));
        assert_eq!(twi.read(ERRORSRC), ANACK);
    }

    const EVENT_TXDSENT: u32 = 0x100 + 4 * TXDSENT;
    const EVENT_RXDREADY: u32 = 0x100 + 4 * RXDREADY;
}
