//! SPI0-SPI2: the legacy SPI master, a byte at a time through TXD and RXD
//! (no EasyDMA), on peripheral ID 3, 4 or 35 while ENABLE is 1, beside the
//! TWIM, TWI and SPIM that share those IDs.
//!
//! Writing TXD queues a byte (two at most, the register being double
//! buffered); each is shifted out in eight clocks at FREQUENCY while the
//! byte the selected device returns is shifted in, and lands in RXD (two
//! deep) with READY. Reading RXD takes the oldest. A device is selected
//! while the board's chip-select line for it, a GPIO the firmware drives,
//! is low; with none selected MISO reads 0xFF.
//!
//! Not modelled: CONFIG (order, phase, polarity: the device sees bytes),
//! ORC, the overrun of a full RXD (the next byte waits instead).

use std::collections::VecDeque;

use super::events::{self, Events};
use super::Ctx;

const READY: u32 = 2;

const ENABLE: u32 = 0x500;
const PSEL_SCK: u32 = 0x508;
const PSEL_MISO: u32 = 0x510;
const RXD: u32 = 0x518;
const TXD: u32 = 0x51C;
const FREQUENCY: u32 = 0x524;
const CONFIG: u32 = 0x554;

/// ENABLE for the legacy SPI master.
pub const ENABLED: u32 = 1;
const PS_PER_S: u64 = 1_000_000_000_000;

/// A device on a SPI bus.
pub trait SpiDevice: Send {
    /// Its chip-select line went low (`true`) or high.
    fn select(&mut self, selected: bool) {
        let _ = selected;
    }
    /// One byte each way while selected: what MOSI carried, what MISO returns.
    fn transfer(&mut self, mosi: u8) -> u8;
}

/// A device and the GPIO its chip select is on (port << 5 | pin).
pub struct SpiSlave {
    pub cs: u32,
    pub device: Box<dyn SpiDevice>,
    /// What the device was last told.
    pub selected: bool,
}

impl SpiSlave {
    pub fn new(cs: u32, device: Box<dyn SpiDevice>) -> Self {
        Self { cs, device, selected: false }
    }
}

/// The devices on one bus.
pub type SpiBus = Vec<SpiSlave>;

/// Whether `pin` (port << 5 | pin) is low in `levels`.
pub fn low(levels: &[u32; 2], pin: u32) -> bool {
    let port = (pin >> 5) as usize;
    port < 2 && levels[port] >> (pin & 31) & 1 == 0
}

pub struct Spi {
    pub events: Events,
    pub enable: u32,
    /// PSEL.SCK, MOSI, MISO.
    psel: [u32; 3],
    frequency: u32,
    config: u32,
    tx: VecDeque<u8>,
    rx: VecDeque<u8>,
    /// The byte being shifted, and when it is done.
    shifting: Option<(u8, u64)>,
}

impl Default for Spi {
    fn default() -> Self {
        Self::new()
    }
}

impl Spi {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            enable: 0,
            psel: [u32::MAX; 3],
            frequency: 0x0400_0000,
            config: 0,
            tx: VecDeque::new(),
            rx: VecDeque::new(),
            shifting: None,
        }
    }

    pub fn enabled(&self) -> bool {
        self.enable == ENABLED
    }

    /// The SCK pin, which names the bus.
    pub fn sck(&self) -> u32 {
        self.psel[0]
    }

    /// Eight clocks at FREQUENCY, whose top bits count 125 kbps.
    fn byte_time(&self) -> u64 {
        let bps = u64::from(self.frequency >> 25).max(1) * 125_000;
        8 * PS_PER_S / bps
    }

    /// Start the next byte if there is one and room for its answer.
    fn start(&mut self, now: u64) {
        if self.shifting.is_none() && self.rx.len() < 2 && self.enabled() {
            if let Some(byte) = self.tx.pop_front() {
                self.shifting = Some((byte, now + self.byte_time()));
            }
        }
    }

    /// Advances to `now`; the time of the next step. `bus` is the devices on
    /// this instance's SCK, `levels` the pins now.
    pub fn update(&mut self, ctx: &Ctx, bus: Option<&mut SpiBus>, levels: &[u32; 2]) -> Option<u64> {
        let mut bus = bus;
        while let Some((byte, end)) = self.shifting {
            if end > ctx.now {
                return Some(end);
            }
            let mut miso = 0xFF;
            if let Some(slaves) = bus.as_deref_mut() {
                for s in slaves.iter_mut().filter(|s| low(levels, s.cs)) {
                    if !s.selected {
                        s.selected = true;
                        s.device.select(true);
                    }
                    miso = s.device.transfer(byte);
                }
            }
            self.rx.push_back(miso);
            self.events.fire(READY);
            self.shifting = None;
            self.start(end);
        }
        None
    }

    pub fn read(&mut self, offset: u32) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            ENABLE => self.enable,
            o if (PSEL_SCK..=PSEL_MISO).contains(&o) => self.psel[((o - PSEL_SCK) / 4) as usize],
            RXD => u32::from(self.rx.pop_front().unwrap_or(0)),
            FREQUENCY => self.frequency,
            CONFIG => self.config,
            _ => 0,
        }
    }

    pub fn write(&mut self, offset: u32, value: u32, ctx: &Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        if events::task(offset, value).is_some() {
            return;
        }
        match offset {
            ENABLE => {
                self.enable = value & 0xF;
                if !self.enabled() {
                    self.tx.clear();
                    self.rx.clear();
                    self.shifting = None;
                }
            }
            o if (PSEL_SCK..=PSEL_MISO).contains(&o) => {
                self.psel[((o - PSEL_SCK) / 4) as usize] = value;
            }
            TXD => {
                if self.tx.len() < 2 {
                    self.tx.push_back(value as u8);
                }
                self.start(ctx.now);
            }
            FREQUENCY => self.frequency = value,
            CONFIG => self.config = value,
            _ => {}
        }
    }

    /// RXD was read: room for the next byte.
    pub fn resume(&mut self, now: u64) {
        self.start(now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    /// Answers each byte with its complement, and logs selects.
    struct Inverter(Arc<Mutex<Vec<String>>>);

    impl SpiDevice for Inverter {
        fn select(&mut self, selected: bool) {
            self.0.lock().unwrap().push(format!("cs {selected}"));
        }
        fn transfer(&mut self, mosi: u8) -> u8 {
            self.0.lock().unwrap().push(format!("{mosi:02x}"));
            !mosi
        }
    }

    fn spi() -> Spi {
        let mut s = Spi::new();
        let ctx = Ctx { now: 0 };
        s.write(PSEL_SCK, 6, &ctx);
        s.write(FREQUENCY, 0x8000_0000, &ctx); // 8 Mbps: a byte in 1 us
        s.write(ENABLE, ENABLED, &ctx);
        s
    }

    /// nrfx_spi's blocking transfer: TXD, wait READY, RXD, next.
    #[test]
    fn a_selected_device_answers_byte_for_byte() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let mut bus = vec![SpiSlave::new(16, Box::new(Inverter(log.clone())))];
        let cs_low = [!(1u32 << 16), 0];
        let mut s = spi();
        s.write(TXD, 0x90, &Ctx { now: 0 });
        s.write(TXD, 0x00, &Ctx { now: 0 });
        assert_eq!(s.update(&Ctx { now: 0 }, Some(&mut bus), &cs_low), Some(1_000_000));
        s.update(&Ctx { now: 1_000_000 }, Some(&mut bus), &cs_low);
        assert!(s.events.is_set(READY));
        assert_eq!(s.read(RXD), 0x6F);
        s.update(&Ctx { now: 2_000_000 }, Some(&mut bus), &cs_low);
        assert_eq!(s.read(RXD), 0xFF);
        assert_eq!(*log.lock().unwrap(), ["cs true", "90", "00"]);
    }

    #[test]
    fn nobody_selected_reads_ff() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let mut bus = vec![SpiSlave::new(16, Box::new(Inverter(log.clone())))];
        let mut s = spi();
        s.write(TXD, 0x05, &Ctx { now: 0 });
        s.update(&Ctx { now: 1_000_000 }, Some(&mut bus), &[u32::MAX, 0]);
        assert_eq!(s.read(RXD), 0xFF);
        assert!(log.lock().unwrap().is_empty());
    }
}
