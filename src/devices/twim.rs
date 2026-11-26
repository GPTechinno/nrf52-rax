//! TWIM0 and TWIM1: the I2C master with EasyDMA, on the devices the board
//! puts on its buses. A bus is named by the pin of its SCL line (port << 5
//! | pin), so a firmware finds a device wherever it routes the instance.
//!
//! STARTTX (TXSTARTED) addresses the device for writing and sends the
//! TXD.MAXCNT bytes, STARTRX (RXSTARTED) addresses it for reading and
//! receives RXD.MAXCNT bytes, each after a (repeated) START. A phase takes
//! its bytes' time on the bus (9 clocks per byte, the address included),
//! then gives LASTTX or LASTRX and AMOUNT, and its shorts (LASTTX_STARTRX,
//! LASTTX_SUSPEND, LASTTX_STOP, LASTRX_STARTTX, LASTRX_SUSPEND,
//! LASTRX_STOP) take effect. An address or data byte not acknowledged gives
//! ERROR with ERRORSRC (ANACK, DNACK) and holds the bus until STOP. STOP
//! sends the STOP condition and gives STOPPED; SUSPEND gives SUSPENDED.
//!
//! Not modelled: clock stretching, arbitration, the EasyDMA list, the
//! legacy TWI.

use std::collections::BTreeMap;

use super::events::{self, Events};
use super::{Ctx, Dma};

/// A device on an I2C bus.
pub trait I2cDevice: Send {
    /// A START (or repeated START) with the device's address, for reading or
    /// writing; whether the device acknowledges it.
    fn start(&mut self, read: bool) -> bool {
        let _ = read;
        true
    }
    /// A byte written to the device; whether it acknowledges it.
    fn write(&mut self, byte: u8) -> bool;
    /// A byte read from the device.
    fn read(&mut self) -> u8;
    /// A STOP condition.
    fn stop(&mut self) {}
}

/// The devices on one bus, by 7-bit address.
pub type I2cBus = BTreeMap<u8, Box<dyn I2cDevice>>;

const STOPPED: u32 = 1;
const ERROR: u32 = 9;
const SUSPENDED: u32 = 18;
const RXSTARTED: u32 = 19;
const TXSTARTED: u32 = 20;
const LASTRX: u32 = 23;
const LASTTX: u32 = 24;

const SHORT_LASTTX_STARTRX: u32 = 7;
const SHORT_LASTTX_SUSPEND: u32 = 8;
const SHORT_LASTTX_STOP: u32 = 9;
const SHORT_LASTRX_STARTTX: u32 = 10;
const SHORT_LASTRX_SUSPEND: u32 = 11;
const SHORT_LASTRX_STOP: u32 = 12;

const ERRORSRC: u32 = 0x4C4;
const ENABLE: u32 = 0x500;
const PSEL_SCL: u32 = 0x508;
const PSEL_SDA: u32 = 0x50C;
const FREQUENCY: u32 = 0x524;
const RXD_PTR: u32 = 0x534;
const RXD_LIST: u32 = 0x540;
const TXD_PTR: u32 = 0x544;
const TXD_LIST: u32 = 0x550;
const ADDRESS: u32 = 0x588;

/// ENABLE for the TWIM.
pub const ENABLED: u32 = 6;
const ANACK: u32 = 1 << 1;
const DNACK: u32 = 1 << 2;
const PS_PER_S: u128 = 1_000_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Phase {
    Idle,
    /// STARTTX or STARTRX was triggered: the transfer is done at the next
    /// update.
    Starting {
        read: bool,
    },
    /// A transfer on the bus until `end`; then LASTTX/LASTRX, or ERROR.
    Transfer {
        read: bool,
        end: u64,
        error: bool,
    },
    /// The bus is held (after a phase without a short, a suspend or an
    /// error).
    Held,
    /// The STOP condition, until `end`.
    Stopping {
        end: u64,
    },
}

pub struct Twim {
    pub events: Events,
    pub enable: u32,
    psel: [u32; 2],
    frequency: u32,
    errorsrc: u32,
    /// PTR, MAXCNT, AMOUNT, LIST.
    rxd: [u32; 4],
    txd: [u32; 4],
    address: u32,
    count_mask: u32,
    phase: Phase,
    /// The address of the device the last START reached.
    addressed: Option<u8>,
}

impl Twim {
    pub fn new(count_mask: u32) -> Self {
        Self {
            events: Events::default(),
            enable: 0,
            psel: [u32::MAX; 2],
            frequency: 0x0400_0000,
            errorsrc: 0,
            rxd: [0; 4],
            txd: [0; 4],
            address: 0,
            count_mask,
            phase: Phase::Idle,
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

    /// One bit on the bus, picoseconds: FREQUENCY is the rate × 2^32 / 16 MHz.
    fn bit_time(&self) -> u64 {
        let hz = ((u128::from(self.frequency) * 16_000_000) >> 32).max(1);
        (PS_PER_S / hz) as u64
    }

    fn task(&mut self, index: u32, now: u64) {
        if !self.enabled() {
            return;
        }
        match index {
            0 => {
                self.events.fire(RXSTARTED);
                self.phase = Phase::Starting { read: true };
            }
            2 => {
                self.events.fire(TXSTARTED);
                self.phase = Phase::Starting { read: false };
            }
            5 => {
                self.phase = if self.addressed.is_some() || self.phase != Phase::Idle {
                    Phase::Stopping {
                        end: now + self.bit_time(),
                    }
                } else {
                    self.events.fire(STOPPED);
                    Phase::Idle
                };
            }
            7 => {
                self.events.fire(SUSPENDED);
                if !matches!(self.phase, Phase::Stopping { .. }) {
                    self.phase = Phase::Held;
                }
            }
            _ => {}
        }
    }

    /// Runs a phase on the bus: the address, then the bytes.
    fn transfer(
        &mut self,
        read: bool,
        now: u64,
        mem: &mut dyn Dma,
        buses: &mut BTreeMap<u32, I2cBus>,
    ) {
        let address = (self.address & 0x7F) as u8;
        let device = buses
            .get_mut(&self.scl())
            .and_then(|bus| bus.get_mut(&address));
        let regs = if read { &mut self.rxd } else { &mut self.txd };
        let (ptr, maxcnt) = (regs[0], regs[1] & self.count_mask);
        let mut error = false;
        let mut amount = 0;
        let acknowledged = device.and_then(|device| device.start(read).then_some(device));
        match acknowledged {
            Some(device) => {
                self.addressed = Some(address);
                if read {
                    let bytes: Vec<u8> = (0..maxcnt).map(|_| device.read()).collect();
                    mem.write(ptr, &bytes);
                    amount = maxcnt;
                } else {
                    let mut bytes = vec![0; maxcnt as usize];
                    mem.read(ptr, &mut bytes);
                    for byte in bytes {
                        amount += 1;
                        if !device.write(byte) {
                            self.errorsrc |= DNACK;
                            error = true;
                            break;
                        }
                    }
                }
            }
            None => {
                self.addressed = None;
                self.errorsrc |= ANACK;
                error = true;
            }
        }
        regs[2] = amount;
        let bits = 9 * u64::from(1 + amount);
        self.phase = Phase::Transfer {
            read,
            end: now + bits * self.bit_time(),
            error,
        };
    }

    /// Ends a phase: LASTTX or LASTRX and the shorts.
    fn finish(&mut self, read: bool, end: u64) {
        let (last, start, suspend, stop) = if read {
            (
                LASTRX,
                SHORT_LASTRX_STARTTX,
                SHORT_LASTRX_SUSPEND,
                SHORT_LASTRX_STOP,
            )
        } else {
            (
                LASTTX,
                SHORT_LASTTX_STARTRX,
                SHORT_LASTTX_SUSPEND,
                SHORT_LASTTX_STOP,
            )
        };
        // a phase without bytes has no last byte
        let amount = if read { self.rxd[2] } else { self.txd[2] };
        if amount > 0 {
            self.events.fire(last);
        }
        self.phase = Phase::Held;
        if self.events.short(start) {
            if read {
                self.events.fire(TXSTARTED);
            } else {
                self.events.fire(RXSTARTED);
            }
            self.phase = Phase::Starting { read: !read };
        } else if self.events.short(suspend) {
            self.events.fire(SUSPENDED);
        } else if self.events.short(stop) {
            self.phase = Phase::Stopping {
                end: end + self.bit_time(),
            };
        }
    }

    /// Advances the transfer to `now`; returns the time of its next step.
    pub fn update(
        &mut self,
        ctx: &Ctx,
        mem: &mut dyn Dma,
        buses: &mut BTreeMap<u32, I2cBus>,
    ) -> Option<u64> {
        loop {
            match self.phase {
                Phase::Starting { read } => self.transfer(read, ctx.now, mem, buses),
                Phase::Transfer { read, end, error } if end <= ctx.now => {
                    if error {
                        self.events.fire(ERROR);
                        self.phase = Phase::Held;
                    } else {
                        self.finish(read, end);
                    }
                }
                Phase::Stopping { end } if end <= ctx.now => {
                    if let Some(address) = self.addressed.take()
                        && let Some(device) = buses
                            .get_mut(&self.scl())
                            .and_then(|bus| bus.get_mut(&address))
                    {
                        device.stop();
                    }
                    self.events.fire(STOPPED);
                    self.phase = Phase::Idle;
                }
                Phase::Transfer { end, .. } | Phase::Stopping { end } => return Some(end),
                Phase::Idle | Phase::Held => return None,
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
            FREQUENCY => self.frequency,
            o if (RXD_PTR..=RXD_LIST).contains(&o) => self.rxd[((o - RXD_PTR) / 4) as usize],
            o if (TXD_PTR..=TXD_LIST).contains(&o) => self.txd[((o - TXD_PTR) / 4) as usize],
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
            ENABLE => {
                self.enable = value & 0xF;
                if !self.enabled() {
                    self.phase = Phase::Idle;
                    self.addressed = None;
                }
            }
            PSEL_SCL | PSEL_SDA => self.psel[((offset - PSEL_SCL) / 4) as usize] = value,
            FREQUENCY => self.frequency = value,
            o if (RXD_PTR..RXD_LIST).contains(&o) && o != RXD_PTR + 8 => {
                self.rxd[((o - RXD_PTR) / 4) as usize] = value;
            }
            o if (TXD_PTR..TXD_LIST).contains(&o) && o != TXD_PTR + 8 => {
                self.txd[((o - TXD_PTR) / 4) as usize] = value;
            }
            RXD_LIST => self.rxd[3] = value,
            TXD_LIST => self.txd[3] = value,
            ADDRESS => self.address = value & 0x7F,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::Ram;
    use super::*;

    /// A register file: the first byte written selects the register, then
    /// bytes are written or read from there.
    struct Regs {
        regs: [u8; 16],
        pointer: Option<usize>,
        stops: usize,
    }

    impl I2cDevice for Regs {
        fn start(&mut self, read: bool) -> bool {
            if !read {
                self.pointer = None;
            }
            true
        }
        fn write(&mut self, byte: u8) -> bool {
            match self.pointer {
                None => self.pointer = Some(byte as usize),
                Some(p) => {
                    self.regs[p] = byte;
                    self.pointer = Some(p + 1);
                }
            }
            true
        }
        fn read(&mut self) -> u8 {
            let p = self.pointer.unwrap_or(0);
            self.pointer = Some(p + 1);
            self.regs[p]
        }
        fn stop(&mut self) {
            self.stops += 1;
        }
    }

    fn setup() -> (Twim, Ram, BTreeMap<u32, I2cBus>) {
        let mut buses = BTreeMap::new();
        let mut bus = I2cBus::new();
        let mut regs = [0u8; 16];
        regs[3] = 0xA5;
        regs[4] = 0x5A;
        bus.insert(
            0x48,
            Box::new(Regs {
                regs,
                pointer: None,
                stops: 0,
            }) as Box<dyn I2cDevice>,
        );
        buses.insert(27, bus);
        let mut twim = Twim::new(0xFFFF);
        let ctx = Ctx { now: 0 };
        twim.write(ENABLE, ENABLED, &ctx);
        twim.write(PSEL_SCL, 27, &ctx);
        twim.write(FREQUENCY, 0x0640_0000, &ctx); // 400 kHz
        (twim, Ram::new(), buses)
    }

    #[test]
    fn write_then_read_with_the_shorts() {
        let (mut twim, mut ram, mut buses) = setup();
        let ctx = Ctx { now: 0 };
        ram.bytes[0] = 3;
        twim.write(ADDRESS, 0x48, &ctx);
        twim.write(TXD_PTR, 0x2000_0000, &ctx);
        twim.write(TXD_PTR + 4, 1, &ctx);
        twim.write(RXD_PTR, 0x2000_0010, &ctx);
        twim.write(RXD_PTR + 4, 2, &ctx);
        twim.write(
            0x200,
            (1 << SHORT_LASTTX_STARTRX) | (1 << SHORT_LASTRX_STOP),
            &ctx,
        );
        twim.write(0x008, 1, &ctx); // STARTTX
        let mut now = 0;
        while let Some(next) = twim.update(&Ctx { now }, &mut ram, &mut buses) {
            now = next;
        }
        assert!(twim.events.is_set(LASTTX) && twim.events.is_set(LASTRX));
        assert!(twim.events.is_set(STOPPED));
        assert!(!twim.events.is_set(ERROR));
        assert_eq!(&ram.bytes[0x10..0x12], &[0xA5, 0x5A]);
        assert_eq!(twim.read(RXD_PTR + 8), 2);
        // 2 + 3 bytes of 9 bits at 390 625 Hz, and a bit for the STOP
        assert_eq!(now, 46 * 2_560_000);
    }

    #[test]
    fn an_absent_device_is_not_acknowledged() {
        let (mut twim, mut ram, mut buses) = setup();
        let ctx = Ctx { now: 0 };
        twim.write(ADDRESS, 0x49, &ctx);
        twim.write(0x008, 1, &ctx);
        let next = twim.update(&ctx, &mut ram, &mut buses).unwrap();
        twim.update(&Ctx { now: next }, &mut ram, &mut buses);
        assert!(twim.events.is_set(ERROR));
        assert_eq!(twim.read(ERRORSRC), ANACK);
        twim.write(0x014, 1, &Ctx { now: next });
        let end = twim
            .update(&Ctx { now: next }, &mut ram, &mut buses)
            .unwrap();
        twim.update(&Ctx { now: end }, &mut ram, &mut buses);
        assert!(twim.events.is_set(STOPPED));
    }
}
