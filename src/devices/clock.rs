//! CLOCK and POWER, which share their address (0x4000_0000), their
//! interrupt and their event and interrupt-enable registers.
//!
//! - HFCLK: the CPU runs from HFINT; HFCLKSTART starts the HFXO, ready after
//!   its power-up (60 µs) and HFXODEBOUNCE × 16 µs.
//! - LFCLK: LFCLKSTART takes the source in LFCLKSRC (copied to
//!   LFCLKSRCCOPY), running after its start-up: 1 ms for the RC oscillator,
//!   0.25 s for the crystal, at once for the synthesised clock.
//! - POWER: RESETREAS (write 1 to clear), GPREGRET and GPREGRET2 (kept by a
//!   reset), SYSTEMOFF, and the other registers kept as written.
//!
//! Not modelled: calibration (DONE and CTTO never come), the power-fail
//! comparator, USB supply detection, the RAM power switches.

use super::events::{self, Events};
use super::regfile::RegFile;
use super::{Ctx, Device};

const HFCLKSTARTED: u32 = 0;
const LFCLKSTARTED: u32 = 1;

const HFCLKRUN: u32 = 0x408;
const HFCLKSTAT: u32 = 0x40C;
const LFCLKRUN: u32 = 0x414;
const LFCLKSTAT: u32 = 0x418;
const LFCLKSRCCOPY: u32 = 0x41C;
const LFCLKSRC: u32 = 0x518;
const HFXODEBOUNCE: u32 = 0x528;
const RESETREAS: u32 = 0x400;
const SYSTEMOFF: u32 = 0x500;
const GPREGRET: u32 = 0x51C;
const GPREGRET2: u32 = 0x520;

/// xxCLKSTAT.STATE: running.
const RUNNING: u32 = 1 << 16;

const PS_PER_US: u64 = 1_000_000;

/// RESETREAS bits.
pub const RESET_PIN: u32 = 1 << 0;
pub const RESET_DOG: u32 = 1 << 1;
pub const RESET_SREQ: u32 = 1 << 2;
pub const RESET_LOCKUP: u32 = 1 << 3;

pub struct ClockPower {
    pub events: Events,
    regs: RegFile,
    hfxo: Oscillator,
    lfclk: Oscillator,
    /// LFCLKSRC when LFCLK was started.
    lfclk_source: u32,
    pub resetreas: u32,
    gpregret: [u32; 2],
    /// SYSTEMOFF was written: the chip is off.
    pub system_off: bool,
}

/// An oscillator: stopped, starting until a time, or running.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Oscillator {
    Stopped,
    Starting(u64),
    Running,
}

impl ClockPower {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            regs: RegFile::with(&[(HFXODEBOUNCE, 0x10)]),
            hfxo: Oscillator::Stopped,
            lfclk: Oscillator::Stopped,
            lfclk_source: 0,
            resetreas: 0,
            gpregret: [0; 2],
            system_off: false,
        }
    }

    /// The state after a reset that keeps the retained registers
    /// (GPREGRET, GPREGRET2) and adds `reason` to RESETREAS.
    pub fn after_reset(&self, reason: u32) -> Self {
        Self {
            resetreas: self.resetreas | reason,
            gpregret: self.gpregret,
            ..Self::new()
        }
    }

    /// Whether LFCLK runs: the RTCs count only then.
    pub fn lfclk_running(&self) -> bool {
        self.lfclk == Oscillator::Running
    }

    fn start(&self, now: u64, delay_us: u64) -> Oscillator {
        if delay_us == 0 {
            Oscillator::Running
        } else {
            Oscillator::Starting(now + delay_us * PS_PER_US)
        }
    }

    fn task(&mut self, index: u32, now: u64) {
        match index {
            0 => {
                // HFCLKSTART
                let debounce = u64::from(self.regs.get(HFXODEBOUNCE) & 0xFF);
                self.hfxo = self.start(now, 60 + 16 * debounce);
                self.regs.set(HFCLKRUN, 1);
            }
            1 => {
                self.hfxo = Oscillator::Stopped;
                self.regs.set(HFCLKRUN, 0);
            }
            2 => {
                // LFCLKSTART
                self.lfclk_source = self.regs.get(LFCLKSRC);
                let delay = match self.lfclk_source & 3 {
                    0 => 1000,
                    1 => 250_000,
                    _ => 0,
                };
                self.lfclk = self.start(now, delay);
                self.regs.set(LFCLKRUN, 1);
                if self.lfclk == Oscillator::Running {
                    self.events.fire(LFCLKSTARTED);
                }
            }
            3 => {
                self.lfclk = Oscillator::Stopped;
                self.regs.set(LFCLKRUN, 0);
            }
            _ => {}
        }
    }
}

impl Default for ClockPower {
    fn default() -> Self {
        Self::new()
    }
}

impl Device for ClockPower {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            HFCLKSTAT => {
                // the CPU's clock runs from HFINT (SRC 0) or the HFXO
                RUNNING | u32::from(self.hfxo == Oscillator::Running)
            }
            LFCLKSTAT => {
                let state = if self.lfclk_running() { RUNNING } else { 0 };
                state | (self.lfclk_source & 3)
            }
            LFCLKSRCCOPY => self.lfclk_source,
            RESETREAS => self.resetreas,
            GPREGRET => self.gpregret[0],
            GPREGRET2 => self.gpregret[1],
            o => self.regs.get(o),
        }
    }

    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        if let Some(index) = events::task(offset, value) {
            self.task(index, ctx.now);
            return;
        }
        match offset {
            RESETREAS => self.resetreas &= !value,
            SYSTEMOFF if value & 1 != 0 => self.system_off = true,
            GPREGRET => self.gpregret[0] = value & 0xFF,
            GPREGRET2 => self.gpregret[1] = value & 0xFF,
            HFCLKSTAT | LFCLKSTAT | LFCLKSRCCOPY | HFCLKRUN | LFCLKRUN => {}
            o => self.regs.set(o, value),
        }
    }

    fn update(&mut self, ctx: &mut Ctx, _: &mut dyn super::Dma) -> Option<u64> {
        let mut next = None;
        if let Oscillator::Starting(at) = self.hfxo {
            if at <= ctx.now {
                self.hfxo = Oscillator::Running;
                self.events.fire(HFCLKSTARTED);
            } else {
                next = Some(at);
            }
        }
        if let Oscillator::Starting(at) = self.lfclk {
            if at <= ctx.now {
                self.lfclk = Oscillator::Running;
                self.events.fire(LFCLKSTARTED);
            } else {
                next = Some(next.map_or(at, |n: u64| n.min(at)));
            }
        }
        next
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}
