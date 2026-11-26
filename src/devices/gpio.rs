//! GPIO ports (P0, and P1 on the nRF52833 and nRF52840) and GPIOTE.
//!
//! A pin's level is its output's when it is an output (DIR, or a GPIOTE
//! channel in task mode driving it), else what the board drives on it, else
//! its pull resistor's (none reads low). Its input buffer must be connected
//! (PIN_CNF.INPUT) for IN to show it. PIN_CNF.SENSE sets LATCH bits and
//! makes the DETECT signal (from LATCH in DETECTMODE's latched mode), whose
//! rising edge is GPIOTE's PORT event.
//!
//! GPIOTE: eight channels, each disabled, watching its pin for edges (event
//! mode: EVENTS_IN by POLARITY) or driving it (task mode: OUTINIT, then the
//! OUT, SET and CLR tasks).

use super::events::{self, Events};
use super::{Ctx, Device};

const OUT: u32 = 0x504;
const OUTSET: u32 = 0x508;
const OUTCLR: u32 = 0x50C;
const IN: u32 = 0x510;
const DIR: u32 = 0x514;
const DIRSET: u32 = 0x518;
const DIRCLR: u32 = 0x51C;
const LATCH: u32 = 0x520;
const DETECTMODE: u32 = 0x524;
const PIN_CNF: u32 = 0x700;

/// One GPIO port.
pub struct Gpio {
    /// Mask of the pins the port has.
    pins: u32,
    pub out: u32,
    pub dir: u32,
    pub cnf: [u32; 32],
    pub latch: u32,
    pub detectmode: u32,
    /// The levels the board drives, on the pins in `driven`.
    pub input: u32,
    pub driven: u32,
    /// Pins a GPIOTE channel drives (task mode), and their levels.
    pub gpiote_driven: u32,
    pub gpiote_out: u32,
}

impl Gpio {
    /// A port of `count` pins at reset: inputs, disconnected, no pull.
    pub fn new(count: u32) -> Self {
        Self {
            pins: if count >= 32 {
                u32::MAX
            } else {
                (1 << count) - 1
            },
            out: 0,
            dir: 0,
            cnf: [0x0000_0002; 32],
            latch: 0,
            detectmode: 0,
            input: 0,
            driven: 0,
            gpiote_driven: 0,
            gpiote_out: 0,
        }
    }

    /// The pins that are outputs: DIR, or driven by GPIOTE.
    pub fn outputs(&self) -> u32 {
        (self.dir | self.gpiote_driven) & self.pins
    }

    /// The level of every pin.
    pub fn levels(&self) -> u32 {
        let mut pulled = 0;
        for pin in 0..32 {
            if (self.cnf[pin as usize] >> 2) & 3 == 3 {
                pulled |= 1 << pin;
            }
        }
        let output = (self.out & !self.gpiote_driven) | (self.gpiote_out & self.gpiote_driven);
        let outputs = self.outputs();
        let inputs = (self.input & self.driven) | (pulled & !self.driven);
        ((output & outputs) | (inputs & !outputs)) & self.pins
    }

    /// The level of `pin`.
    pub fn level(&self, pin: u32) -> bool {
        self.levels() & (1 << pin) != 0
    }

    /// The pins whose SENSE condition holds.
    fn sensing(&self) -> u32 {
        let levels = self.levels();
        let mut matched = 0;
        for pin in 0..32u32 {
            let sense = (self.cnf[pin as usize] >> 16) & 3;
            let high = levels & (1 << pin) != 0;
            if (sense == 2 && high) || (sense == 3 && !high) {
                matched |= 1 << pin;
            }
        }
        matched & self.pins
    }

    /// Updates LATCH; returns this port's DETECT signal.
    pub fn detect(&mut self) -> bool {
        let matched = self.sensing();
        self.latch |= matched;
        if self.detectmode & 1 != 0 {
            self.latch != 0
        } else {
            matched != 0
        }
    }

    pub fn read(&self, offset: u32) -> u32 {
        match offset {
            OUT | OUTSET | OUTCLR => self.out,
            IN => {
                let connected = (0..32).fold(0u32, |mask, pin| {
                    mask | (u32::from(self.cnf[pin] & 2 == 0) << pin)
                });
                self.levels() & connected
            }
            DIR | DIRSET | DIRCLR => self.dir,
            LATCH => self.latch,
            DETECTMODE => self.detectmode,
            o if (PIN_CNF..PIN_CNF + 0x80).contains(&o) => {
                let pin = ((o - PIN_CNF) / 4) as usize;
                (self.cnf[pin] & !1) | ((self.dir >> pin) & 1)
            }
            _ => 0,
        }
    }

    /// A write; whether the offset is a register of the port.
    pub fn write(&mut self, offset: u32, value: u32) -> bool {
        let value_pins = value & self.pins;
        match offset {
            OUT => self.out = value_pins,
            OUTSET => self.out |= value_pins,
            OUTCLR => self.out &= !value_pins,
            DIR => self.dir = value_pins,
            DIRSET => self.dir |= value_pins,
            DIRCLR => self.dir &= !value_pins,
            // write 1 to clear
            LATCH => self.latch &= !value,
            DETECTMODE => self.detectmode = value & 1,
            o if (PIN_CNF..PIN_CNF + 0x80).contains(&o) => {
                let pin = (o - PIN_CNF) / 4;
                self.cnf[pin as usize] = value & 0x0003_070F;
                // PIN_CNF.DIR is DIR's bit
                if value & 1 != 0 {
                    self.dir |= (1 << pin) & self.pins;
                } else {
                    self.dir &= !(1 << pin);
                }
            }
            IN => {}
            _ => return false,
        }
        true
    }
}

const CONFIG: u32 = 0x510;
/// EVENTS_PORT: event 31.
pub const PORT: u32 = 31;

/// GPIOTE.
pub struct Gpiote {
    pub events: Events,
    pub config: [u32; 8],
    /// The output level of each channel in task mode.
    pub level: [bool; 8],
    /// A task or a CONFIG write changed the pins GPIOTE drives.
    pub changed: bool,
}

impl Gpiote {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            config: [0; 8],
            level: [false; 8],
            changed: false,
        }
    }

    /// Channel `n`'s mode (0 disabled, 1 event, 3 task), port and pin, and
    /// polarity.
    pub fn channel(&self, n: usize) -> (u32, usize, u32, u32) {
        let config = self.config[n];
        (
            config & 3,
            ((config >> 13) & 1) as usize,
            (config >> 8) & 0x1F,
            (config >> 16) & 3,
        )
    }

    /// Watches the pins of both ports go from `before` to `after`: the
    /// event-mode channels' edges.
    pub fn observe(&mut self, before: [u32; 2], after: [u32; 2]) {
        for n in 0..8 {
            let (mode, port, pin, polarity) = self.channel(n);
            if mode != 1 {
                continue;
            }
            let (was, is) = (before[port] >> pin & 1, after[port] >> pin & 1);
            let edge = match polarity {
                1 => was == 0 && is == 1,
                2 => was == 1 && is == 0,
                3 => was != is,
                _ => false,
            };
            if edge {
                self.events.fire(n as u32);
            }
        }
    }

    /// The pins of each port the task-mode channels drive, and their levels.
    pub fn driven(&self) -> [(u32, u32); 2] {
        let mut driven = [(0, 0); 2];
        for n in 0..8 {
            let (mode, port, pin, _) = self.channel(n);
            if mode == 3 {
                driven[port].0 |= 1 << pin;
                if self.level[n] {
                    driven[port].1 |= 1 << pin;
                }
            }
        }
        driven
    }
}

impl Default for Gpiote {
    fn default() -> Self {
        Self::new()
    }
}

impl Device for Gpiote {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            o if (CONFIG..CONFIG + 32).contains(&o) => self.config[((o - CONFIG) / 4) as usize],
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, _: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        if let Some(index) = events::task(offset, value) {
            // TASKS_OUT[n] (0..8), TASKS_SET[n] (12..20), TASKS_CLR[n] (24..32)
            let (n, kind) = ((index % 12) as usize, index / 12);
            if n < 8 && self.config[n] & 3 == 3 {
                let polarity = (self.config[n] >> 16) & 3;
                self.level[n] = match (kind, polarity) {
                    (0, 1) => true,
                    (0, 2) => false,
                    (0, 3) => !self.level[n],
                    (1, _) => true,
                    (2, _) => false,
                    (_, _) => self.level[n],
                };
                self.changed = true;
            }
            return;
        }
        if let o @ CONFIG..=0x52C = offset {
            let n = ((o - CONFIG) / 4) as usize;
            self.config[n] = value & 0x0013_3F03;
            if value & 3 == 3 {
                self.level[n] = value & (1 << 20) != 0;
            }
            self.changed = true;
        }
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_follow_outputs_then_the_board_then_the_pulls() {
        let mut p0 = Gpio::new(32);
        p0.write(PIN_CNF + 4 * 3, 0x0000_000C); // pin 3: input, connected, pull-up
        p0.write(PIN_CNF + 4 * 4, 0x0000_0001); // pin 4: output, input connected
        p0.write(OUTSET, 1 << 4);
        assert_eq!(p0.read(IN) & 0x18, 0x18);
        p0.driven = 1 << 3;
        p0.input = 0;
        assert_eq!(p0.read(IN) & 0x18, 0x10, "the board pulls pin 3 low");
        p0.write(OUTCLR, 1 << 4);
        assert!(!p0.level(4));
        // pin 5 disconnected: IN shows 0 whatever the board drives
        p0.driven |= 1 << 5;
        p0.input = 1 << 5;
        assert!(p0.level(5));
        assert_eq!(p0.read(IN) & (1 << 5), 0);
    }

    #[test]
    fn sense_latches_and_detects() {
        let mut p0 = Gpio::new(32);
        p0.write(PIN_CNF + 4 * 7, 0x0003_000C); // pin 7: pull-up, sense low
        assert!(!p0.detect());
        p0.driven = 1 << 7;
        assert!(p0.detect());
        assert_eq!(p0.latch, 1 << 7);
        p0.driven = 0;
        assert!(!p0.detect(), "DETECT follows the pin by default");
        p0.write(DETECTMODE, 1);
        assert!(p0.detect(), "latched: DETECT stays until LATCH is cleared");
        p0.write(LATCH, 1 << 7);
        assert!(!p0.detect());
    }

    #[test]
    fn gpiote_channels_watch_and_drive_their_pins() {
        let mut gpiote = Gpiote::new();
        let mut ctx = Ctx { now: 0 };
        // channel 0: event on P0.13 going low; channel 1: task on P1.02,
        // toggling, starting high
        gpiote.write(CONFIG, 1 | (13 << 8) | (2 << 16), &mut ctx);
        gpiote.write(
            CONFIG + 4,
            3 | (2 << 8) | (1 << 13) | (3 << 16) | (1 << 20),
            &mut ctx,
        );
        gpiote.observe([1 << 13, 0], [0, 0]);
        assert!(gpiote.events.is_set(0));
        assert_eq!(gpiote.driven(), [(0, 0), (1 << 2, 1 << 2)]);
        gpiote.write(0x004, 1, &mut ctx); // TASKS_OUT[1]
        assert_eq!(gpiote.driven()[1], (1 << 2, 0));
        gpiote.write(0x034, 1, &mut ctx); // TASKS_SET[1]
        assert_eq!(gpiote.driven()[1], (1 << 2, 1 << 2));
    }
}
