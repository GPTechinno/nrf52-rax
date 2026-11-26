//! RTC0-2: a 24-bit counter on LFCLK (32 768 Hz) divided by PRESCALER + 1,
//! with TICK, OVRFLW and COMPARE[n] events. An event reaches its EVENTS
//! register (and the PPI) only while it is enabled in INTEN or EVTEN. A
//! compare event comes when COUNTER becomes CC[n]: writing the current value
//! gives none until the counter comes round again.
//!
//! The counter runs from START to STOP; LFCLK is taken to be running.

use super::events::{self, Events};
use super::{Ctx, Device};

const TICK: u32 = 0;
const OVRFLW: u32 = 1;
const COMPARE: u32 = 16;

const EVTEN: u32 = 0x340;
const EVTENSET: u32 = 0x344;
const EVTENCLR: u32 = 0x348;
const COUNTER: u32 = 0x504;
const PRESCALER: u32 = 0x508;
const CC: u32 = 0x540;

const MASK: u32 = 0xFF_FFFF;
const LFCLK_HZ: u128 = 32_768;
const PS_PER_S: u128 = 1_000_000_000_000;

/// The LFCLK cycle running at `now` (picoseconds).
fn lf_cycle(now: u64) -> u64 {
    (u128::from(now) * LFCLK_HZ / PS_PER_S) as u64
}

/// When LFCLK cycle `cycle` starts.
fn lf_time(cycle: u64) -> u64 {
    (u128::from(cycle) * PS_PER_S).div_ceil(LFCLK_HZ) as u64
}

pub struct Rtc {
    pub events: Events,
    evten: u32,
    prescaler: u32,
    cc: Vec<u32>,
    /// While running: the LFCLK cycle of the last counter update, and the
    /// counter's total increments since it started.
    running: Option<u64>,
    /// The counter, at `running`'s cycle (or stopped).
    counter: u32,
    /// LFCLK cycles since the last increment.
    phase: u64,
}

impl Rtc {
    /// An RTC with `compares` compare registers (3 on RTC0, 4 on RTC1 and
    /// RTC2).
    pub fn new(compares: usize) -> Self {
        Self {
            events: Events::default(),
            evten: 0,
            prescaler: 0,
            cc: vec![0; compares],
            running: None,
            counter: 0,
            phase: 0,
        }
    }

    fn divider(&self) -> u64 {
        u64::from(self.prescaler) + 1
    }

    /// Brings the counter to `now`, generating the events it passes.
    fn advance(&mut self, now: u64) {
        let Some(last) = self.running else {
            return;
        };
        let cycle = lf_cycle(now);
        if cycle <= last {
            return;
        }
        let elapsed = cycle - last + self.phase;
        let increments = elapsed / self.divider();
        self.phase = elapsed % self.divider();
        self.running = Some(cycle);
        if increments == 0 {
            return;
        }
        let before = self.counter;
        let total = u64::from(before) + increments;
        self.counter = (total & u64::from(MASK)) as u32;
        let routed = self.events.inten | self.evten;
        let fire = |events: &mut Events, i: u32| {
            if routed & (1 << i) != 0 {
                events.fire(i);
            }
        };
        fire(&mut self.events, TICK);
        if total > u64::from(MASK) {
            fire(&mut self.events, OVRFLW);
        }
        for n in 0..self.cc.len() {
            // the counter took the values before + 1 ..= before + increments
            let distance = u64::from(self.cc[n].wrapping_sub(before).wrapping_sub(1) & MASK);
            if distance < increments {
                fire(&mut self.events, COMPARE + n as u32);
            }
        }
    }

    /// The time of the next event that is routed somewhere.
    fn next_event(&self) -> Option<u64> {
        let last = self.running?;
        let routed = self.events.inten | self.evten;
        let mut increments: Option<u64> = None;
        let mut consider = |n: u64| increments = Some(increments.map_or(n, |m| m.min(n)));
        if routed & (1 << TICK) != 0 {
            consider(1);
        }
        if routed & (1 << OVRFLW) != 0 {
            consider(u64::from(MASK - self.counter) + 1);
        }
        for n in 0..self.cc.len() {
            if routed & (1 << (COMPARE + n as u32)) != 0 {
                consider(
                    u64::from(self.cc[n].wrapping_sub(self.counter).wrapping_sub(1) & MASK) + 1,
                );
            }
        }
        let increments = increments?;
        let cycle = last + increments * self.divider() - self.phase;
        Some(lf_time(cycle))
    }

    fn task(&mut self, index: u32, now: u64) {
        match index {
            0 => {
                if self.running.is_none() {
                    self.running = Some(lf_cycle(now));
                    self.phase = 0;
                }
            }
            1 => self.running = None,
            2 => {
                self.counter = 0;
                self.phase = 0;
            }
            3 => self.counter = 0xFF_FFF0,
            _ => {}
        }
    }
}

impl Device for Rtc {
    fn read(&mut self, offset: u32, ctx: &mut Ctx) -> u32 {
        self.advance(ctx.now);
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            EVTEN | EVTENSET | EVTENCLR => self.evten,
            COUNTER => self.counter,
            PRESCALER => self.prescaler,
            o if (CC..CC + 4 * self.cc.len() as u32).contains(&o) => {
                self.cc[((o - CC) / 4) as usize]
            }
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx) {
        self.advance(ctx.now);
        if self.events.write(offset, value) {
            return;
        }
        if let Some(index) = events::task(offset, value) {
            self.task(index, ctx.now);
            return;
        }
        match offset {
            EVTEN => self.evten = value,
            EVTENSET => self.evten |= value,
            EVTENCLR => self.evten &= !value,
            // PRESCALER is written while the RTC is stopped
            PRESCALER if self.running.is_none() => self.prescaler = value & 0xFFF,
            o if (CC..CC + 4 * self.cc.len() as u32).contains(&o) => {
                self.cc[((o - CC) / 4) as usize] = value & MASK;
            }
            _ => {}
        }
    }

    fn update(&mut self, ctx: &mut Ctx, _: &mut dyn super::Dma) -> Option<u64> {
        self.advance(ctx.now);
        self.next_event()
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::Ram;
    use super::*;

    fn ctx(now: u64) -> Ctx {
        Ctx { now }
    }

    #[test]
    fn counts_lfclk_cycles_through_the_prescaler() {
        let mut rtc = Rtc::new(4);
        rtc.write(PRESCALER, 1, &mut ctx(0));
        rtc.write(0x000, 1, &mut ctx(0)); // START
        // 10 LFCLK cycles: 5 increments
        assert_eq!(rtc.read(COUNTER, &mut ctx(lf_time(10))), 5);
        assert_eq!(rtc.read(COUNTER, &mut ctx(lf_time(11))), 5);
        rtc.write(0x004, 1, &mut ctx(lf_time(11))); // STOP
        assert_eq!(rtc.read(COUNTER, &mut ctx(lf_time(100))), 5);
    }

    #[test]
    fn compare_needs_inten_or_evten_and_fires_when_the_counter_gets_there() {
        let mut rtc = Rtc::new(4);
        rtc.write(0x000, 1, &mut ctx(0));
        rtc.write(CC, 3, &mut ctx(0));
        assert_eq!(
            rtc.update(&mut ctx(0), &mut Ram::new()),
            None,
            "nothing routed"
        );
        rtc.write(0x304, 1 << 16, &mut ctx(0)); // INTENSET COMPARE0
        assert_eq!(rtc.update(&mut ctx(0), &mut Ram::new()), Some(lf_time(3)));
        rtc.update(&mut ctx(lf_time(3)), &mut Ram::new());
        assert!(rtc.events.is_set(16));
        assert!(rtc.events.irq());
        // CC written with the current value: no event before a full turn
        rtc.write(0x140, 0, &mut ctx(lf_time(3)));
        rtc.write(CC + 4, 3, &mut ctx(lf_time(3)));
        rtc.write(EVTENSET, 1 << 17, &mut ctx(lf_time(3)));
        assert_eq!(
            rtc.update(&mut ctx(lf_time(3)), &mut Ram::new()),
            Some(lf_time(3 + (1 << 24)))
        );
    }

    #[test]
    fn overflow_wraps_to_zero() {
        let mut rtc = Rtc::new(3);
        rtc.write(0x000, 1, &mut ctx(0));
        rtc.write(0x00C, 1, &mut ctx(0)); // TRIGOVRFLW
        rtc.write(0x304, 1 << OVRFLW, &mut ctx(0));
        assert_eq!(rtc.update(&mut ctx(0), &mut Ram::new()), Some(lf_time(16)));
        rtc.update(&mut ctx(lf_time(17)), &mut Ram::new());
        assert!(rtc.events.is_set(OVRFLW));
        assert_eq!(rtc.read(COUNTER, &mut ctx(lf_time(17))), 1);
    }
}
