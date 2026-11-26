//! TIMER0-4: a counter of 8, 16, 24 or 32 bits (BITMODE), counting the
//! 16 MHz clock divided by 2^PRESCALER in timer mode, or COUNT tasks in the
//! counter modes. COMPARE[n] comes when the counter becomes CC[n]; SHORTS
//! bit n clears and bit 8 + n stops the timer on it. CAPTURE[n] copies the
//! counter into CC[n].

use super::events::{self, Events};
use super::{Ctx, Device};

const COMPARE: u32 = 16;

const MODE: u32 = 0x504;
const BITMODE: u32 = 0x508;
const PRESCALER: u32 = 0x510;
const CC: u32 = 0x540;

/// One 16 MHz cycle, picoseconds.
const PS_16MHZ: u64 = 62_500;

pub struct Timer {
    pub events: Events,
    mode: u32,
    bitmode: u32,
    prescaler: u32,
    cc: Vec<u32>,
    counter: u32,
    /// While started: when `counter` was last brought up to date, at a tick
    /// boundary.
    started: Option<u64>,
}

impl Timer {
    /// A timer with `compares` compare registers (4 on TIMER0-2, 6 on
    /// TIMER3 and TIMER4).
    pub fn new(compares: usize) -> Self {
        Self {
            events: Events::default(),
            mode: 0,
            bitmode: 0,
            prescaler: 4,
            cc: vec![0; compares],
            counter: 0,
            started: None,
        }
    }

    fn mask(&self) -> u32 {
        match self.bitmode & 3 {
            0 => 0xFFFF,
            1 => 0xFF,
            2 => 0xFF_FFFF,
            _ => u32::MAX,
        }
    }

    fn period(&self) -> u64 {
        PS_16MHZ << self.prescaler.min(9)
    }

    fn timer_mode(&self) -> bool {
        self.mode & 3 == 0
    }

    /// One increment: the compare events and their shorts. Returns whether
    /// the timer stopped.
    fn increment(&mut self) -> bool {
        self.counter = self.counter.wrapping_add(1) & self.mask();
        let mut stop = false;
        let mut clear = false;
        for n in 0..self.cc.len() {
            if self.counter == self.cc[n] & self.mask() {
                self.events.fire(COMPARE + n as u32);
                clear |= self.events.short(n as u32);
                stop |= self.events.short(8 + n as u32);
            }
        }
        if clear {
            self.counter = 0;
        }
        stop
    }

    /// Increments until the next compare, or `limit`; how many.
    fn ticks_to_compare(&self) -> u64 {
        let mask = u64::from(self.mask());
        self.cc
            .iter()
            .map(|&cc| {
                let distance =
                    (u64::from(cc & self.mask()).wrapping_sub(u64::from(self.counter))) & mask;
                if distance == 0 { mask + 1 } else { distance }
            })
            .min()
            .unwrap_or(mask + 1)
    }

    /// Brings the counter to `now` in timer mode.
    fn advance(&mut self, now: u64) {
        let Some(mut since) = self.started else {
            return;
        };
        if !self.timer_mode() {
            return;
        }
        let period = self.period();
        loop {
            let ticks = (now.saturating_sub(since)) / period;
            if ticks == 0 {
                break;
            }
            let to_compare = self.ticks_to_compare();
            if ticks < to_compare {
                let mask = u64::from(self.mask());
                self.counter = ((u64::from(self.counter) + ticks) & mask) as u32;
                since += ticks * period;
                break;
            }
            // up to the compare
            let mask = u64::from(self.mask());
            self.counter = ((u64::from(self.counter) + to_compare - 1) & mask) as u32;
            since += to_compare * period;
            if self.increment() {
                self.started = None;
                return;
            }
        }
        self.started = Some(since);
    }

    fn task(&mut self, index: u32, now: u64) {
        match index {
            0 => {
                if self.started.is_none() {
                    self.started = Some(now);
                }
            }
            1 | 4 => self.started = None,
            2 => {
                if self.started.is_some() && !self.timer_mode() && self.increment() {
                    self.started = None;
                }
            }
            3 => self.counter = 0,
            i if (COMPARE..COMPARE + self.cc.len() as u32).contains(&i) => {
                // CAPTURE[n]
                self.cc[(i - COMPARE) as usize] = self.counter;
            }
            _ => {}
        }
    }
}

impl Device for Timer {
    fn read(&mut self, offset: u32, ctx: &mut Ctx) -> u32 {
        self.advance(ctx.now);
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            MODE => self.mode,
            BITMODE => self.bitmode,
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
            MODE => self.mode = value & 3,
            BITMODE => self.bitmode = value & 3,
            PRESCALER => self.prescaler = value & 0xF,
            o if (CC..CC + 4 * self.cc.len() as u32).contains(&o) => {
                self.cc[((o - CC) / 4) as usize] = value;
            }
            _ => {}
        }
    }

    fn update(&mut self, ctx: &mut Ctx, _: &mut dyn super::Dma) -> Option<u64> {
        self.advance(ctx.now);
        let since = self.started?;
        if !self.timer_mode() {
            return None;
        }
        Some(since + self.ticks_to_compare() * self.period())
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
    fn compare_clears_and_stops_through_the_shorts() {
        let mut timer = Timer::new(4);
        timer.write(PRESCALER, 0, &mut ctx(0));
        timer.write(BITMODE, 3, &mut ctx(0));
        timer.write(CC, 10, &mut ctx(0));
        timer.write(0x200, 1, &mut ctx(0)); // COMPARE0_CLEAR
        timer.write(0x000, 1, &mut ctx(0)); // START
        assert_eq!(
            timer.update(&mut ctx(0), &mut Ram::new()),
            Some(10 * PS_16MHZ)
        );
        timer.update(&mut ctx(25 * PS_16MHZ), &mut Ram::new());
        assert!(timer.events.is_set(16));
        timer.write(0x040, 1, &mut ctx(25 * PS_16MHZ)); // CAPTURE0
        assert_eq!(
            timer.read(CC, &mut ctx(25 * PS_16MHZ)),
            5,
            "cleared at 10 and 20"
        );
        timer.write(0x200, 1 << 8, &mut ctx(25 * PS_16MHZ)); // COMPARE0_STOP
        timer.write(CC, 7, &mut ctx(25 * PS_16MHZ));
        timer.update(&mut ctx(100 * PS_16MHZ), &mut Ram::new());
        timer.write(0x044, 1, &mut ctx(100 * PS_16MHZ)); // CAPTURE1
        assert_eq!(
            timer.read(CC + 4, &mut ctx(100 * PS_16MHZ)),
            7,
            "stopped at 7"
        );
    }

    #[test]
    fn counter_mode_counts_count_tasks() {
        let mut timer = Timer::new(4);
        timer.write(MODE, 2, &mut ctx(0));
        timer.write(CC + 8, 2, &mut ctx(0));
        timer.write(0x000, 1, &mut ctx(0));
        timer.write(0x008, 1, &mut ctx(0));
        timer.write(0x008, 1, &mut ctx(1_000_000_000));
        assert!(timer.events.is_set(18));
        timer.write(0x040, 1, &mut ctx(0));
        assert_eq!(timer.read(CC, &mut ctx(0)), 2);
    }
}
