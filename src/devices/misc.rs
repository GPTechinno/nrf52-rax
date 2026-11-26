//! The small peripherals: EGU0-5, RNG, TEMP and WDT.

use super::events::{self, Events};
use super::{Ctx, Device};

const PS_PER_US: u64 = 1_000_000;

/// EGU0-5: TASKS_TRIGGER[n] gives EVENTS_TRIGGERED[n], 16 of each; the
/// interrupt is shared with SWI0-5.
#[derive(Default)]
pub struct Egu {
    pub events: Events,
}

impl Device for Egu {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        self.events.read(offset).unwrap_or(0)
    }

    fn write(&mut self, offset: u32, value: u32, _: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        if let Some(n) = events::task(offset, value).filter(|&n| n < 16) {
            self.events.fire(n);
        }
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

/// RNG: START gives a random byte in VALUE with VALRDY every 30 µs (120 µs
/// with the bias correction of CONFIG.DERCEN), until STOP or the
/// VALRDY_STOP short. The bytes come from a xorshift generator seeded by
/// the machine.
pub struct Rng {
    pub events: Events,
    config: u32,
    value: u32,
    state: u64,
    next: Option<u64>,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            events: Events::default(),
            config: 0,
            value: 0,
            state: seed | 1,
            next: None,
        }
    }

    /// The generator's state, to seed the next one after a reset.
    pub fn byte_seed(&self) -> u64 {
        self.state
    }

    fn interval(&self) -> u64 {
        if self.config & 1 != 0 { 120 } else { 30 }
    }

    fn byte(&mut self) -> u8 {
        // xorshift64*
        self.state ^= self.state >> 12;
        self.state ^= self.state << 25;
        self.state ^= self.state >> 27;
        (self.state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 56) as u8
    }
}

impl Device for Rng {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            0x504 => self.config,
            0x508 => self.value,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        match events::task(offset, value) {
            Some(0) if self.next.is_none() => {
                self.next = Some(ctx.now + self.interval() * PS_PER_US);
            }
            Some(1) => self.next = None,
            Some(_) => {}
            None if offset == 0x504 => self.config = value & 1,
            None => {}
        }
    }

    fn update(&mut self, ctx: &mut Ctx, _: &mut dyn super::Dma) -> Option<u64> {
        while let Some(at) = self.next.filter(|&at| at <= ctx.now) {
            self.value = u32::from(self.byte());
            self.events.fire(0);
            self.next = if self.events.short(0) {
                None
            } else {
                Some(at + self.interval() * PS_PER_US)
            };
        }
        self.next
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

/// TEMP: START gives the die temperature in TEMP (0.25 °C steps) with
/// DATARDY 36 µs later. The board sets the temperature. The calibration
/// registers (A0-A5, B0-B5, T0-T4) keep what is written.
pub struct Temp {
    pub events: Events,
    /// The temperature, quarter degrees Celsius.
    pub quarter_degrees: i32,
    temp: i32,
    calibration: [u32; 17],
    ready: Option<u64>,
}

impl Temp {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            quarter_degrees: 25 * 4,
            temp: 0,
            calibration: [0; 17],
            ready: None,
        }
    }
}

impl Default for Temp {
    fn default() -> Self {
        Self::new()
    }
}

impl Device for Temp {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            0x508 => self.temp as u32,
            o if (0x520..0x570).contains(&o) => calibration_index(o)
                .map(|i| self.calibration[i])
                .unwrap_or(0),
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        match events::task(offset, value) {
            Some(0) => self.ready = Some(ctx.now + 36 * PS_PER_US),
            Some(1) => self.ready = None,
            Some(_) => {}
            None => {
                if let Some(i) = calibration_index(offset) {
                    self.calibration[i] = value;
                }
            }
        }
    }

    fn update(&mut self, ctx: &mut Ctx, _: &mut dyn super::Dma) -> Option<u64> {
        if let Some(at) = self.ready
            && at <= ctx.now
        {
            self.temp = self.quarter_degrees;
            self.events.fire(0);
            self.ready = None;
        }
        self.ready
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

/// A0-A5 (0x520), B0-B5 (0x540), T0-T4 (0x560).
fn calibration_index(offset: u32) -> Option<usize> {
    match offset {
        0x520..=0x534 => Some(((offset - 0x520) / 4) as usize),
        0x540..=0x554 => Some(6 + ((offset - 0x540) / 4) as usize),
        0x560..=0x570 => Some(12 + ((offset - 0x560) / 4) as usize),
        _ => None,
    }
}

/// WDT: START runs a down-counter of CRV + 1 LFCLK cycles, reloaded when
/// every reload request enabled in RREN has written 0x6E524635. At zero it
/// gives TIMEOUT, then resets the chip two cycles later. Once started it
/// runs until a reset; CONFIG's sleep and halt behaviour is not modelled
/// (the watchdog always runs).
pub struct Wdt {
    pub events: Events,
    crv: u32,
    rren: u32,
    config: u32,
    /// The reload requests written since the last reload.
    requested: u32,
    /// When the counter reaches zero, while running.
    timeout: Option<u64>,
    /// When the chip resets, after the TIMEOUT event.
    pub reset_at: Option<u64>,
}

const RELOAD: u32 = 0x6E52_4635;
const LFCLK_HZ: u128 = 32_768;

impl Wdt {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            crv: u32::MAX,
            rren: 1,
            config: 1,
            requested: 0,
            timeout: None,
            reset_at: None,
        }
    }

    fn period(&self, cycles: u64) -> u64 {
        (u128::from(cycles) * 1_000_000_000_000 / LFCLK_HZ) as u64
    }

    pub fn running(&self) -> bool {
        self.timeout.is_some() || self.reset_at.is_some()
    }
}

impl Default for Wdt {
    fn default() -> Self {
        Self::new()
    }
}

impl Device for Wdt {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            0x400 => u32::from(self.running()),
            0x404 => self.rren & !self.requested,
            0x504 => self.crv,
            0x508 => self.rren,
            0x50C => self.config,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        match offset {
            0x000 if value & 1 != 0 && !self.running() => {
                self.timeout = Some(ctx.now + self.period(u64::from(self.crv) + 1));
            }
            // CRV, RREN and CONFIG are written before START
            0x504 if !self.running() => self.crv = value.max(0xF),
            0x508 if !self.running() => self.rren = value,
            0x50C if !self.running() => self.config = value,
            o if (0x600..0x620).contains(&o) && value == RELOAD => {
                let n = (o - 0x600) / 4;
                if self.rren & (1 << n) != 0 {
                    self.requested |= 1 << n;
                }
                if self.timeout.is_some() && self.requested & self.rren == self.rren {
                    self.requested = 0;
                    self.timeout = Some(ctx.now + self.period(u64::from(self.crv) + 1));
                }
            }
            _ => {}
        }
    }

    fn update(&mut self, ctx: &mut Ctx, _: &mut dyn super::Dma) -> Option<u64> {
        if let Some(at) = self.timeout.filter(|&at| at <= ctx.now) {
            self.events.fire(0);
            self.timeout = None;
            self.reset_at = Some(at + self.period(2));
        }
        self.timeout.or(self.reset_at)
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::Ram;
    use super::*;

    #[test]
    fn rng_gives_bytes_until_the_short_stops_it() {
        let mut rng = Rng::new(7);
        let mut ram = Ram::new();
        let mut ctx = Ctx { now: 0 };
        rng.write(0x200, 1, &mut ctx); // VALRDY_STOP
        rng.write(0x000, 1, &mut ctx);
        assert_eq!(rng.update(&mut ctx, &mut ram), Some(30 * PS_PER_US));
        let mut ctx = Ctx {
            now: 100 * PS_PER_US,
        };
        assert_eq!(rng.update(&mut ctx, &mut ram), None);
        assert!(rng.events.is_set(0));
    }

    #[test]
    fn watchdog_resets_unless_reloaded() {
        let mut wdt = Wdt::new();
        let mut ram = Ram::new();
        let mut ctx = Ctx { now: 0 };
        wdt.write(0x504, 32_767, &mut ctx); // 1 s
        wdt.write(0x000, 1, &mut ctx);
        let second = 1_000_000_000_000;
        let mut ctx = Ctx { now: second / 2 };
        wdt.write(0x600, RELOAD, &mut ctx);
        assert_eq!(wdt.update(&mut ctx, &mut ram), Some(second * 3 / 2));
        let mut ctx = Ctx {
            now: second * 3 / 2,
        };
        wdt.update(&mut ctx, &mut ram);
        assert!(wdt.events.is_set(0));
        assert!(wdt.reset_at.is_some());
    }
}
