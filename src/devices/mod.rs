//! The peripherals and their place in the memory map: peripheral ID `n` sits
//! at `0x4000_0000 + n × 0x1000` and raises interrupt `n`; the GPIO ports
//! are at 0x5000_0000 (P0) and 0x5000_0300 (P1). Events are routed through
//! the PPI to tasks, EasyDMA reads and writes the data RAM, and virtual time
//! drives the clocks.
//!
//! The peripherals not modelled are registers that keep what is written; the
//! first access to each is reported in [`Peripherals::log`], with the PC
//! that made it.

pub mod clock;
pub mod events;
pub mod gpio;
pub mod misc;
pub mod ppi;
pub mod regfile;
pub mod rtc;
pub mod timer;
pub mod twi;
pub mod twim;
pub mod uarte;

use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

use clock::ClockPower;
use events::{EVENTS, Events};
use gpio::{Gpio, Gpiote};
use misc::{Egu, Rng, Temp, Wdt};
use ppi::Ppi;
use regfile::{RegFile, sub_read, sub_write};
use rtc::Rtc;
use timer::Timer;
use twi::Twi;
use twim::{I2cBus, Twim};
use uarte::Uarte;

use crate::variant::Variant;

/// The virtual time in picoseconds and the PC of the instruction running,
/// shared by the machine with the peripherals; `kick` asks the machine to
/// service the peripherals after the instruction (a register was written).
#[derive(Default)]
pub struct Clock {
    pub now: AtomicU64,
    pub pc: AtomicU32,
    pub kick: AtomicBool,
}

impl Clock {
    pub fn now(&self) -> u64 {
        self.now.load(Ordering::Relaxed)
    }

    pub fn pc(&self) -> u32 {
        self.pc.load(Ordering::Relaxed)
    }
}

/// What a peripheral sees of the machine on an access.
pub struct Ctx {
    /// Virtual time, picoseconds.
    pub now: u64,
}

/// EasyDMA's view of memory: the data RAM.
pub trait Dma {
    /// Reads `buf.len()` bytes at `address`; `false` outside the data RAM.
    fn read(&mut self, address: u32, buf: &mut [u8]) -> bool;
    /// Writes `data` at `address`; `false` outside the data RAM.
    fn write(&mut self, address: u32, data: &[u8]) -> bool;
}

/// A peripheral with the Nordic register interface.
pub trait Device {
    /// A read of the 32-bit register at `offset`.
    fn read(&mut self, offset: u32, ctx: &mut Ctx) -> u32;
    /// A write of the 32-bit register at `offset`.
    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx);
    /// Advances to `ctx.now`, moving data with EasyDMA; returns the time of
    /// the next thing to happen.
    fn update(&mut self, ctx: &mut Ctx, mem: &mut dyn Dma) -> Option<u64> {
        let _ = (ctx, mem);
        None
    }
    fn events(&mut self) -> &mut Events;
}

/// Watches the pins: called with a port and the level of its pins after
/// they change.
pub trait PinWatcher: Send {
    fn changed(&mut self, port: usize, levels: u32, now: u64);
}

const PERIPHERALS: u32 = 0x4000_0000;
const PERIPHERALS_END: u32 = 0x4010_0000;
const P0: u32 = 0x5000_0000;
const P1: u32 = 0x5000_0300;
const GPIO_END: u32 = 0x5000_0B00;

pub const ID_CLOCK: u32 = 0;
pub const ID_UARTE0: u32 = 2;
pub const ID_SERIAL0: u32 = 3;
pub const ID_SERIAL1: u32 = 4;
pub const ID_GPIOTE: u32 = 6;
pub const ID_PPI: u32 = 31;
pub const ID_UARTE1: u32 = 40;

/// The peripherals modelled as [`Device`]s, by ID.
const DEVICES: [u32; 21] = [
    0,  // CLOCK, POWER
    2,  // UARTE0
    6,  // GPIOTE
    8,  // TIMER0
    9,  // TIMER1
    10, // TIMER2
    11, // RTC0
    12, // TEMP
    13, // RNG
    16, // WDT
    17, // RTC1
    20, // EGU0
    21, // EGU1
    22, // EGU2
    23, // EGU3
    24, // EGU4
    25, // EGU5
    26, // TIMER3
    27, // TIMER4
    36, // RTC2
    40, // UARTE1
];

/// Peripheral ID 3 or 4: TWIM, SPIM, SPIS, TWIS or the legacy TWI and SPI,
/// picked by ENABLE. The TWIM (6) and the legacy TWI (5) are modelled, the
/// TWIM holding the registers while the instance is disabled; the others
/// keep their registers.
pub struct Serial {
    pub twim: Twim,
    pub twi: Twi,
    regs: RegFile,
}

impl Serial {
    /// Whether the legacy TWI is the instance.
    fn is_twi(&self) -> bool {
        self.twim.enable == twi::ENABLED
    }
}

const ENABLE: u32 = 0x500;

pub struct Peripherals {
    pub clock: Arc<Clock>,
    pub variant: Variant,
    pub power: ClockPower,
    /// UARTE0, and UARTE1 on the nRF52833 and nRF52840.
    pub uarte: Vec<Uarte>,
    /// IDs 3 and 4.
    pub serial: [Serial; 2],
    pub gpiote: Gpiote,
    /// TIMER0-4.
    pub timers: Vec<Timer>,
    /// RTC0-2.
    pub rtc: Vec<Rtc>,
    pub temp: Temp,
    pub rng: Rng,
    pub wdt: Wdt,
    /// EGU0-5.
    pub egu: Vec<Egu>,
    pub ppi: Ppi,
    /// P0, and P1 on the nRF52833 and nRF52840.
    pub gpio: Vec<Gpio>,
    /// The board's I2C buses, by the pin of their SCL line (port << 5 | pin).
    pub i2c: BTreeMap<u32, I2cBus>,
    pub watcher: Option<Box<dyn PinWatcher>>,
    /// The other blocks of 4 KB, by base address.
    other: BTreeMap<u32, RegFile>,
    /// First accesses to registers not modelled, and the traced accesses.
    pub log: Vec<String>,
    /// Accesses in this range of addresses (start, end) are all logged.
    pub trace: Option<(u32, u32)>,
    /// The pin levels the GPIOTE and the watcher last saw.
    levels: [u32; 2],
    detect: bool,
    /// A reset the peripherals ask for, with its RESETREAS bit.
    reset: Option<u32>,
}

impl Peripherals {
    pub fn new(clock: Arc<Clock>, variant: Variant, seed: u64) -> Self {
        let dma = variant.dma_count_mask();
        let uartes = if variant.has_uarte1() { 2 } else { 1 };
        let mut gpio = vec![Gpio::new(32)];
        if variant.p1_pins() > 0 {
            gpio.push(Gpio::new(variant.p1_pins()));
        }
        Self {
            clock,
            variant,
            power: ClockPower::new(),
            uarte: (0..uartes).map(|_| Uarte::new(dma)).collect(),
            serial: [0, 1].map(|_| Serial {
                twim: Twim::new(dma),
                twi: Twi::new(),
                regs: RegFile::default(),
            }),
            gpiote: Gpiote::new(),
            timers: [4, 4, 4, 6, 6].into_iter().map(Timer::new).collect(),
            rtc: [3, 4, 4].into_iter().map(Rtc::new).collect(),
            temp: Temp::new(),
            rng: Rng::new(seed),
            wdt: Wdt::new(),
            egu: (0..6).map(|_| Egu::default()).collect(),
            ppi: Ppi::new(),
            gpio,
            i2c: BTreeMap::new(),
            watcher: None,
            other: BTreeMap::new(),
            log: Vec::new(),
            trace: None,
            levels: [0; 2],
            detect: false,
            reset: None,
        }
    }

    /// The peripherals after a chip reset for `reason` (a RESETREAS bit):
    /// all at their reset state, but for the retained POWER registers. The
    /// board's side stays: the pin inputs, the I2C buses, the watcher, the
    /// bytes on the UART lines.
    pub fn reset(&mut self, reason: u32) {
        let mut fresh = Self::new(self.clock.clone(), self.variant, self.rng.byte_seed());
        fresh.power = self.power.after_reset(reason);
        fresh.i2c = std::mem::take(&mut self.i2c);
        fresh.watcher = self.watcher.take();
        fresh.trace = self.trace;
        fresh.log = std::mem::take(&mut self.log);
        for (new, old) in fresh.gpio.iter_mut().zip(&self.gpio) {
            new.input = old.input;
            new.driven = old.driven;
        }
        for (new, old) in fresh.uarte.iter_mut().zip(self.uarte.iter_mut()) {
            new.line_in = std::mem::take(&mut old.line_in);
            new.line_out = std::mem::take(&mut old.line_out);
        }
        *self = fresh;
        self.sync_pins();
    }

    /// Takes the reset the peripherals ask for (the watchdog's).
    pub fn take_reset(&mut self) -> Option<u32> {
        self.reset.take()
    }

    fn device(&mut self, id: u32) -> Option<&mut dyn Device> {
        Some(match id {
            0 => &mut self.power,
            2 => &mut self.uarte[0],
            6 => &mut self.gpiote,
            8..=10 => &mut self.timers[(id - 8) as usize],
            11 => &mut self.rtc[0],
            12 => &mut self.temp,
            13 => &mut self.rng,
            16 => &mut self.wdt,
            17 => &mut self.rtc[1],
            20..=25 => &mut self.egu[(id - 20) as usize],
            26 | 27 => &mut self.timers[(id - 23) as usize],
            36 => &mut self.rtc[2],
            40 => return self.uarte.get_mut(1).map(|u| u as &mut dyn Device),
            _ => return None,
        })
    }

    /// Advances the peripherals to `now` and runs the PPI; returns the time
    /// of their next event.
    pub fn update(&mut self, now: u64, mem: &mut dyn Dma) -> Option<u64> {
        let mut next = None;
        for _ in 0..8 {
            next = None;
            let mut ctx = Ctx { now };
            for id in DEVICES {
                if let Some(device) = self.device(id) {
                    next = earliest(next, device.update(&mut ctx, mem));
                }
            }
            for serial in &mut self.serial {
                if serial.twim.enabled() {
                    next = earliest(next, serial.twim.update(&ctx, mem, &mut self.i2c));
                } else if serial.is_twi() {
                    next = earliest(next, serial.twi.update(&ctx, &mut self.i2c));
                }
            }
            if !self.route() {
                break;
            }
        }
        self.sync_pins();
        if let Some(at) = self.wdt.reset_at
            && at <= now
        {
            self.reset = Some(clock::RESET_DOG);
        }
        next
    }

    /// Routes the events generated since the last call through the PPI;
    /// whether a task was triggered.
    fn route(&mut self) -> bool {
        let mut tasks = Vec::new();
        let mut fired = Vec::new();
        for id in DEVICES {
            if let Some(device) = self.device(id) {
                fired.push((id, device.events().take_fired()));
            }
        }
        for (i, serial) in self.serial.iter_mut().enumerate() {
            fired.push((ID_SERIAL0 + i as u32, serial.twim.events.take_fired()));
            fired.push((ID_SERIAL0 + i as u32, serial.twi.events.take_fired()));
        }
        for (id, mask) in fired {
            let base = PERIPHERALS + 0x1000 * id;
            for i in (0..32).filter(|i| mask & (1 << i) != 0) {
                self.ppi.route(base + EVENTS + 4 * i, &mut tasks);
            }
        }
        for &task in &tasks {
            self.write_inner(task, 4, 1);
        }
        !tasks.is_empty()
    }

    /// The interrupt lines that are high.
    pub fn irq_lines(&mut self) -> Vec<u16> {
        let mut lines = Vec::new();
        for id in DEVICES {
            if let Some(device) = self.device(id)
                && device.events().irq()
            {
                lines.push(id as u16);
            }
        }
        for (i, serial) in self.serial.iter().enumerate() {
            if (serial.twim.enabled() && serial.twim.events.irq())
                || (serial.is_twi() && serial.twi.events.irq())
            {
                lines.push((ID_SERIAL0 + i as u32) as u16);
            }
        }
        lines
    }

    /// Whether a pin's SENSE condition holds: it wakes the chip from System
    /// OFF.
    pub fn detect(&self) -> bool {
        self.detect
    }

    /// The board drives `pin` of `port` high or low, or leaves it (`None`).
    pub fn set_input(&mut self, port: usize, pin: u32, level: Option<bool>) {
        let Some(gpio) = self.gpio.get_mut(port) else {
            return;
        };
        match level {
            Some(high) => {
                gpio.driven |= 1 << pin;
                if high {
                    gpio.input |= 1 << pin;
                } else {
                    gpio.input &= !(1 << pin);
                }
            }
            None => gpio.driven &= !(1 << pin),
        }
        self.sync_pins();
        self.clock.kick.store(true, Ordering::Relaxed);
    }

    /// The level of `pin` of `port`.
    pub fn level(&self, port: usize, pin: u32) -> bool {
        self.gpio.get(port).is_some_and(|g| g.level(pin))
    }

    /// Applies GPIOTE's outputs to the pins, then gives the pin changes to
    /// GPIOTE (edges, DETECT) and to the watcher.
    fn sync_pins(&mut self) {
        let driven = self.gpiote.driven();
        let mut levels = [0; 2];
        let mut detect = false;
        for (port, gpio) in self.gpio.iter_mut().enumerate() {
            gpio.gpiote_driven = driven[port].0;
            gpio.gpiote_out = driven[port].1;
            levels[port] = gpio.levels();
            detect |= gpio.detect();
        }
        self.gpiote.observe(self.levels, levels);
        if detect && !self.detect {
            self.gpiote.events.fire(gpio::PORT);
        }
        self.detect = detect;
        if levels != self.levels {
            let now = self.clock.now();
            if let Some(watcher) = self.watcher.as_mut() {
                let ports = self.gpio.len();
                for (port, (&new, &old)) in levels.iter().zip(&self.levels).enumerate().take(ports)
                {
                    if new != old {
                        watcher.changed(port, new, now);
                    }
                }
            }
            self.levels = levels;
        }
    }

    fn note(&mut self, kind: &str, address: u32) {
        let pc = self.clock.pc();
        self.log.push(format!(
            "{kind} {address:#010x} (pc {pc:#010x}): not modelled"
        ));
    }

    fn traced(&self, address: u32) -> bool {
        self.trace
            .is_some_and(|(start, end)| (start..end).contains(&address))
    }

    /// A read of `size` bytes at `address`; `None` outside the peripherals.
    pub fn read(&mut self, address: u32, size: u32) -> Option<u32> {
        let word = self.read_inner(address & !3)?;
        let value = sub_read(word, address, size);
        if self.traced(address) {
            let pc = self.clock.pc();
            self.log.push(format!(
                "read{} {address:#010x} = {value:#x} (pc {pc:#010x})",
                8 * size
            ));
        }
        Some(value)
    }

    /// A write of `size` bytes at `address`; `false` outside the
    /// peripherals.
    pub fn write(&mut self, address: u32, size: u32, value: u32) -> bool {
        if self.traced(address) {
            let pc = self.clock.pc();
            self.log.push(format!(
                "write{} {address:#010x} = {value:#x} (pc {pc:#010x})",
                8 * size
            ));
        }
        let ok = self.write_inner(address, size, value);
        // a GPIO write takes effect at once; the others may start something
        // the machine must service
        if (PERIPHERALS..PERIPHERALS_END).contains(&address) {
            self.clock.kick.store(true, Ordering::Relaxed);
        }
        ok
    }

    fn read_inner(&mut self, address: u32) -> Option<u32> {
        let now = self.clock.now();
        let mut ctx = Ctx { now };
        match address {
            a if (PERIPHERALS..PERIPHERALS_END).contains(&a) => {
                let (id, offset) = ((a - PERIPHERALS) >> 12, a & 0xFFF);
                if let Some(device) = self.device(id) {
                    return Some(device.read(offset, &mut ctx));
                }
                match id {
                    ID_SERIAL0 | ID_SERIAL1 => {
                        let serial = &mut self.serial[(id - ID_SERIAL0) as usize];
                        if serial.twim.enable == 0 || serial.twim.enabled() || offset == ENABLE {
                            return Some(serial.twim.read(offset));
                        }
                        if serial.is_twi() {
                            return Some(serial.twi.read(offset));
                        }
                        if serial.regs.first(offset) {
                            self.note("read", a);
                        }
                        let serial = &self.serial[(id - ID_SERIAL0) as usize];
                        Some(serial.regs.get(offset))
                    }
                    ID_PPI => Some(self.ppi.read(offset)),
                    _ => Some(self.other_read(a)),
                }
            }
            a if (P0..P0 + 0x800).contains(&a) => Some(self.gpio[0].read(a - P0)),
            a if (P0 + 0x800..GPIO_END).contains(&a) => {
                Some(self.gpio.get(1).map_or(0, |p1| p1.read(a - P1)))
            }
            _ => None,
        }
    }

    fn other_read(&mut self, address: u32) -> u32 {
        let base = address & !0xFFF;
        let first = self.other.entry(base).or_default().first(address & 0xFFF);
        if first {
            self.note("read", address);
        }
        self.other[&base].get(address & 0xFFF)
    }

    fn write_inner(&mut self, address: u32, size: u32, value: u32) -> bool {
        if !(PERIPHERALS..PERIPHERALS_END).contains(&address) && !(P0..GPIO_END).contains(&address)
        {
            return false;
        }
        // registers are 32 bits: a narrower write merges into the register
        let word = if size == 4 {
            value
        } else {
            let old = self.read_inner(address & !3).unwrap_or(0);
            sub_write(old, address, size, value)
        };
        let address = address & !3;
        let now = self.clock.now();
        let mut ctx = Ctx { now };
        match address {
            a if (PERIPHERALS..PERIPHERALS_END).contains(&a) => {
                let (id, offset) = ((a - PERIPHERALS) >> 12, a & 0xFFF);
                if let Some(device) = self.device(id) {
                    device.write(offset, word, &mut ctx);
                    if id == ID_GPIOTE {
                        self.sync_pins();
                    }
                    return true;
                }
                match id {
                    ID_SERIAL0 | ID_SERIAL1 => {
                        let serial = &mut self.serial[(id - ID_SERIAL0) as usize];
                        if offset == ENABLE || serial.twim.enable == 0 {
                            // ENABLE, and what a driver sets before it (pins,
                            // frequency, address), serve whichever instance
                            // ENABLE picks: the TWI keeps its own copy
                            serial.twi.write(offset, word, &ctx);
                        }
                        if serial.twim.enable == 0 || serial.twim.enabled() || offset == ENABLE {
                            serial.twim.write(offset, word, &ctx);
                        } else if serial.is_twi() {
                            serial.twi.write(offset, word, &ctx);
                        } else {
                            if serial.regs.first(offset) {
                                self.note("write", a);
                            }
                            self.serial[(id - ID_SERIAL0) as usize]
                                .regs
                                .set(offset, word);
                        }
                    }
                    ID_PPI => self.ppi.write(offset, word),
                    _ => {
                        let base = a & !0xFFF;
                        let first = self.other.entry(base).or_default().first(offset);
                        if first {
                            self.note("write", a);
                        }
                        if let Some(regs) = self.other.get_mut(&base) {
                            regs.set(offset, word);
                        }
                    }
                }
            }
            a if (P0..P0 + 0x800).contains(&a) => {
                self.gpio[0].write(a - P0, word);
                self.sync_pins();
            }
            a => {
                if let Some(p1) = self.gpio.get_mut(1) {
                    p1.write(a - P1, word);
                    self.sync_pins();
                }
            }
        }
        true
    }
}

fn earliest(a: Option<u64>, b: Option<u64>) -> Option<u64> {
    match (a, b) {
        (Some(a), Some(b)) => Some(a.min(b)),
        (a, b) => a.or(b),
    }
}

#[cfg(test)]
pub mod tests {
    use super::*;

    /// 64 KB of data RAM at 0x2000_0000.
    pub struct Ram {
        pub bytes: Vec<u8>,
    }

    impl Ram {
        pub fn new() -> Self {
            Self {
                bytes: vec![0; 64 << 10],
            }
        }

        fn range(&self, address: u32, len: usize) -> Option<std::ops::Range<usize>> {
            let start = address.checked_sub(0x2000_0000)? as usize;
            (start + len <= self.bytes.len()).then_some(start..start + len)
        }
    }

    impl Default for Ram {
        fn default() -> Self {
            Self::new()
        }
    }

    impl Dma for Ram {
        fn read(&mut self, address: u32, buf: &mut [u8]) -> bool {
            match self.range(address, buf.len()) {
                Some(r) => {
                    buf.copy_from_slice(&self.bytes[r]);
                    true
                }
                None => false,
            }
        }

        fn write(&mut self, address: u32, data: &[u8]) -> bool {
            match self.range(address, data.len()) {
                Some(r) => {
                    self.bytes[r].copy_from_slice(data);
                    true
                }
                None => false,
            }
        }
    }

    fn peripherals() -> Peripherals {
        Peripherals::new(Arc::new(Clock::default()), Variant::Nrf52833, 1)
    }

    #[test]
    fn uarte_rxdrdy_clears_and_starts_the_idle_timer_through_the_ppi() {
        let mut p = peripherals();
        let mut ram = Ram::new();
        // UARTE0 enabled at 9600 baud, receiving 8 bytes
        p.write(0x4000_2500, 4, 8);
        p.write(0x4000_2524, 4, 0x0027_5000);
        p.write(0x4000_2534, 4, 0x2000_0000);
        p.write(0x4000_2538, 4, 8);
        // TIMER1 at 16 MHz, CC0 = 2 bytes' time, COMPARE0 clears and stops
        p.write(0x4000_9510, 4, 0);
        p.write(0x4000_9508, 4, 3);
        p.write(0x4000_9540, 4, 33_334);
        p.write(0x4000_9200, 4, (1 << 0) | (1 << 8));
        // PPI 0: RXDRDY -> TIMER1 CLEAR, fork START; PPI 1: COMPARE0 -> STOPRX
        p.write(0x4001_F510, 4, 0x4000_2108);
        p.write(0x4001_F514, 4, 0x4000_900C);
        p.write(0x4001_F910, 4, 0x4000_9000);
        p.write(0x4001_F518, 4, 0x4000_9140);
        p.write(0x4001_F51C, 4, 0x4000_2004);
        p.write(0x4001_F504, 4, 3);
        p.write(0x4000_2000, 4, 1); // STARTRX
        p.uarte[0].receive(b"hi", 0);
        let mut now = 0;
        while let Some(next) = p.update(now, &mut ram) {
            now = next;
            p.clock.now.store(now, Ordering::Relaxed);
        }
        assert_eq!(&ram.bytes[..2], b"hi");
        assert_eq!(p.read(0x4000_253C, 4), Some(2), "RXD.AMOUNT");
        assert_eq!(
            p.read(0x4000_2144, 4),
            Some(1),
            "RXTO after the line went idle"
        );
        // the second byte ends at 2 frames; the timer, cleared then, counts
        // its 33 334 ticks from the phase of its 16 MHz clock
        let frame = 10 * 1_000_000_000_000 / 9597;
        assert!(now > 2 * frame + 33_333 * 62_500 && now <= 2 * frame + 33_334 * 62_500);
    }

    #[test]
    fn gpiote_sees_board_edges_and_drives_pins() {
        let mut p = peripherals();
        // GPIOTE 0: event on P1.09 falling; GPIOTE 1: task on P0.22
        p.write(0x4000_6510, 4, 1 | (9 << 8) | (1 << 13) | (2 << 16));
        p.write(0x4000_6514, 4, 3 | (22 << 8) | (3 << 16));
        p.set_input(1, 9, Some(true));
        p.set_input(1, 9, Some(false));
        assert_eq!(p.read(0x4000_6100, 4), Some(1));
        assert!(!p.level(0, 22));
        p.write(0x4000_6034, 4, 1); // TASKS_SET[1]
        assert!(p.level(0, 22));
        // the interrupt needs INTEN
        assert!(p.irq_lines().is_empty());
        p.write(0x4000_6304, 4, 1);
        assert_eq!(p.irq_lines(), [ID_GPIOTE as u16]);
    }

    #[test]
    fn narrow_writes_merge_into_the_register() {
        let mut p = peripherals();
        p.write(0x5000_0504, 4, 0x1234_5678);
        p.write(0x5000_0505, 1, 0xAB);
        assert_eq!(p.read(0x5000_0504, 4), Some(0x1234_AB78));
        assert_eq!(p.read(0x5000_0506, 2), Some(0x1234));
        // P1 has 10 pins on the nRF52833
        p.write(0x5000_0804, 4, u32::MAX);
        assert_eq!(p.read(0x5000_0804, 4), Some(0x3FF));
    }
}
