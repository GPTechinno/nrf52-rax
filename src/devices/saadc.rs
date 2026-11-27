//! SAADC: the successive-approximation ADC, eight channels, EasyDMA into a
//! RAM buffer (Product Specification, SAADC).
//!
//! START latches RESULT.PTR and RESULT.MAXCNT and gives STARTED; each
//! SAMPLE converts every enabled channel (PSELP not NC) in turn, each in its
//! TACQ plus 2 µs, and gives DONE; a result is the conversion, or with
//! OVERSAMPLE the average of 2^OVERSAMPLE of them, taken as fast as they
//! go in BURST mode and one per SAMPLE otherwise, and gives RESULTDONE, its
//! LIMITH and LIMITL events, and lands in RAM as a sign-extended 16-bit
//! value; END when RESULT.MAXCNT are there, which stops the sampling until
//! the next START. SAMPLERATE in timer mode samples again every CC / 16 MHz
//! from one SAMPLE until STOP or END. STOP gives STOPPED, started or not;
//! CALIBRATEOFFSET gives CALIBRATEDONE.
//!
//! The value is the data sheet's
//! `RESULT = (V(P) − V(N)) × GAIN / REFERENCE × 2^(RESOLUTION − m)`, m 1 in
//! differential mode, from the voltages the board puts on AIN0-7, VDD and
//! VDDH (VDDHDIV5 reads a fifth of it), clamped to the converter's range.
//!
//! Not modelled: the resistor ladder (only RESP/RESN at VDD/2, which set
//! the input there), offset, gain and linearity errors and noise, the
//! calibration's effect, STATUS busy during conversions (always 0), the
//! double-buffering of RESULT.PTR beyond the latch at START.

use std::collections::VecDeque;

use super::events::{self, Events};
use super::{Ctx, Device, Dma};

const PS_PER_US: u64 = 1_000_000;
/// The SAADC's timer, for SAMPLERATE.
const PS_PER_TICK: u64 = 62_500;

const STARTED: u32 = 0;
const END: u32 = 1;
const DONE: u32 = 2;
const RESULTDONE: u32 = 3;
const CALIBRATEDONE: u32 = 4;
const STOPPED: u32 = 5;
/// CH[n].LIMITH; LIMITL is the next.
const LIMITH: u32 = 6;

const ENABLE: u32 = 0x500;
const CH: u32 = 0x510;
const RESOLUTION: u32 = 0x5F0;
const OVERSAMPLE: u32 = 0x5F4;
const SAMPLERATE: u32 = 0x5F8;
const RESULT_PTR: u32 = 0x62C;
const RESULT_MAXCNT: u32 = 0x630;
const RESULT_AMOUNT: u32 = 0x634;

/// CH[n].CONFIG at reset: TACQ 10 µs.
const CONFIG_RESET: u32 = 0x0002_0000;
/// How long a calibration takes; the data sheet gives none.
const CALIBRATION_US: u64 = 100;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Op {
    Started,
    /// A SAMPLE's conversions are done.
    Converted,
    Calibrated,
    Stopped,
}

pub struct Saadc {
    pub events: Events,
    /// The voltages on AIN0 to AIN7, volts, as the board drives them.
    pub ain: [f64; 8],
    /// VDD and VDDH, volts.
    pub vdd: f64,
    pub vddh: f64,
    enable: u32,
    /// PSELP, PSELN, CONFIG, LIMIT of each channel.
    ch: [[u32; 4]; 8],
    resolution: u32,
    oversample: u32,
    samplerate: u32,
    ptr: u32,
    maxcnt: u32,
    amount: u32,
    /// The buffer START latched, while sampling may fill it.
    buffer: Option<(u32, u32)>,
    /// Conversions summed toward a result, per enabled channel, and how many.
    sums: Vec<i64>,
    count: u32,
    /// What happens when, in order.
    ops: VecDeque<(u64, Op)>,
    /// Timer mode: the next SAMPLE it triggers.
    next_sample: Option<u64>,
}

impl Default for Saadc {
    fn default() -> Self {
        Self::new()
    }
}

impl Saadc {
    pub fn new() -> Self {
        Self {
            events: Events::default(),
            ain: [0.0; 8],
            vdd: 3.0,
            vddh: 5.0,
            enable: 0,
            ch: [[0, 0, CONFIG_RESET, 0x7FFF_8000]; 8],
            resolution: 1,
            oversample: 0,
            samplerate: 0,
            ptr: 0,
            maxcnt: 0,
            amount: 0,
            buffer: None,
            sums: Vec::new(),
            count: 0,
            ops: VecDeque::new(),
            next_sample: None,
        }
    }

    fn enabled(&self) -> bool {
        self.enable & 1 != 0
    }

    /// The enabled channels: PSELP not NC.
    fn channels(&self) -> Vec<usize> {
        (0..8).filter(|&n| self.ch[n][0] & 0x1F != 0).collect()
    }

    fn oversampling(&self) -> u32 {
        1 << self.oversample.min(8)
    }

    fn burst(&self, n: usize) -> bool {
        self.ch[n][2] >> 24 & 1 != 0
    }

    /// The time one SAMPLE's conversions take.
    fn conversion_time(&self) -> u64 {
        const TACQ: [u64; 8] = [3, 5, 10, 15, 20, 40, 40, 40];
        let os = u64::from(self.oversampling());
        self.channels()
            .iter()
            .map(|&n| {
                let one = (TACQ[(self.ch[n][2] >> 16 & 7) as usize] + 2) * PS_PER_US;
                if self.burst(n) { one * os } else { one }
            })
            .sum::<u64>()
            .max(PS_PER_US)
    }

    /// The voltage a PSELP or PSELN code reads; `None` for NC.
    fn input(&self, code: u32) -> Option<f64> {
        match code & 0x1F {
            k @ 1..=8 => Some(self.ain[(k - 1) as usize]),
            9 => Some(self.vdd),
            0x0D => Some(self.vddh / 5.0),
            _ => None,
        }
    }

    /// One conversion of channel `n`.
    fn convert(&self, n: usize) -> i64 {
        const GAIN: [f64; 8] = [1.0 / 6.0, 0.2, 0.25, 1.0 / 3.0, 0.5, 1.0, 2.0, 4.0];
        let [pselp, pseln, config, _] = self.ch[n];
        let diff = config >> 20 & 1 != 0;
        let ladder = |code: u32, res: u32| if res == 3 { Some(self.vdd / 2.0) } else { self.input(code) };
        let vp = ladder(pselp, config & 3).unwrap_or(0.0);
        let vn = if diff { ladder(pseln, config >> 4 & 3).unwrap_or(0.0) } else { 0.0 };
        let gain = GAIN[(config >> 8 & 7) as usize];
        let reference = if config >> 12 & 1 != 0 { self.vdd / 4.0 } else { 0.6 };
        let bits = [8, 10, 12, 14][(self.resolution & 3) as usize] - i32::from(diff);
        let full = f64::from(1u32 << bits);
        let code = ((vp - vn) * gain / reference * full).round() as i64;
        let top = (1i64 << bits) - 1;
        code.clamp(-(1i64 << bits), top)
    }

    /// A SAMPLE's conversions are in: sum them, and write the results when
    /// there are enough.
    fn converted(&mut self, mem: &mut dyn Dma) {
        let channels = self.channels();
        if self.sums.len() != channels.len() {
            self.sums = vec![0; channels.len()];
            self.count = 0;
        }
        let os = self.oversampling();
        let burst = channels.iter().any(|&n| self.burst(n));
        let per_sample = if burst { os } else { 1 };
        let codes: Vec<i64> = channels.iter().map(|&n| self.convert(n)).collect();
        for (sum, code) in self.sums.iter_mut().zip(codes) {
            *sum += i64::from(per_sample) * code;
        }
        self.count += per_sample;
        self.events.fire(DONE);
        if self.count < os {
            return;
        }
        let results: Vec<i64> = self.sums.iter().map(|s| s / i64::from(os)).collect();
        self.sums.iter_mut().for_each(|s| *s = 0);
        self.count = 0;
        self.events.fire(RESULTDONE);
        for (&n, &r) in channels.iter().zip(&results) {
            let limit = self.ch[n][3];
            let (low, high) = (limit as u16 as i16, (limit >> 16) as u16 as i16);
            if r >= i64::from(high) {
                self.events.fire(LIMITH + 2 * n as u32);
            }
            if r <= i64::from(low) {
                self.events.fire(LIMITH + 2 * n as u32 + 1);
            }
            if let Some((ptr, maxcnt)) = self.buffer
                && self.amount < maxcnt
            {
                let at = ptr + 2 * self.amount;
                if mem.write(at, &(r as i16).to_le_bytes()) {
                    self.amount += 1;
                }
                if self.amount == maxcnt {
                    self.events.fire(END);
                    self.buffer = None;
                    self.next_sample = None;
                }
            }
        }
    }

    fn sample(&mut self, now: u64) {
        if !self.enabled() || self.buffer.is_none() {
            return;
        }
        if self.ops.iter().any(|&(_, op)| op == Op::Converted) {
            return;
        }
        self.schedule(now + self.conversion_time(), Op::Converted);
        if self.samplerate >> 12 & 1 != 0 {
            let cc = u64::from(self.samplerate & 0x7FF).max(80);
            self.next_sample = Some(now + cc * PS_PER_TICK);
        }
    }

    fn schedule(&mut self, at: u64, op: Op) {
        let i = self.ops.iter().position(|&(t, _)| t > at).unwrap_or(self.ops.len());
        self.ops.insert(i, (at, op));
    }
}

impl Device for Saadc {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            ENABLE => self.enable,
            o if (CH..CH + 0x80).contains(&o) => {
                self.ch[((o - CH) / 16) as usize][((o - CH) % 16 / 4) as usize]
            }
            RESOLUTION => self.resolution,
            OVERSAMPLE => self.oversample,
            SAMPLERATE => self.samplerate,
            RESULT_PTR => self.ptr,
            RESULT_MAXCNT => self.maxcnt,
            RESULT_AMOUNT => self.amount,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, ctx: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        match events::task(offset, value) {
            Some(0) if self.enabled() => {
                self.buffer = Some((self.ptr, self.maxcnt & 0x7FFF));
                self.amount = 0;
                self.schedule(ctx.now + PS_PER_US, Op::Started);
            }
            Some(1) => self.sample(ctx.now),
            Some(2) => {
                self.ops.retain(|&(_, op)| op != Op::Converted);
                self.buffer = None;
                self.next_sample = None;
                self.sums.clear();
                self.count = 0;
                self.schedule(ctx.now + PS_PER_US, Op::Stopped);
            }
            Some(3) if self.enabled() => {
                self.schedule(ctx.now + CALIBRATION_US * PS_PER_US, Op::Calibrated)
            }
            Some(_) => {}
            None => match offset {
                ENABLE => {
                    self.enable = value & 1;
                    if !self.enabled() {
                        self.ops.clear();
                        self.buffer = None;
                        self.next_sample = None;
                    }
                }
                o if (CH..CH + 0x80).contains(&o) => {
                    self.ch[((o - CH) / 16) as usize][((o - CH) % 16 / 4) as usize] = value;
                }
                RESOLUTION => self.resolution = value & 7,
                OVERSAMPLE => self.oversample = value & 0xF,
                SAMPLERATE => self.samplerate = value,
                RESULT_PTR => self.ptr = value,
                RESULT_MAXCNT => self.maxcnt = value,
                _ => {}
            },
        }
    }

    fn update(&mut self, ctx: &mut Ctx, mem: &mut dyn Dma) -> Option<u64> {
        loop {
            if let Some(at) = self.next_sample
                && at <= ctx.now
            {
                self.next_sample = None;
                self.sample(at);
                continue;
            }
            match self.ops.front() {
                Some(&(at, op)) if at <= ctx.now => {
                    self.ops.pop_front();
                    match op {
                        Op::Started => self.events.fire(STARTED),
                        Op::Converted => self.converted(mem),
                        Op::Calibrated => self.events.fire(CALIBRATEDONE),
                        Op::Stopped => self.events.fire(STOPPED),
                    }
                }
                _ => break,
            }
        }
        let next = self.ops.front().map(|&(at, _)| at);
        match (next, self.next_sample) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (a, b) => a.or(b),
        }
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Ram(Vec<u8>);

    impl Dma for Ram {
        fn read(&mut self, address: u32, buf: &mut [u8]) -> bool {
            let a = (address - 0x2000_0000) as usize;
            buf.copy_from_slice(&self.0[a..a + buf.len()]);
            true
        }
        fn write(&mut self, address: u32, data: &[u8]) -> bool {
            let a = (address - 0x2000_0000) as usize;
            self.0[a..a + data.len()].copy_from_slice(data);
            true
        }
    }

    fn run(s: &mut Saadc, ram: &mut Ram, until: u64) {
        let mut ctx = Ctx { now: until };
        s.update(&mut ctx, ram);
    }

    fn word(ram: &Ram, i: usize) -> i16 {
        i16::from_le_bytes([ram.0[2 * i], ram.0[2 * i + 1]])
    }

    /// The stock Scubajet Remote's battery channel: AIN7, gain 1/6, internal
    /// reference, TACQ 10 µs, 12 bits, one result per buffer.
    fn remote(s: &mut Saadc) {
        let mut ctx = Ctx { now: 0 };
        s.write(ENABLE, 1, &mut ctx);
        s.write(CH, 8, &mut ctx);
        s.write(CH + 8, 0x0002_0000, &mut ctx);
        s.write(RESOLUTION, 2, &mut ctx);
        s.write(RESULT_PTR, 0x2000_0000, &mut ctx);
        s.write(RESULT_MAXCNT, 1, &mut ctx);
    }

    #[test]
    fn a_sample_lands_in_ram_by_the_data_sheet_formula() {
        let mut s = Saadc::new();
        let mut ram = Ram(vec![0; 16]);
        remote(&mut s);
        // a 3.9 V cell behind 10 k over 33 k
        s.ain[7] = 3.9 * 33.0 / 43.0;
        s.write(0x000, 1, &mut Ctx { now: 0 });
        run(&mut s, &mut ram, PS_PER_US);
        assert!(s.events.is_set(STARTED));
        s.write(0x004, 1, &mut Ctx { now: PS_PER_US });
        run(&mut s, &mut ram, 12 * PS_PER_US);
        assert!(!s.events.is_set(DONE), "10 µs TACQ + 2 µs not over");
        run(&mut s, &mut ram, 13 * PS_PER_US);
        assert!(s.events.is_set(DONE) && s.events.is_set(RESULTDONE) && s.events.is_set(END));
        // 2.993 V × (1/6) / 0.6 × 4096 = 3405
        assert_eq!(word(&ram, 0), 3405);
        assert_eq!(s.read(RESULT_AMOUNT, &mut Ctx { now: 0 }), 1);
        // what the Remote makes of it: raw × 3.6 / 4096 × 43 / 33
        let v = f64::from(word(&ram, 0)) * 3.6 / 4096.0 * 43.0 / 33.0;
        assert!((v - 3.9).abs() < 0.002);
    }

    #[test]
    fn oversampling_averages_one_conversion_per_sample() {
        let mut s = Saadc::new();
        let mut ram = Ram(vec![0; 16]);
        remote(&mut s);
        s.write(OVERSAMPLE, 2, &mut Ctx { now: 0 });
        s.ain[7] = 0.6;
        s.write(0x000, 1, &mut Ctx { now: 0 });
        let mut t = PS_PER_US;
        for i in 0..4 {
            s.write(0x004, 1, &mut Ctx { now: t });
            t += 20 * PS_PER_US;
            run(&mut s, &mut ram, t);
            assert_eq!(s.events.is_set(RESULTDONE), i == 3, "after sample {i}");
        }
        assert_eq!(word(&ram, 0), 683);
    }

    #[test]
    fn stop_and_calibration_answer_started_or_not() {
        let mut s = Saadc::new();
        let mut ram = Ram(vec![0; 16]);
        remote(&mut s);
        s.write(0x008, 1, &mut Ctx { now: 0 });
        s.write(0x00C, 1, &mut Ctx { now: 0 });
        run(&mut s, &mut ram, 200 * PS_PER_US);
        assert!(s.events.is_set(STOPPED) && s.events.is_set(CALIBRATEDONE));
        // no buffer: a SAMPLE converts nothing
        s.write(0x004, 1, &mut Ctx { now: 200 * PS_PER_US });
        run(&mut s, &mut ram, 300 * PS_PER_US);
        assert!(!s.events.is_set(DONE));
    }
}
