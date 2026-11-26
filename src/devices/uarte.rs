//! UARTE0 and UARTE1: a UART with EasyDMA. The board puts bytes on the
//! receive line (`line_in`) and takes the transmitted ones (`line_out`);
//! both move at the baud rate and frame (start bit, 8 data bits, parity,
//! one or two stop bits).
//!
//! - Receive: STARTRX latches RXD.PTR and RXD.MAXCNT (RXSTARTED). Each byte
//!   received gives RXDRDY and goes into the buffer, or into the 4-byte
//!   FIFO when the buffer is full (beyond it, an overrun ERROR); a full
//!   buffer gives ENDRX with RXD.AMOUNT. The next STARTRX takes the FIFO's
//!   bytes first. STOPRX ends the buffer (ENDRX) then gives RXTO; FLUSHRX
//!   moves the FIFO into the buffer. Bytes arriving while the receiver is
//!   stopped are lost (`rx_lost`).
//! - Transmit: STARTTX (TXSTARTED) reads TXD.MAXCNT bytes; each byte sent
//!   gives TXDRDY, the last ENDTX with TXD.AMOUNT. STOPTX gives ENDTX if it
//!   has not come, then TXSTOPPED.
//!
//! Not modelled: the legacy UART (ENABLE 4), flow control (CTS, NCTS,
//! RTS), parity, framing and break errors, the EasyDMA list.

use std::collections::VecDeque;

use super::events::{self, Events};
use super::{Ctx, Device, Dma};

const RXDRDY: u32 = 2;
const ENDRX: u32 = 4;
const TXDRDY: u32 = 7;
const ENDTX: u32 = 8;
const ERROR: u32 = 9;
const RXTO: u32 = 17;
const RXSTARTED: u32 = 19;
const TXSTARTED: u32 = 20;
const TXSTOPPED: u32 = 22;

const SHORT_ENDRX_STARTRX: u32 = 5;
const SHORT_ENDRX_STOPRX: u32 = 6;

const ERRORSRC: u32 = 0x480;
const ENABLE: u32 = 0x500;
const PSEL_RTS: u32 = 0x508;
const PSEL_RXD: u32 = 0x514;
const BAUDRATE: u32 = 0x524;
const RXD_PTR: u32 = 0x534;
const RXD_MAXCNT: u32 = 0x538;
const RXD_AMOUNT: u32 = 0x53C;
const TXD_PTR: u32 = 0x544;
const TXD_MAXCNT: u32 = 0x548;
const TXD_AMOUNT: u32 = 0x54C;
const CONFIG: u32 = 0x56C;

const ENABLED: u32 = 8;
const OVERRUN: u32 = 1 << 0;
const FIFO: usize = 4;
const PS_PER_S: u128 = 1_000_000_000_000;

pub struct Uarte {
    pub events: Events,
    enable: u32,
    /// PSEL.RTS, TXD, CTS, RXD.
    psel: [u32; 4],
    baudrate: u32,
    config: u32,
    errorsrc: u32,
    rxd: [u32; 3],
    txd: [u32; 3],
    /// MAXCNT's width.
    count_mask: u32,
    /// The receiver runs (STARTRX to STOPRX).
    rx_on: bool,
    /// The buffer being filled: pointer, size, bytes in it.
    rx_buf: Option<(u32, u32, u32)>,
    /// STARTRX was triggered: the FIFO's bytes go into the new buffer.
    rx_drain: bool,
    /// FLUSHRX was triggered.
    rx_flush: bool,
    fifo: VecDeque<u8>,
    /// Bytes the board sent, not received yet, and when the first one ends.
    pub line_in: VecDeque<u8>,
    rx_next: Option<u64>,
    /// Bytes lost while the receiver was stopped.
    pub rx_lost: usize,
    /// STARTTX was triggered: the buffer is read.
    tx_load: bool,
    /// Bytes being sent, when the first one ends, and how many were sent.
    tx_bytes: VecDeque<u8>,
    tx_next: Option<u64>,
    tx_sent: u32,
    /// ENDTX is still to come for the buffer.
    tx_on: bool,
    /// Bytes sent, for the board to take.
    pub line_out: Vec<u8>,
}

impl Uarte {
    pub fn new(count_mask: u32) -> Self {
        Self {
            events: Events::default(),
            enable: 0,
            psel: [u32::MAX; 4],
            baudrate: 0x0400_0000,
            config: 0,
            errorsrc: 0,
            rxd: [0; 3],
            txd: [0; 3],
            count_mask,
            rx_on: false,
            rx_buf: None,
            rx_drain: false,
            rx_flush: false,
            fifo: VecDeque::new(),
            line_in: VecDeque::new(),
            rx_next: None,
            rx_lost: 0,
            tx_load: false,
            tx_bytes: VecDeque::new(),
            tx_next: None,
            tx_sent: 0,
            tx_on: false,
            line_out: Vec::new(),
        }
    }

    pub fn enabled(&self) -> bool {
        self.enable == ENABLED
    }

    /// The pin the transmitter drives (PSEL.TXD: port << 5 | pin, bit 31
    /// set when disconnected).
    pub fn txd_pin(&self) -> u32 {
        self.psel[1]
    }

    /// The pin the receiver listens on (PSEL.RXD).
    pub fn rxd_pin(&self) -> u32 {
        self.psel[3]
    }

    /// The baud rate, from BAUDRATE (its bits 31:12, in steps of 16 MHz /
    /// 2^20).
    pub fn baud(&self) -> u64 {
        let steps = u64::from(self.baudrate >> 12);
        ((steps * 16_000_000) >> 20).max(1)
    }

    /// One frame on the line, picoseconds.
    pub fn byte_time(&self) -> u64 {
        let parity = u32::from(self.config & 0xE != 0);
        let stop = if self.config & 0x10 != 0 { 2 } else { 1 };
        let bits = u128::from(1 + 8 + parity + stop);
        (bits * PS_PER_S / u128::from(self.baud())) as u64
    }

    /// The board sends `bytes` on the receive line.
    pub fn receive(&mut self, bytes: &[u8], now: u64) {
        self.line_in.extend(bytes);
        if self.rx_next.is_none() && !self.line_in.is_empty() {
            self.rx_next = Some(now + self.byte_time());
        }
    }

    fn start_rx(&mut self) {
        self.rx_on = true;
        self.rx_buf = Some((self.rxd[0], self.rxd[1] & self.count_mask, 0));
        self.rx_drain = true;
        self.events.fire(RXSTARTED);
    }

    /// The buffer is done: ENDRX, and its shorts unless `stopping`.
    fn end_rx(&mut self, stopping: bool) {
        let Some((_, _, amount)) = self.rx_buf.take() else {
            return;
        };
        self.rxd[2] = amount;
        self.events.fire(ENDRX);
        if stopping {
            return;
        }
        if self.events.short(SHORT_ENDRX_STARTRX) {
            self.start_rx();
        }
        if self.events.short(SHORT_ENDRX_STOPRX) {
            self.stop_rx();
        }
    }

    fn stop_rx(&mut self) {
        if self.rx_on {
            self.end_rx(true);
            self.rx_on = false;
            self.events.fire(RXTO);
        }
    }

    /// Puts `byte` into the buffer if it has room.
    fn store(&mut self, byte: u8, mem: &mut dyn Dma) -> bool {
        let Some((ptr, maxcnt, amount)) = self.rx_buf.as_mut() else {
            return false;
        };
        if *amount >= *maxcnt {
            return false;
        }
        mem.write(*ptr + *amount, &[byte]);
        *amount += 1;
        if *amount == *maxcnt {
            self.end_rx(false);
        }
        true
    }

    fn receive_byte(&mut self, byte: u8, mem: &mut dyn Dma) {
        if !self.rx_on || !self.enabled() {
            self.rx_lost += 1;
            return;
        }
        self.events.fire(RXDRDY);
        if !self.fifo.is_empty() || !self.store(byte, mem) {
            if self.fifo.len() < FIFO {
                self.fifo.push_back(byte);
            } else {
                self.errorsrc |= OVERRUN;
                self.events.fire(ERROR);
            }
        }
    }

    fn stop_tx(&mut self) {
        self.tx_load = false;
        self.tx_bytes.clear();
        self.tx_next = None;
        if self.tx_on {
            self.tx_on = false;
            self.txd[2] = self.tx_sent;
            self.events.fire(ENDTX);
        }
        self.events.fire(TXSTOPPED);
    }

    fn task(&mut self, index: u32) {
        if !self.enabled() {
            return;
        }
        match index {
            0 => self.start_rx(),
            1 => self.stop_rx(),
            2 => {
                self.tx_load = true;
                self.tx_on = true;
                self.events.fire(TXSTARTED);
            }
            3 => self.stop_tx(),
            11 => self.rx_flush = true,
            _ => {}
        }
    }
}

impl Device for Uarte {
    fn read(&mut self, offset: u32, _: &mut Ctx) -> u32 {
        if let Some(value) = self.events.read(offset) {
            return value;
        }
        match offset {
            ERRORSRC => self.errorsrc,
            ENABLE => self.enable,
            o if (PSEL_RTS..=PSEL_RXD).contains(&o) => self.psel[((o - PSEL_RTS) / 4) as usize],
            BAUDRATE => self.baudrate,
            o if (RXD_PTR..=RXD_AMOUNT).contains(&o) => self.rxd[((o - RXD_PTR) / 4) as usize],
            o if (TXD_PTR..=TXD_AMOUNT).contains(&o) => self.txd[((o - TXD_PTR) / 4) as usize],
            CONFIG => self.config,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32, _: &mut Ctx) {
        if self.events.write(offset, value) {
            return;
        }
        if let Some(index) = events::task(offset, value) {
            self.task(index);
            return;
        }
        match offset {
            // write 1 to clear
            ERRORSRC => self.errorsrc &= !value,
            ENABLE => {
                self.enable = value & 0xF;
                if !self.enabled() {
                    self.rx_on = false;
                    self.rx_buf = None;
                    self.tx_load = false;
                    self.tx_bytes.clear();
                    self.tx_next = None;
                    self.tx_on = false;
                }
            }
            o if (PSEL_RTS..=PSEL_RXD).contains(&o) => {
                self.psel[((o - PSEL_RTS) / 4) as usize] = value;
            }
            BAUDRATE => self.baudrate = value,
            RXD_PTR | RXD_MAXCNT => self.rxd[((offset - RXD_PTR) / 4) as usize] = value,
            TXD_PTR | TXD_MAXCNT => self.txd[((offset - TXD_PTR) / 4) as usize] = value,
            CONFIG => self.config = value,
            _ => {}
        }
    }

    fn update(&mut self, ctx: &mut Ctx, mem: &mut dyn Dma) -> Option<u64> {
        let now = ctx.now;
        if std::mem::take(&mut self.rx_drain) {
            while let Some(&byte) = self.fifo.front() {
                if !self.store(byte, mem) {
                    break;
                }
                self.fifo.pop_front();
            }
        }
        if std::mem::take(&mut self.rx_flush) && !self.rx_on {
            let (ptr, maxcnt) = (self.rxd[0], self.rxd[1] & self.count_mask);
            let n = self.fifo.len().min(maxcnt as usize);
            let bytes: Vec<u8> = self.fifo.drain(..n).collect();
            mem.write(ptr, &bytes);
            self.rxd[2] = n as u32;
            self.events.fire(ENDRX);
        }
        while let Some(at) = self.rx_next.filter(|&at| at <= now) {
            if let Some(byte) = self.line_in.pop_front() {
                self.receive_byte(byte, mem);
            }
            self.rx_next = (!self.line_in.is_empty()).then(|| at + self.byte_time());
        }
        if std::mem::take(&mut self.tx_load) {
            let (ptr, maxcnt) = (self.txd[0], self.txd[1] & self.count_mask);
            let mut bytes = vec![0; maxcnt as usize];
            mem.read(ptr, &mut bytes);
            self.tx_bytes = bytes.into();
            self.tx_sent = 0;
            self.tx_next = Some(now + self.byte_time());
        }
        while let Some(at) = self.tx_next.filter(|&at| at <= now) {
            match self.tx_bytes.pop_front() {
                Some(byte) => {
                    self.line_out.push(byte);
                    self.tx_sent += 1;
                    self.events.fire(TXDRDY);
                    self.tx_next = Some(at + self.byte_time());
                }
                None => self.tx_next = None,
            }
            if self.tx_bytes.is_empty() && self.tx_on {
                self.tx_on = false;
                self.tx_next = None;
                self.txd[2] = self.tx_sent;
                self.events.fire(ENDTX);
            }
        }
        [self.rx_next, self.tx_next].into_iter().flatten().min()
    }

    fn events(&mut self) -> &mut Events {
        &mut self.events
    }
}

#[cfg(test)]
mod tests {
    use super::super::tests::Ram;
    use super::*;

    const PS_9600: u64 = 10 * 1_000_000_000_000 / 9597;

    fn uarte() -> Uarte {
        let mut uarte = Uarte::new(0xFFFF);
        let mut ctx = Ctx { now: 0 };
        uarte.write(ENABLE, 8, &mut ctx);
        uarte.write(BAUDRATE, 0x0027_5000, &mut ctx); // 9600 baud
        uarte
    }

    #[test]
    fn baud_rate_and_frame() {
        let uarte = uarte();
        assert_eq!(uarte.baud(), 9597);
        assert_eq!(uarte.byte_time(), PS_9600);
    }

    #[test]
    fn receives_into_the_buffer_then_the_fifo() {
        let mut uarte = uarte();
        let mut ram = Ram::new();
        let mut ctx = Ctx { now: 0 };
        uarte.write(RXD_PTR, 0x2000_0100, &mut ctx);
        uarte.write(RXD_MAXCNT, 2, &mut ctx);
        uarte.write(0x000, 1, &mut ctx); // STARTRX
        assert!(uarte.events.is_set(RXSTARTED));
        uarte.receive(b"abcd", 0);
        let mut ctx = Ctx { now: 4 * PS_9600 };
        assert_eq!(uarte.update(&mut ctx, &mut ram), None);
        assert!(uarte.events.is_set(ENDRX));
        assert_eq!(uarte.read(RXD_AMOUNT, &mut ctx), 2);
        assert_eq!(&ram.bytes[0x100..0x102], b"ab");
        assert_eq!(uarte.fifo.len(), 2, "the receiver runs on into the FIFO");
        // the next buffer takes the FIFO first
        uarte.write(RXD_PTR, 0x2000_0200, &mut ctx);
        uarte.write(RXD_MAXCNT, 8, &mut ctx);
        uarte.write(0x000, 1, &mut ctx);
        uarte.update(&mut ctx, &mut ram);
        assert_eq!(&ram.bytes[0x200..0x202], b"cd");
        uarte.write(0x110, 0, &mut ctx);
        uarte.write(0x004, 1, &mut ctx); // STOPRX
        assert!(uarte.events.is_set(ENDRX) && uarte.events.is_set(RXTO));
        assert_eq!(uarte.read(RXD_AMOUNT, &mut ctx), 2);
        uarte.receive(b"e", ctx.now);
        let mut ctx = Ctx {
            now: ctx.now + PS_9600,
        };
        uarte.update(&mut ctx, &mut ram);
        assert_eq!(uarte.rx_lost, 1);
    }

    #[test]
    fn transmits_at_the_baud_rate() {
        let mut uarte = uarte();
        let mut ram = Ram::new();
        ram.bytes[0x300..0x303].copy_from_slice(b"xyz");
        let mut ctx = Ctx { now: 0 };
        uarte.write(TXD_PTR, 0x2000_0300, &mut ctx);
        uarte.write(TXD_MAXCNT, 3, &mut ctx);
        uarte.write(0x008, 1, &mut ctx); // STARTTX
        assert_eq!(uarte.update(&mut ctx, &mut ram), Some(PS_9600));
        let mut ctx = Ctx { now: 2 * PS_9600 };
        assert_eq!(uarte.update(&mut ctx, &mut ram), Some(3 * PS_9600));
        assert_eq!(uarte.line_out, b"xy");
        assert!(!uarte.events.is_set(ENDTX));
        let mut ctx = Ctx { now: 3 * PS_9600 };
        assert_eq!(uarte.update(&mut ctx, &mut ram), None);
        assert!(uarte.events.is_set(ENDTX));
        assert_eq!(uarte.read(TXD_AMOUNT, &mut ctx), 3);
    }
}
