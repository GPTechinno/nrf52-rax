//! Headless runner: boots an ELF on the emulated chip for a while, printing
//! the defmt logs (RTT up buffer 0), the bytes the UARTEs send, the first
//! accesses to peripherals not modelled, and why it stopped.
//!
//! run [--variant 52832|52833|52840] [--ms N] [--hex FILE]... [--boot ADDRESS] [--softdevice]
//!     [--i2c PORT.PIN:ADDR]... [--trace START:END] ELF
//!
//! `--softdevice` answers the SoftDevice's calls (with `--boot 27000` for an
//! application linked above S140 7.x) and prints what happens on its BLE
//! connection; nobody connects.

use std::time::Instant;

use defmt_decoder::{DecodeError, Table};
use nrf52_rax::devices::twim::{I2cBus, I2cDevice};
use nrf52_rax::{Config, Event, Nrf52, Variant};

const PS_PER_MS: u64 = 1_000_000_000;

/// A device that acknowledges everything and counts the bytes written.
struct Sink {
    address: u8,
    written: std::sync::Arc<std::sync::atomic::AtomicUsize>,
}

impl I2cDevice for Sink {
    fn write(&mut self, _: u8) -> bool {
        self.written
            .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        true
    }
    fn read(&mut self) -> u8 {
        0
    }
    fn stop(&mut self) {
        let _ = self.address;
    }
}

/// The data records of an Intel HEX file: (address, bytes).
fn parse_hex(text: &str) -> Result<Vec<(u32, Vec<u8>)>, String> {
    let mut images = Vec::new();
    let mut base = 0u32;
    for (n, line) in text.lines().enumerate() {
        let line = line.trim();
        let Some(hex) = line.strip_prefix(':') else {
            continue;
        };
        let bytes: Vec<u8> = (0..hex.len() / 2)
            .map(|i| u8::from_str_radix(&hex[2 * i..2 * i + 2], 16))
            .collect::<Result<_, _>>()
            .map_err(|e| format!("line {}: {e}", n + 1))?;
        if bytes.len() < 5 || bytes.iter().fold(0u8, |s, b| s.wrapping_add(*b)) != 0 {
            return Err(format!("line {}: bad record", n + 1));
        }
        let len = bytes[0] as usize;
        let offset = u32::from(u16::from_be_bytes([bytes[1], bytes[2]]));
        let data = &bytes[4..4 + len];
        match bytes[3] {
            0 => images.push((base + offset, data.to_vec())),
            2 => base = u32::from(u16::from_be_bytes([data[0], data[1]])) << 4,
            4 => base = u32::from(u16::from_be_bytes([data[0], data[1]])) << 16,
            _ => {}
        }
    }
    Ok(images)
}

fn symbol(elf: &[u8], name: &str) -> Option<u32> {
    let obj = goblin::elf::Elf::parse(elf).ok()?;
    obj.syms
        .iter()
        .find(|s| obj.strtab.get_at(s.st_name) == Some(name))
        .map(|s| s.st_value as u32)
}

fn main() -> Result<(), String> {
    let mut variant = Variant::Nrf52833;
    let mut ms = 1000u64;
    let mut images = Vec::new();
    let mut trace = None;
    let mut boot = 0;
    let mut softdevice = false;
    let mut sinks: Vec<(u32, u8)> = Vec::new();
    let mut elf_path = None;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--variant" => {
                variant = match args.next().as_deref() {
                    Some("52832") => Variant::Nrf52832,
                    Some("52833") => Variant::Nrf52833,
                    Some("52840") => Variant::Nrf52840,
                    other => return Err(format!("unknown variant {other:?}")),
                }
            }
            "--boot" => {
                let v = args.next().ok_or("--boot ADDRESS")?;
                boot = u32::from_str_radix(v.trim_start_matches("0x"), 16)
                    .map_err(|e| e.to_string())?;
            }
            "--softdevice" => softdevice = true,
            "--i2c" => {
                // PORT.PIN of SCL, and the 7-bit address in hex: 0.27:3c
                let v = args.next().ok_or("--i2c PORT.PIN:ADDRESS")?;
                let (pin, address) = v.split_once(':').ok_or("--i2c PORT.PIN:ADDRESS")?;
                let (port, pin) = pin.split_once('.').ok_or("--i2c PORT.PIN:ADDRESS")?;
                let scl = (port.parse::<u32>().map_err(|e| e.to_string())? << 5)
                    | pin.parse::<u32>().map_err(|e| e.to_string())?;
                sinks.push((
                    scl,
                    u8::from_str_radix(address, 16).map_err(|e| e.to_string())?,
                ));
            }
            "--ms" => ms = args.next().and_then(|v| v.parse().ok()).ok_or("--ms N")?,
            "--hex" => {
                let path = args.next().ok_or("--hex FILE")?;
                let text = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
                images.extend(parse_hex(&text).map_err(|e| format!("{path}: {e}"))?);
            }
            "--trace" => {
                let range = args.next().ok_or("--trace START:END")?;
                let (a, b) = range.split_once(':').ok_or("--trace START:END")?;
                let parse = |s: &str| {
                    u32::from_str_radix(s.trim_start_matches("0x"), 16).map_err(|e| e.to_string())
                };
                trace = Some((parse(a)?, parse(b)?));
            }
            path => elf_path = Some(path.to_string()),
        }
    }
    let elf_path = elf_path.ok_or("usage: run [--variant 52833] [--ms N] [--hex FILE]... ELF")?;
    let elf = std::fs::read(&elf_path).map_err(|e| format!("{elf_path}: {e}"))?;

    let table = Table::parse(&elf).ok().flatten();
    let mut decoder = table.as_ref().map(|t| t.new_stream_decoder());
    let rtt = symbol(&elf, "_SEGGER_RTT");

    let mut config = Config::new(variant);
    config.images = images;
    config.boot_address = boot;
    config.softdevice = softdevice;
    let mut chip = Nrf52::new(config, &elf)?;
    chip.periph().trace = trace;
    let written = std::sync::Arc::new(std::sync::atomic::AtomicUsize::new(0));
    for (scl, address) in sinks {
        let sink = Sink {
            address,
            written: written.clone(),
        };
        chip.periph()
            .i2c
            .entry(scl)
            .or_insert_with(I2cBus::new)
            .insert(address, Box::new(sink));
    }

    let started = Instant::now();
    let mut logged = 0;
    let mut advertised: Option<(bool, Vec<u8>)> = None;
    let mut stop = None;
    for _ in 0..ms {
        let event = chip.run_for(PS_PER_MS);
        if let Some(rtt) = rtt {
            let bytes = chip.rtt_read(rtt, 0);
            match decoder.as_mut() {
                Some(decoder) => {
                    decoder.received(&bytes);
                    loop {
                        match decoder.decode() {
                            Ok(frame) => {
                                let level = frame
                                    .level()
                                    .map(|l| l.as_str().to_uppercase())
                                    .unwrap_or_default();
                                println!(
                                    "{:>10.3} ms  {level:<5} {}",
                                    chip.now() as f64 / PS_PER_MS as f64,
                                    frame.display_message()
                                );
                            }
                            Err(DecodeError::UnexpectedEof) => break,
                            Err(DecodeError::Malformed) => println!("(malformed defmt frame)"),
                        }
                    }
                }
                None if !bytes.is_empty() => print!("{}", String::from_utf8_lossy(&bytes)),
                None => {}
            }
        }
        let mut p = chip.periph();
        for (i, uarte) in p.uarte.iter_mut().enumerate() {
            if !uarte.line_out.is_empty() {
                let bytes = std::mem::take(&mut uarte.line_out);
                let hex: Vec<String> = bytes.iter().map(|b| format!("{b:02x}")).collect();
                println!(
                    "{:>10.3} ms  UARTE{i} sent {}",
                    chip.now() as f64 / PS_PER_MS as f64,
                    hex.join(" ")
                );
            }
        }
        for line in &p.log[logged..] {
            println!("               {line}");
        }
        logged = p.log.len();
        drop(p);
        let now_ms = chip.now() as f64 / PS_PER_MS as f64;
        if let Some(sd) = chip.softdevice() {
            for e in sd.host.drain(..) {
                println!("{now_ms:>10.3} ms  BLE {e:?}");
            }
            let adv = sd.advertising().map(|a| (a.connectable, a.data.clone()));
            if adv != advertised {
                match &adv {
                    Some((connectable, data)) => {
                        let hex: Vec<String> = data.iter().map(|b| format!("{b:02x}")).collect();
                        println!(
                            "{now_ms:>10.3} ms  BLE advertising{}: {}",
                            if *connectable { ", connectable" } else { "" },
                            hex.join(" ")
                        );
                    }
                    None => println!("{now_ms:>10.3} ms  BLE not advertising"),
                }
                advertised = adv;
            }
        }
        if let Some(event) = event {
            if !matches!(event, Event::SystemOff) {
                stop = Some(event);
                break;
            }
            println!(
                "{:>10.3} ms  System OFF",
                chip.now() as f64 / PS_PER_MS as f64
            );
        }
    }
    let host = started.elapsed().as_secs_f64();
    if let Some(event) = &stop {
        println!("stopped: {event:?}");
        let r = chip.registers();
        println!(
            "r0 {:08x} r1 {:08x} r2 {:08x} r3 {:08x} r12 {:08x} sp {:08x} lr {:08x} pc {:08x}",
            r[0], r[1], r[2], r[3], r[12], r[13], r[14], r[15]
        );
    }
    let written = written.load(std::sync::atomic::Ordering::Relaxed);
    if written > 0 {
        println!("I2C bytes written to the attached devices: {written}");
    }
    let virtual_s = chip.now() as f64 / 1e12;
    println!(
        "{:.3} s virtual in {host:.2} s, {} instructions ({:.1} MIPS), asleep {:.1} %, resets {:?}",
        virtual_s,
        chip.instructions,
        chip.instructions as f64 / host / 1e6,
        100.0 * chip.asleep as f64 / chip.now().max(1) as f64,
        chip.resets
    );
    Ok(())
}
