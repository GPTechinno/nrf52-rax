//! The chip: the Cortex-M4F on the bus, booted from an ELF image, run in
//! virtual time.
//!
//! Time advances by one cycle of the 64 MHz core clock per instruction.
//! While the core sleeps (WFI, WFE), time jumps to the next peripheral event
//! or SysTick interrupt, or to the end of the run, and SysTick counts the
//! cycles skipped. The peripherals are serviced between slices of
//! instructions, and after any instruction that wrote one of their
//! registers: they advance, move their EasyDMA data, route their events
//! through the PPI, and raise their interrupt lines (as levels: an
//! interrupt is pended while its line is high and it is neither pending nor
//! active).
//!
//! A system reset (SYSRESETREQ, the watchdog, a wake from System OFF)
//! resets the core and the peripherals; the flash, the UICR, the RAM and the
//! retained registers stay.
//!
//! With [`Config::softdevice`], a supervisor call is answered by the
//! SoftDevice model ([`crate::softdevice`]) as soon as the instruction runs,
//! and the stack's events pend SWI2/EGU2.

mod bus;

use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex, MutexGuard};

use goblin::elf::{Elf, program_header::PT_LOAD};
use rax::isa::arm::common::cpu::{ArmCpu, CpuExit};
use rax::isa::arm::common::memory::ArmMemory;
use rax::isa::arm::cortex_m::{CortexMCpu, CortexMVariant};

use bus::Bus;
pub use bus::{CODE_RAM, FICR, RAM_BASE, UICR};

use crate::devices::clock::{RESET_DOG, RESET_SREQ};
use crate::devices::{Clock, Dma, Peripherals};
use crate::softdevice::{EVENT_IRQ, EVENT_IRQ_PRIORITY, Guest, SoftDevice};
use crate::variant::Variant;

/// Instructions between two looks at the peripherals.
const SLICE: u32 = 256;
/// One cycle of the 64 MHz core clock, picoseconds.
const PS_PER_CYCLE: u64 = 15_625;
/// RESETREAS.OFF: woken from System OFF by a pin.
const RESET_OFF: u32 = 1 << 16;

pub struct Config {
    pub variant: Variant,
    /// FICR.DEVICEID, which also makes FICR.DEVICEADDR.
    pub device_id: u64,
    /// FICR.INFO.VARIANT; `None` for the chip's latest revision.
    pub build: Option<u32>,
    /// Seed of the RNG.
    pub seed: u64,
    /// Where the core boots: its vector table, the reset value of VTOR (0,
    /// the start of the flash, where a SoftDevice puts its MBR; an
    /// application linked above a SoftDevice boots at its own address when
    /// the SoftDevice is absent).
    pub boot_address: u32,
    /// Images loaded before the ELF: (address, bytes) in the flash, the
    /// UICR or the RAM (a SoftDevice, a bootloader, UICR settings).
    pub images: Vec<(u32, Vec<u8>)>,
    /// Answer the SoftDevice's supervisor calls (an application linked
    /// above a SoftDevice, booted at its own address with no SoftDevice
    /// image).
    pub softdevice: bool,
}

impl Config {
    pub fn new(variant: Variant) -> Self {
        Self {
            variant,
            device_id: 0x0123_4567_89AB_CDEF,
            build: None,
            seed: 1,
            boot_address: 0,
            images: Vec::new(),
            softdevice: false,
        }
    }
}

/// Why a run stopped early.
#[derive(Debug, Clone)]
pub enum Event {
    /// A BKPT instruction, with its immediate.
    Breakpoint { pc: u32, imm: u32 },
    /// An instruction the core reports instead of taking a fault: undefined
    /// or unimplemented, a bus error, a lockup.
    Fault { pc: u32, message: String },
    /// The firmware entered System OFF; a pin's SENSE condition wakes the
    /// chip with a reset.
    SystemOff,
}

pub struct Nrf52 {
    cpu: CortexMCpu,
    periph: Arc<Mutex<Peripherals>>,
    clock: Arc<Clock>,
    variant: Variant,
    /// Picoseconds.
    now: u64,
    next_event: Option<u64>,
    /// In System OFF.
    off: bool,
    pub instructions: u64,
    /// Time spent asleep, picoseconds.
    pub asleep: u64,
    /// Chip resets since power-on, with their RESETREAS bit.
    pub resets: Vec<u32>,
    softdevice: Option<SoftDevice>,
    /// Supervisor calls with no SoftDevice to answer them, noted once.
    svc_noted: bool,
}

/// The core's registers and memory, as a SoftDevice call sees them.
struct CoreGuest<'a>(&'a mut CortexMCpu);

impl Guest for CoreGuest<'_> {
    fn reg(&self, n: u8) -> u32 {
        self.0.get_gpr(n) as u32
    }
    fn set_reg(&mut self, n: u8, value: u32) {
        self.0.set_gpr(n, u64::from(value));
    }
    fn read(&self, address: u32, len: usize) -> Option<Vec<u8>> {
        let mut buf = vec![0; len];
        self.0
            .memory()
            .read(u64::from(address), &mut buf)
            .ok()
            .map(|()| buf)
    }
    fn write(&mut self, address: u32, data: &[u8]) -> bool {
        self.0.memory_mut().write(u64::from(address), data).is_ok()
    }
}

impl Nrf52 {
    /// The chip with the images of `cfg` and `elf` loaded (its loadable
    /// segments, at their physical addresses), powered on.
    pub fn new(cfg: Config, elf: &[u8]) -> Result<Self, String> {
        let variant = cfg.variant;
        let clock = Arc::new(Clock::default());
        let periph = Arc::new(Mutex::new(Peripherals::new(
            clock.clone(),
            variant,
            cfg.seed,
        )));
        let build = cfg.build.unwrap_or(variant.latest_build());
        let mut bus = Bus::new(periph.clone(), variant, variant.ficr(cfg.device_id, build));
        for (address, bytes) in &cfg.images {
            if !bus.load(*address, bytes) {
                return Err(format!(
                    "image of {} bytes at {address:#010x} outside the memories",
                    bytes.len()
                ));
            }
        }
        let image = Elf::parse(elf).map_err(|e| format!("ELF: {e}"))?;
        for ph in image
            .program_headers
            .iter()
            .filter(|ph| ph.p_type == PT_LOAD)
        {
            if ph.p_filesz == 0 {
                continue;
            }
            let start = ph.p_offset as usize;
            let bytes = elf
                .get(start..start + ph.p_filesz as usize)
                .ok_or("ELF: segment beyond the file")?;
            if !bus.load(ph.p_paddr as u32, bytes) {
                return Err(format!(
                    "ELF: segment of {} bytes at {:#010x} outside the memories",
                    bytes.len(),
                    ph.p_paddr
                ));
            }
        }
        let mut cpu = CortexMCpu::new(CortexMVariant::CortexM4, Box::new(bus));
        cpu.set_floating_point(true);
        cpu.map_private_peripheral_bus(true);
        cpu.set_initial_vtor(cfg.boot_address);
        let mut machine = Self {
            cpu,
            periph,
            clock,
            variant,
            now: 0,
            next_event: None,
            off: false,
            instructions: 0,
            asleep: 0,
            resets: Vec::new(),
            softdevice: cfg.softdevice.then(SoftDevice::new),
            svc_noted: false,
        };
        machine.cpu.reset();
        machine.service();
        Ok(machine)
    }

    pub fn variant(&self) -> Variant {
        self.variant
    }

    /// A chip reset for `reason` (a RESETREAS bit).
    fn reset_chip(&mut self, reason: u32) {
        self.periph().reset(reason);
        self.softdevice = self.softdevice.take().map(SoftDevice::after_reset);
        self.cpu.reset();
        self.off = false;
        self.resets.push(reason);
        self.service();
    }

    /// A pin reset (the nRESET pin).
    pub fn pin_reset(&mut self) {
        self.reset_chip(crate::devices::clock::RESET_PIN);
    }

    /// The supply comes back after going away: a power-on reset. The RAM is
    /// lost (it reads zero), RESETREAS and the retained registers read zero,
    /// the bytes on the UART lines are gone; the flash and the UICR stay, and
    /// so does the board's side (pin inputs, I2C devices, the watcher).
    /// Between [`Nrf52::run_for`] calls the board decides when the supply is
    /// there; nothing runs while it is not.
    pub fn power_on(&mut self) {
        let ram = vec![0u8; self.variant.ram_size()];
        let _ = self.cpu.memory_mut().write(u64::from(RAM_BASE), &ram);
        self.periph().power_on_reset();
        self.softdevice = self.softdevice.take().map(SoftDevice::after_reset);
        self.cpu.reset();
        self.off = false;
        self.resets.push(0);
        self.service();
    }

    pub fn periph(&self) -> MutexGuard<'_, Peripherals> {
        self.periph.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn cpu(&self) -> &CortexMCpu {
        &self.cpu
    }

    /// The SoftDevice model, with [`Config::softdevice`].
    pub fn softdevice(&mut self) -> Option<&mut SoftDevice> {
        self.softdevice.as_mut()
    }

    /// Virtual time, picoseconds.
    pub fn now(&self) -> u64 {
        self.now
    }

    /// Whether the chip is in System OFF.
    pub fn system_off(&self) -> bool {
        self.off
    }

    pub fn pc(&self) -> u32 {
        self.cpu.get_pc() as u32
    }

    /// R0-R12, SP, LR, PC.
    pub fn registers(&self) -> [u32; 16] {
        let mut regs = [0; 16];
        for (i, r) in regs.iter_mut().enumerate().take(13) {
            *r = self.cpu.get_gpr(i as u8) as u32;
        }
        regs[13] = self.cpu.get_sp() as u32;
        regs[14] = self.cpu.get_lr() as u32;
        regs[15] = self.pc();
        regs
    }

    pub fn read_bytes(&self, address: u32, len: usize) -> Option<Vec<u8>> {
        let mut buf = vec![0; len];
        self.cpu
            .memory()
            .read(u64::from(address), &mut buf)
            .ok()
            .map(|()| buf)
    }

    pub fn read_u32(&self, address: u32) -> Option<u32> {
        let bytes = self.read_bytes(address, 4)?;
        Some(u32::from_le_bytes(bytes.try_into().ok()?))
    }

    pub fn write_u32(&mut self, address: u32, value: u32) -> bool {
        self.cpu
            .memory_mut()
            .write(u64::from(address), &value.to_le_bytes())
            .is_ok()
    }

    /// The flash, as the firmware left it.
    pub fn flash(&self) -> Vec<u8> {
        self.read_bytes(0, self.variant.flash_size())
            .unwrap_or_default()
    }

    /// The UICR, as the firmware left it.
    pub fn uicr(&self) -> Vec<u8> {
        self.read_bytes(UICR, 0x1000).unwrap_or_default()
    }

    /// Runs for `duration` picoseconds of virtual time, or until an event.
    pub fn run_for(&mut self, duration: u64) -> Option<Event> {
        let end = self.now.saturating_add(duration);
        while self.now < end {
            if self.off {
                // only a pin wakes the chip: the board changes them between
                // runs
                self.now = end;
                self.clock.now.store(self.now, Ordering::Relaxed);
                self.service();
                if self.periph().detect() {
                    self.reset_chip(RESET_OFF);
                }
                break;
            }
            let target = self.next_event.map_or(end, |t| t.clamp(self.now + 1, end));
            let mut slice = 0;
            while self.now < target && slice < SLICE {
                let pc = self.pc();
                self.clock.pc.store(pc, Ordering::Relaxed);
                match self.cpu.step() {
                    Ok(CpuExit::Wfi | CpuExit::Wfe) => {
                        self.sleep(target);
                        break;
                    }
                    Ok(CpuExit::Breakpoint(imm)) => return Some(Event::Breakpoint { pc, imm }),
                    Ok(CpuExit::Svc(imm)) => self.supervisor_call(imm),
                    Ok(_) => {}
                    Err(e) => {
                        return Some(Event::Fault {
                            pc,
                            message: e.to_string(),
                        });
                    }
                }
                self.now += PS_PER_CYCLE;
                self.instructions += 1;
                slice += 1;
                self.clock.now.store(self.now, Ordering::Relaxed);
                if self.clock.kick.load(Ordering::Relaxed) {
                    break;
                }
            }
            self.clock.kick.store(false, Ordering::Relaxed);
            self.clock.now.store(self.now, Ordering::Relaxed);
            if self.cpu.scb_mut().take_reset_request() {
                self.reset_chip(RESET_SREQ);
                continue;
            }
            self.service();
            let reset = self.periph().take_reset();
            if let Some(reason) = reset {
                debug_assert_eq!(reason, RESET_DOG);
                self.reset_chip(reason);
                continue;
            }
            if self.periph().power.system_off {
                self.off = true;
                return Some(Event::SystemOff);
            }
        }
        None
    }

    /// A supervisor call: answered by the SoftDevice model if there is one.
    fn supervisor_call(&mut self, imm: u32) {
        let now = self.now;
        let answered = match self.softdevice.as_mut() {
            Some(sd) => {
                let answered = sd.call(imm, &mut CoreGuest(&mut self.cpu), now);
                let enabled = sd.take_enabled();
                if sd.take_irq() {
                    self.cpu.nvic_mut().set_pending(EVENT_IRQ);
                }
                if enabled {
                    // IPR bytes are MSB-aligned; the nRF52 implements three bits
                    self.cpu.nvic_mut().set_priority(EVENT_IRQ, EVENT_IRQ_PRIORITY << 5);
                }
                answered
            }
            None => false,
        };
        if !answered && !self.svc_noted {
            self.svc_noted = true;
            let pc = self.pc();
            self.periph().log.push(format!(
                "SVC #{imm} at {pc:#010x} not answered (no SoftDevice)"
            ));
        }
    }

    /// Sleeps until `target`, or SysTick's next interrupt before it.
    fn sleep(&mut self, target: u64) {
        let systick = self.cpu.systick();
        let mut wake = target.max(self.now + PS_PER_CYCLE);
        if systick.is_enabled()
            && systick.is_interrupt_enabled()
            && let Some(cycles) = systick.cycles_until_tick()
        {
            wake = wake.min(self.now + cycles.max(1) * PS_PER_CYCLE);
        }
        let cycles = (wake - self.now) / PS_PER_CYCLE;
        if cycles > 0 {
            self.cpu.systick_mut().tick(cycles);
        }
        self.asleep += wake - self.now;
        self.now = wake;
    }

    /// Advances the peripherals, runs their EasyDMA, raises their
    /// interrupts.
    fn service(&mut self) {
        let periph = self.periph.clone();
        let ram = self.variant.ram_size() as u32;
        let lines = {
            let mut p = periph.lock().unwrap_or_else(|e| e.into_inner());
            let mut dma = RamDma {
                memory: self.cpu.memory_mut(),
                end: RAM_BASE + ram,
                faults: Vec::new(),
            };
            self.next_event = p.update(self.now, &mut dma);
            let faults = dma.faults;
            for address in faults {
                p.log
                    .push(format!("EasyDMA outside the data RAM: {address:#010x}"));
            }
            p.irq_lines()
        };
        let nvic = self.cpu.nvic_mut();
        for irq in lines {
            if !nvic.is_pending(irq) && !nvic.is_active(irq) {
                nvic.set_pending(irq);
            }
        }
        if let Some(sd) = self.softdevice.as_mut() {
            sd.poll(self.now);
            if let Some(t) = sd.next_arrival() {
                self.next_event = Some(self.next_event.map_or(t, |n| n.min(t)));
            }
            if sd.take_irq() {
                self.cpu.nvic_mut().set_pending(EVENT_IRQ);
            }
            if !sd.notes.is_empty() {
                let notes = std::mem::take(&mut sd.notes);
                periph
                    .lock()
                    .unwrap_or_else(|e| e.into_inner())
                    .log
                    .extend(notes);
            }
        }
    }

    /// The bytes waiting in RTT up buffer `channel` of the control block at
    /// `control_block`, which are then consumed.
    pub fn rtt_read(&mut self, control_block: u32, channel: u32) -> Vec<u8> {
        // Control block: id[16], max_up, max_down, then up buffers of 24
        // bytes: name, buffer, size, write offset, read offset, flags.
        let Some(max_up) = self.read_u32(control_block + 16) else {
            return Vec::new();
        };
        if channel >= max_up {
            return Vec::new();
        }
        let desc = control_block + 24 + 24 * channel;
        let field = |m: &Self, i: u32| m.read_u32(desc + 4 * i).unwrap_or(0);
        let (buf, size, wr, rd) = (
            field(self, 1),
            field(self, 2),
            field(self, 3),
            field(self, 4),
        );
        if size == 0 || wr >= size || rd >= size || wr == rd {
            return Vec::new();
        }
        let out = if wr > rd {
            self.read_bytes(buf + rd, (wr - rd) as usize)
                .unwrap_or_default()
        } else {
            let mut out = self
                .read_bytes(buf + rd, (size - rd) as usize)
                .unwrap_or_default();
            out.extend(self.read_bytes(buf, wr as usize).unwrap_or_default());
            out
        };
        self.write_u32(desc + 16, wr);
        out
    }
}

/// EasyDMA reaches the data RAM only.
struct RamDma<'a> {
    memory: &'a mut dyn ArmMemory,
    end: u32,
    faults: Vec<u32>,
}

impl RamDma<'_> {
    fn allowed(&mut self, address: u32, len: usize) -> bool {
        let ok = address >= RAM_BASE && u64::from(address) + len as u64 <= u64::from(self.end);
        if !ok {
            self.faults.push(address);
        }
        ok
    }
}

impl Dma for RamDma<'_> {
    fn read(&mut self, address: u32, buf: &mut [u8]) -> bool {
        if buf.is_empty() {
            return true;
        }
        if !self.allowed(address, buf.len()) {
            buf.fill(0);
            return false;
        }
        self.memory.read(u64::from(address), buf).is_ok()
    }

    fn write(&mut self, address: u32, data: &[u8]) -> bool {
        if data.is_empty() {
            return true;
        }
        self.allowed(address, data.len()) && self.memory.write(u64::from(address), data).is_ok()
    }
}
