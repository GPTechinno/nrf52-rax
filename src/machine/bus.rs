//! The memory map (Product Specification, "Memory"): the flash at 0, the
//! data RAM at 0x2000_0000 and its alias for code at 0x0080_0000, the
//! factory information (FICR) and the user information (UICR), the NVMC
//! that programs and erases the flash and the UICR, and the peripherals.
//!
//! The NVMC programs and erases at once (READY always reads 1): with
//! CONFIG.WEN a write clears the bits that are 0 in the value (programming
//! sets no bit back to 1); with CONFIG.EEN, ERASEPAGE erases the page of
//! the flash it is given, ERASEALL the flash and the UICR, ERASEUICR the
//! UICR; with CONFIG.PEEN, ERASEPAGEPARTIAL erases its page at the first
//! call.

use std::fmt;
use std::sync::{Arc, Mutex};

use rax::isa::arm::common::cpu::AccessType;
use rax::isa::arm::common::memory::{ArmMemory, MemResult, MemoryError, MmioHandler};

use crate::devices::Peripherals;
use crate::devices::regfile::{RegFile, sub_read, sub_write};
use crate::variant::{PAGE_SIZE, Variant};

pub const RAM_BASE: u32 = 0x2000_0000;
/// The data RAM seen on the code bus.
pub const CODE_RAM: u32 = 0x0080_0000;
pub const FICR: u32 = 0x1000_0000;
pub const UICR: u32 = 0x1000_1000;
const INFO_SIZE: usize = 0x1000;
const NVMC: u32 = 0x4001_E000;

const READY: u32 = 0x400;
const READYNEXT: u32 = 0x408;
const CONFIG: u32 = 0x504;
const ERASEPAGE: u32 = 0x508;
const ERASEALL: u32 = 0x50C;
const ERASEPCR0: u32 = 0x510;
const ERASEUICR: u32 = 0x514;
const ERASEPAGEPARTIAL: u32 = 0x518;
const WEN: u32 = 1;
const EEN: u32 = 2;
const PEEN: u32 = 4;

pub struct Bus {
    pub flash: Vec<u8>,
    pub ram: Vec<u8>,
    ficr: Vec<u8>,
    pub uicr: Vec<u8>,
    nvmc_config: u32,
    nvmc: RegFile,
    periph: Arc<Mutex<Peripherals>>,
    exclusive: Option<(u64, u8)>,
}

impl fmt::Debug for Bus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bus")
            .field("flash", &self.flash.len())
            .field("ram", &self.ram.len())
            .finish_non_exhaustive()
    }
}

fn unmapped(address: u64, size: usize, access: AccessType) -> MemoryError {
    MemoryError::Unmapped {
        addr: address,
        size,
        access,
    }
}

/// `offset..offset + len` of a memory starting at `base` of `size` bytes, if
/// `address..address + len` is in it.
fn within(address: u64, len: usize, base: u32, size: usize) -> Option<usize> {
    let offset = address.checked_sub(u64::from(base))? as usize;
    (offset + len <= size).then_some(offset)
}

/// Where an access lands among the memories.
enum Area {
    Flash,
    Ram,
    Ficr,
    Uicr,
}

impl Bus {
    pub fn new(periph: Arc<Mutex<Peripherals>>, variant: Variant, ficr: Vec<u8>) -> Self {
        Self {
            flash: vec![0xFF; variant.flash_size()],
            ram: vec![0; variant.ram_size()],
            ficr,
            uicr: vec![0xFF; INFO_SIZE],
            nvmc_config: 0,
            nvmc: RegFile::with(&[(0x51C, 0x0A), (0x540, 0x0000_0001)]),
            periph,
            exclusive: None,
        }
    }

    fn area(&self, address: u64, len: usize) -> Option<(Area, usize)> {
        if let Some(o) = within(address, len, 0, self.flash.len()) {
            return Some((Area::Flash, o));
        }
        if let Some(o) = within(address, len, RAM_BASE, self.ram.len()) {
            return Some((Area::Ram, o));
        }
        if let Some(o) = within(address, len, CODE_RAM, self.ram.len()) {
            return Some((Area::Ram, o));
        }
        if let Some(o) = within(address, len, FICR, INFO_SIZE) {
            return Some((Area::Ficr, o));
        }
        within(address, len, UICR, INFO_SIZE).map(|o| (Area::Uicr, o))
    }

    fn bytes(&self, area: &Area) -> &[u8] {
        match area {
            Area::Flash => &self.flash,
            Area::Ram => &self.ram,
            Area::Ficr => &self.ficr,
            Area::Uicr => &self.uicr,
        }
    }

    /// Writes `bytes` at `address` whatever the memory (loading an image):
    /// the flash, the RAM or the UICR.
    pub fn load(&mut self, address: u32, bytes: &[u8]) -> bool {
        let Some((area, o)) = self.area(u64::from(address), bytes.len()) else {
            return false;
        };
        let target = match area {
            Area::Flash => &mut self.flash,
            Area::Ram => &mut self.ram,
            Area::Uicr => &mut self.uicr,
            Area::Ficr => return false,
        };
        target[o..o + bytes.len()].copy_from_slice(bytes);
        true
    }

    fn nvmc_read(&self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            READY | READYNEXT => 1,
            CONFIG => self.nvmc_config,
            o => self.nvmc.get(o),
        };
        sub_read(word, offset, size)
    }

    fn erase_page(&mut self, address: u32) {
        let page = address as usize & !(PAGE_SIZE - 1);
        if page + PAGE_SIZE <= self.flash.len() {
            self.flash[page..page + PAGE_SIZE].fill(0xFF);
        } else if address & !0xFFF == UICR {
            self.uicr.fill(0xFF);
        }
    }

    fn nvmc_write(&mut self, offset: u32, size: u32, value: u32) {
        let word = sub_write(self.nvmc_read(offset, 4), offset, size, value);
        match offset & !3 {
            CONFIG => self.nvmc_config = word & 7,
            ERASEPAGE | ERASEPCR0 if self.nvmc_config & EEN != 0 => self.erase_page(word),
            ERASEPAGEPARTIAL if self.nvmc_config & PEEN != 0 => self.erase_page(word),
            ERASEALL if self.nvmc_config & EEN != 0 && word & 1 != 0 => {
                self.flash.fill(0xFF);
                self.uicr.fill(0xFF);
            }
            ERASEUICR if self.nvmc_config & EEN != 0 && word & 1 != 0 => self.uicr.fill(0xFF),
            READY | READYNEXT | ERASEPAGE | ERASEPCR0 | ERASEPAGEPARTIAL | ERASEALL | ERASEUICR => {
            }
            o => self.nvmc.set(o, word),
        }
    }

    fn mmio_read(&self, address: u64, buf: &mut [u8]) -> MemResult<()> {
        let len = buf.len();
        let fail = || unmapped(address, len, AccessType::Read);
        let address32 = u32::try_from(address).map_err(|_| fail())?;
        let mut done = 0;
        while done < buf.len() {
            let at = address32 + done as u32;
            let size = access_size(at, buf.len() - done);
            let value = if (NVMC..NVMC + 0x1000).contains(&at) {
                self.nvmc_read(at - NVMC, size)
            } else {
                let mut p = self.periph.lock().map_err(|_| fail())?;
                p.read(at, size).ok_or_else(fail)?
            };
            buf[done..done + size as usize].copy_from_slice(&value.to_le_bytes()[..size as usize]);
            done += size as usize;
        }
        Ok(())
    }

    fn mmio_write(&mut self, address: u64, data: &[u8]) -> MemResult<()> {
        let len = data.len();
        let fail = || unmapped(address, len, AccessType::Write);
        let address32 = u32::try_from(address).map_err(|_| fail())?;
        let mut done = 0;
        while done < data.len() {
            let at = address32 + done as u32;
            let size = access_size(at, data.len() - done);
            let mut word = [0u8; 4];
            word[..size as usize].copy_from_slice(&data[done..done + size as usize]);
            let value = u32::from_le_bytes(word);
            if (NVMC..NVMC + 0x1000).contains(&at) {
                self.nvmc_write(at - NVMC, size, value);
            } else {
                let mut p = self.periph.lock().map_err(|_| fail())?;
                if !p.write(at, size, value) {
                    return Err(fail());
                }
            }
            done += size as usize;
        }
        Ok(())
    }
}

/// The widest aligned access at `address` for `remaining` bytes.
fn access_size(address: u32, remaining: usize) -> u32 {
    match remaining {
        n if n >= 4 && address.is_multiple_of(4) => 4,
        n if n >= 2 && address.is_multiple_of(2) => 2,
        _ => 1,
    }
}

impl ArmMemory for Bus {
    fn read(&self, address: u64, buf: &mut [u8]) -> MemResult<()> {
        match self.area(address, buf.len()) {
            Some((area, o)) => {
                buf.copy_from_slice(&self.bytes(&area)[o..o + buf.len()]);
                Ok(())
            }
            None => self.mmio_read(address, buf),
        }
    }

    fn write(&mut self, address: u64, data: &[u8]) -> MemResult<()> {
        match self.area(address, data.len()) {
            Some((Area::Ram, o)) => {
                self.ram[o..o + data.len()].copy_from_slice(data);
                Ok(())
            }
            Some((area @ (Area::Flash | Area::Uicr), o)) => {
                // programming clears bits; without WEN the write is ignored
                if self.nvmc_config & WEN != 0 {
                    let target = match area {
                        Area::Flash => &mut self.flash,
                        _ => &mut self.uicr,
                    };
                    for (byte, value) in target[o..o + data.len()].iter_mut().zip(data) {
                        *byte &= value;
                    }
                }
                Ok(())
            }
            Some((Area::Ficr, _)) => Ok(()),
            None => self.mmio_write(address, data),
        }
    }

    fn mark_exclusive(&mut self, address: u64, size: u8) {
        self.exclusive = Some((address, size));
    }

    fn check_exclusive(&mut self, address: u64, size: u8) -> bool {
        self.exclusive.take() == Some((address, size))
    }

    fn clear_exclusive(&mut self) {
        self.exclusive = None;
    }

    fn register_mmio(&mut self, _base: u64, _size: u64, _handler: Box<dyn MmioHandler>) {}

    fn unregister_mmio(&mut self, _base: u64) {}
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::Clock;

    fn bus() -> Bus {
        let periph = Peripherals::new(Arc::new(Clock::default()), Variant::Nrf52833, 1);
        Bus::new(
            Arc::new(Mutex::new(periph)),
            Variant::Nrf52833,
            Variant::Nrf52833.ficr(1, Variant::Nrf52833.latest_build()),
        )
    }

    fn word(bus: &Bus, address: u32) -> u32 {
        let mut b = [0; 4];
        bus.read(u64::from(address), &mut b).unwrap();
        u32::from_le_bytes(b)
    }

    #[test]
    fn the_nvmc_programs_and_erases() {
        let mut bus = bus();
        let pselreset = UICR + 0x200;
        bus.write(u64::from(pselreset), &18u32.to_le_bytes())
            .unwrap();
        assert_eq!(word(&bus, pselreset), u32::MAX, "no WEN: ignored");
        bus.write(u64::from(NVMC + CONFIG), &WEN.to_le_bytes())
            .unwrap();
        bus.write(u64::from(pselreset), &18u32.to_le_bytes())
            .unwrap();
        bus.write(u64::from(0x1000u32), &0x1234_5678u32.to_le_bytes())
            .unwrap();
        bus.write(u64::from(0x1000u32), &0xFFFF_0000u32.to_le_bytes())
            .unwrap();
        assert_eq!(word(&bus, pselreset), 18);
        assert_eq!(
            word(&bus, 0x1000),
            0x1234_0000,
            "programming only clears bits"
        );
        bus.write(u64::from(NVMC + CONFIG), &EEN.to_le_bytes())
            .unwrap();
        bus.write(u64::from(NVMC + ERASEPAGE), &0x1004u32.to_le_bytes())
            .unwrap();
        assert_eq!(word(&bus, 0x1000), u32::MAX);
        bus.write(u64::from(NVMC + ERASEUICR), &1u32.to_le_bytes())
            .unwrap();
        assert_eq!(word(&bus, pselreset), u32::MAX);
        assert_eq!(word(&bus, NVMC + READY), 1);
    }

    #[test]
    fn the_ram_is_seen_on_both_buses_and_ficr_describes_the_chip() {
        let mut bus = bus();
        bus.write(u64::from(RAM_BASE + 8), &[1, 2, 3, 4]).unwrap();
        assert_eq!(word(&bus, CODE_RAM + 8), 0x0403_0201);
        assert_eq!(word(&bus, FICR + 0x100), 0x52833);
        assert_eq!(word(&bus, FICR + 0x10C), 128);
        assert!(
            bus.read(u64::from(RAM_BASE + (128 << 10)), &mut [0; 4])
                .is_err()
        );
    }
}
