//! The three chips: nRF52832, nRF52833 and nRF52840. They share the
//! Cortex-M4F, the memory map and the peripherals' design (each Product
//! Specification, chapters "Memory" and "Peripherals"); they differ in
//! memory sizes, the second GPIO port, a few instances, the width of the
//! EasyDMA counters and the factory information (FICR).

/// Which chip.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Variant {
    /// 512 KB flash, 64 KB RAM, P0 only, UARTE0 only, 8-bit EasyDMA counts.
    Nrf52832,
    /// 512 KB flash, 128 KB RAM, P0 and P1.00-P1.09, UARTE0 and UARTE1, USBD.
    Nrf52833,
    /// 1 MB flash, 256 KB RAM, P0 and P1.00-P1.15, UARTE0 and UARTE1, USBD.
    Nrf52840,
}

/// The flash page, the unit of an erase: 4 KB on the three chips.
pub const PAGE_SIZE: usize = 4096;

impl Variant {
    pub fn flash_size(self) -> usize {
        match self {
            Variant::Nrf52832 | Variant::Nrf52833 => 512 << 10,
            Variant::Nrf52840 => 1 << 20,
        }
    }

    pub fn ram_size(self) -> usize {
        match self {
            Variant::Nrf52832 => 64 << 10,
            Variant::Nrf52833 => 128 << 10,
            Variant::Nrf52840 => 256 << 10,
        }
    }

    /// The pins of P1 (none on the nRF52832).
    pub fn p1_pins(self) -> u32 {
        match self {
            Variant::Nrf52832 => 0,
            Variant::Nrf52833 => 10,
            Variant::Nrf52840 => 16,
        }
    }

    /// Whether UARTE1 (and USBD) exist.
    pub fn has_uarte1(self) -> bool {
        self != Variant::Nrf52832
    }

    /// The EasyDMA MAXCNT and AMOUNT fields: 8 bits on the nRF52832, 16 on
    /// the others.
    pub fn dma_count_mask(self) -> u32 {
        match self {
            Variant::Nrf52832 => 0xFF,
            Variant::Nrf52833 | Variant::Nrf52840 => 0xFFFF,
        }
    }

    /// FICR.INFO.PART.
    pub fn part(self) -> u32 {
        match self {
            Variant::Nrf52832 => 0x52832,
            Variant::Nrf52833 => 0x52833,
            Variant::Nrf52840 => 0x52840,
        }
    }

    /// FICR.INFO.VARIANT of the latest revision, ASCII: "AAG0", "AAB0",
    /// "AAF0". The build code (the third letter) tells firmware which
    /// access-port protection the chip has.
    pub fn latest_build(self) -> u32 {
        match self {
            Variant::Nrf52832 => u32::from_be_bytes(*b"AAG0"),
            Variant::Nrf52833 => u32::from_be_bytes(*b"AAB0"),
            Variant::Nrf52840 => u32::from_be_bytes(*b"AAF0"),
        }
    }

    /// FICR.INFO.PACKAGE: QFN48 (0x2000) for the nRF52832, aQFN73 (0x2004)
    /// for the others.
    fn package(self) -> u32 {
        match self {
            Variant::Nrf52832 => 0x2000,
            Variant::Nrf52833 | Variant::Nrf52840 => 0x2004,
        }
    }

    /// The ROM table's 4 KB at 0xF000_0000, of which firmware reads only the
    /// peripheral ID registers: older errata checks of Nordic's MDK and SDK
    /// tell the chip and its revision from PID0 (0xFE0, the part: 0x06 for
    /// the nRF52832 and 0x08 for the nRF52840; 0x0D for the nRF52833 [I],
    /// the code its FICR word 0x130 carries) and from the high nibble of
    /// PID2 (0xFE8), which reads 0 here: a check for an early nRF52832 (3)
    /// sees a later chip. The component ID (0xFF0-0xFFC) is CoreSight's.
    pub fn rom_table(self) -> Vec<u8> {
        let mut rom = vec![0; 4096];
        let part = match self {
            Variant::Nrf52832 => 0x06,
            Variant::Nrf52833 => 0x0D,
            Variant::Nrf52840 => 0x08,
        };
        for (offset, value) in [
            (0xFE0, part),
            (0xFF0, 0x0D),
            (0xFF4, 0x10),
            (0xFF8, 0x05),
            (0xFFC, 0xB1),
        ] {
            rom[offset] = value;
        }
        rom
    }

    /// The 4 KB of FICR: the code page size and count, the device ID and
    /// address, the part, variant, package, RAM and flash sizes. `device_id`
    /// makes DEVICEID and DEVICEADDR (a random static address); `build` is
    /// INFO.VARIANT. The rest reads erased (all ones), the words outside the
    /// register map among them; the temperature sensor's calibration (TEMP
    /// A0 to T4), which firmware copies into TEMP, reads zero.
    pub fn ficr(self, device_id: u64, build: u32) -> Vec<u8> {
        let mut ficr = vec![0xFF; 4096];
        let mut word = |offset: usize, value: u32| {
            ficr[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        };
        word(0x010, PAGE_SIZE as u32);
        word(0x014, (self.flash_size() / PAGE_SIZE) as u32);
        word(0x060, device_id as u32);
        word(0x064, (device_id >> 32) as u32);
        // DEVICEADDRTYPE: random; DEVICEADDR: its top two bits set make it
        // a random static address
        word(0x0A0, 0xFFFF_FFFF);
        word(0x0A4, (device_id >> 16) as u32);
        word(0x0A8, (device_id as u32 & 0xFFFF) | 0xC000);
        word(0x100, self.part());
        word(0x104, build);
        word(0x108, self.package());
        word(0x10C, (self.ram_size() >> 10) as u32);
        word(0x110, (self.flash_size() >> 10) as u32);
        // TEMP.A0-A5, B0-B5, T0-T4
        for i in 0..17 {
            word(0x404 + 4 * i, 0);
        }
        ficr
    }
}
