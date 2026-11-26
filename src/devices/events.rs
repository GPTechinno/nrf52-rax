//! What every peripheral shares (Product Specification, "Peripheral
//! interface"): event `i` is the register at `0x100 + 4 i`, set by the
//! peripheral and cleared by firmware writing 0; INTEN bit `i` lets it raise
//! the peripheral's interrupt (INTEN at 0x300, INTENSET at 0x304, INTENCLR at
//! 0x308); SHORTS (0x200) connects a peripheral's events to its own tasks.
//! Tasks are the registers below 0x100, triggered by writing 1.

/// The first event register, and the number of them.
pub const EVENTS: u32 = 0x100;
const EVENT_COUNT: u32 = 32;
const SHORTS: u32 = 0x200;
const INTEN: u32 = 0x300;
const INTENSET: u32 = 0x304;
const INTENCLR: u32 = 0x308;

#[derive(Clone, Debug, Default)]
pub struct Events {
    /// The events set.
    pub pending: u32,
    pub inten: u32,
    pub shorts: u32,
    /// The events generated since they were last taken, for the PPI.
    fired: u32,
}

impl Events {
    /// Generates event `i`.
    pub fn fire(&mut self, i: u32) {
        self.pending |= 1 << i;
        self.fired |= 1 << i;
    }

    pub fn is_set(&self, i: u32) -> bool {
        self.pending & (1 << i) != 0
    }

    /// Whether SHORTS bit `bit` is set.
    pub fn short(&self, bit: u32) -> bool {
        self.shorts & (1 << bit) != 0
    }

    /// The interrupt line: an enabled event is set.
    pub fn irq(&self) -> bool {
        self.pending & self.inten != 0
    }

    /// The events generated since the last call.
    pub fn take_fired(&mut self) -> u32 {
        std::mem::take(&mut self.fired)
    }

    /// A read of an event, SHORTS or INTEN register; `None` for another
    /// offset.
    pub fn read(&self, offset: u32) -> Option<u32> {
        match offset {
            o if (EVENTS..EVENTS + 4 * EVENT_COUNT).contains(&o) => {
                Some(u32::from(self.is_set((o - EVENTS) / 4)))
            }
            SHORTS => Some(self.shorts),
            INTEN | INTENSET | INTENCLR => Some(self.inten),
            _ => None,
        }
    }

    /// A write of an event, SHORTS or INTEN register; whether the offset was
    /// one. Writing 1 to an event generates it, as the peripheral would.
    pub fn write(&mut self, offset: u32, value: u32) -> bool {
        match offset {
            o if (EVENTS..EVENTS + 4 * EVENT_COUNT).contains(&o) => {
                let i = (o - EVENTS) / 4;
                if value & 1 != 0 {
                    self.fire(i);
                } else {
                    self.pending &= !(1 << i);
                }
            }
            SHORTS => self.shorts = value,
            INTEN => self.inten = value,
            INTENSET => self.inten |= value,
            INTENCLR => self.inten &= !value,
            _ => return false,
        }
        true
    }
}

/// The index of the task at `offset`, if it is a task triggered (a write of
/// 1 below 0x100).
pub fn task(offset: u32, value: u32) -> Option<u32> {
    (offset < EVENTS && value & 1 != 0).then_some(offset / 4)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn events_raise_the_interrupt_when_enabled_and_clear_on_zero() {
        let mut events = Events::default();
        events.fire(4);
        assert_eq!(events.read(0x110), Some(1));
        assert!(!events.irq());
        events.write(INTENSET, 1 << 4);
        assert!(events.irq());
        assert_eq!(events.take_fired(), 1 << 4);
        assert_eq!(events.take_fired(), 0);
        events.write(0x110, 0);
        assert!(!events.irq());
        events.write(INTENCLR, 1 << 4);
        assert_eq!(events.read(INTEN), Some(0));
    }
}
