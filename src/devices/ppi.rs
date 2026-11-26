//! PPI: 32 channels, each connecting an event (EEP, the address of an event
//! register) to a task (TEP) and a second one (FORK.TEP), when enabled in
//! CHEN. Channels 0-19 are programmable, 20-31 pre-programmed (their FORK
//! stays programmable). Six groups (CHG) of channels are enabled or
//! disabled together by the TASKS_CHG[n].EN and DIS tasks.

const CHEN: u32 = 0x500;
const CHENSET: u32 = 0x504;
const CHENCLR: u32 = 0x508;
const CH: u32 = 0x510;
const CHG: u32 = 0x800;
const FORK: u32 = 0x910;

const PROGRAMMABLE: usize = 20;
const GROUPS: usize = 6;

/// The pre-programmed channels 20-31: (EEP, TEP).
const FIXED: [(u32, u32); 12] = [
    (0x4000_8140, 0x4000_1000), // TIMER0 COMPARE[0] -> RADIO TXEN
    (0x4000_8140, 0x4000_1004), // TIMER0 COMPARE[0] -> RADIO RXEN
    (0x4000_8144, 0x4000_1010), // TIMER0 COMPARE[1] -> RADIO DISABLE
    (0x4000_1128, 0x4000_F000), // RADIO BCMATCH -> AAR START
    (0x4000_1100, 0x4000_F000), // RADIO READY -> CCM KSGEN
    (0x4000_1104, 0x4000_F004), // RADIO ADDRESS -> CCM CRYPT
    (0x4000_1104, 0x4000_8044), // RADIO ADDRESS -> TIMER0 CAPTURE[1]
    (0x4000_110C, 0x4000_8048), // RADIO END -> TIMER0 CAPTURE[2]
    (0x4000_B140, 0x4000_1000), // RTC0 COMPARE[0] -> RADIO TXEN
    (0x4000_B140, 0x4000_1004), // RTC0 COMPARE[0] -> RADIO RXEN
    (0x4000_B140, 0x4000_800C), // RTC0 COMPARE[0] -> TIMER0 CLEAR
    (0x4000_B140, 0x4000_8000), // RTC0 COMPARE[0] -> TIMER0 START
];

pub struct Ppi {
    pub chen: u32,
    eep: [u32; PROGRAMMABLE],
    tep: [u32; PROGRAMMABLE],
    fork: [u32; 32],
    chg: [u32; GROUPS],
}

impl Ppi {
    pub fn new() -> Self {
        Self {
            chen: 0,
            eep: [0; PROGRAMMABLE],
            tep: [0; PROGRAMMABLE],
            fork: [0; 32],
            chg: [0; GROUPS],
        }
    }

    fn channel(&self, n: usize) -> (u32, u32) {
        if n < PROGRAMMABLE {
            (self.eep[n], self.tep[n])
        } else {
            FIXED[n - PROGRAMMABLE]
        }
    }

    /// Whether some enabled channel listens to an event of the peripheral
    /// at `base`.
    pub fn listens(&self, base: u32) -> bool {
        (0..32).any(|n| self.chen & (1 << n) != 0 && self.channel(n).0 & !0xFFF == base)
    }

    /// The tasks the event at address `event` triggers.
    pub fn route(&self, event: u32, tasks: &mut Vec<u32>) {
        for n in 0..32 {
            if self.chen & (1 << n) == 0 {
                continue;
            }
            let (eep, tep) = self.channel(n);
            if eep == event {
                tasks.extend([tep, self.fork[n]].into_iter().filter(|&t| t != 0));
            }
        }
    }

    pub fn read(&self, offset: u32) -> u32 {
        match offset {
            CHEN | CHENSET | CHENCLR => self.chen,
            o if (CH..CH + 8 * PROGRAMMABLE as u32).contains(&o) => {
                let n = ((o - CH) / 8) as usize;
                if o & 4 == 0 { self.eep[n] } else { self.tep[n] }
            }
            o if (CHG..CHG + 4 * GROUPS as u32).contains(&o) => self.chg[((o - CHG) / 4) as usize],
            o if (FORK..FORK + 4 * 32).contains(&o) => self.fork[((o - FORK) / 4) as usize],
            _ => 0,
        }
    }

    pub fn write(&mut self, offset: u32, value: u32) {
        match offset {
            // TASKS_CHG[n].EN and DIS
            o if o < 8 * GROUPS as u32 => {
                if value & 1 != 0 {
                    let group = self.chg[(o / 8) as usize];
                    if o & 4 == 0 {
                        self.chen |= group;
                    } else {
                        self.chen &= !group;
                    }
                }
            }
            CHEN => self.chen = value,
            CHENSET => self.chen |= value,
            CHENCLR => self.chen &= !value,
            o if (CH..CH + 8 * PROGRAMMABLE as u32).contains(&o) => {
                let n = ((o - CH) / 8) as usize;
                if o & 4 == 0 {
                    self.eep[n] = value;
                } else {
                    self.tep[n] = value;
                }
            }
            o if (CHG..CHG + 4 * GROUPS as u32).contains(&o) => {
                self.chg[((o - CHG) / 4) as usize] = value;
            }
            o if (FORK..FORK + 4 * 32).contains(&o) => self.fork[((o - FORK) / 4) as usize] = value,
            _ => {}
        }
    }
}

impl Default for Ppi {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channels_route_their_event_to_task_and_fork() {
        let mut ppi = Ppi::new();
        ppi.write(CH, 0x4000_2108); // UARTE0 RXDRDY
        ppi.write(CH + 4, 0x4000_900C); // TIMER1 CLEAR
        ppi.write(FORK, 0x4000_9000); // TIMER1 START
        let mut tasks = Vec::new();
        ppi.route(0x4000_2108, &mut tasks);
        assert!(tasks.is_empty(), "channel 0 disabled");
        ppi.write(CHENSET, 1);
        ppi.route(0x4000_2108, &mut tasks);
        assert_eq!(tasks, [0x4000_900C, 0x4000_9000]);
        assert!(ppi.listens(0x4000_2000));
        // groups
        ppi.write(CHG + 4, 1 | (1 << 30));
        ppi.write(0x00C, 1); // TASKS_CHG[1].DIS
        assert_eq!(ppi.chen, 0);
        ppi.write(0x008, 1); // TASKS_CHG[1].EN
        assert_eq!(ppi.chen, 1 | (1 << 30));
        tasks.clear();
        ppi.route(0x4000_B140, &mut tasks);
        assert_eq!(tasks, [0x4000_800C], "pre-programmed channel 30");
    }
}
