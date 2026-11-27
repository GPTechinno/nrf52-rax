//! A SoftDevice (Nordic's BLE stack, S132/S140 7.x) modelled at its
//! interface: the supervisor calls an application makes (`svc #n`, the
//! `sd_*` functions), answered here, and the events it reads back with
//! `sd_ble_evt_get` after the stack pends SWI2/EGU2, as the real one does.
//! No radio, link layer or ATT bearer runs: the board emulator plays the
//! device at the other end of the connection through the methods below,
//! either a central that connects to the firmware's advertising and writes
//! its characteristics ([`SoftDevice::central_connect`],
//! [`SoftDevice::central_write`], notifications in [`SoftDevice::host`]) or
//! a peripheral with its GATT table ([`Peer`]) that the firmware connects
//! to, discovers, writes and receives notifications from
//! ([`SoftDevice::peer_notify`]).
//!
//! One connection, handle 0. Calls a firmware makes that change nothing the
//! model keeps (configuration, connection parameters, PHY, data length, TX
//! power) succeed. The SoC calls act on the peripheral models: PPI, POWER,
//! CLOCK, flash through the NVMC. Not modelled: security (pairing,
//! bonding, encryption), indications, L2CAP channels, scanning reports,
//! the radio timeslot API, `sd_ecb_block_encrypt`; such calls return
//! `NRF_ERROR_NOT_SUPPORTED` and are noted once in [`SoftDevice::notes`].
//!
//! Structure layouts are those of the S140 7.x headers (32-bit ARM).

use std::collections::VecDeque;

/// What a call needs from the machine: the argument registers and memory.
pub trait Guest {
    fn reg(&self, n: u8) -> u32;
    fn set_reg(&mut self, n: u8, value: u32);
    fn read(&self, address: u32, len: usize) -> Option<Vec<u8>>;
    fn write(&mut self, address: u32, data: &[u8]) -> bool;
}

fn bytes(g: &dyn Guest, address: u32, len: usize) -> Vec<u8> {
    if address == 0 {
        return vec![0; len];
    }
    g.read(address, len).unwrap_or_else(|| vec![0; len])
}

fn rd8(g: &dyn Guest, address: u32) -> u8 {
    bytes(g, address, 1)[0]
}

fn rd16(g: &dyn Guest, address: u32) -> u16 {
    let b = bytes(g, address, 2);
    u16::from_le_bytes([b[0], b[1]])
}

fn rd32(g: &dyn Guest, address: u32) -> u32 {
    let b = bytes(g, address, 4);
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn wr16(g: &mut dyn Guest, address: u32, v: u16) {
    if address != 0 {
        g.write(address, &v.to_le_bytes());
    }
}

fn wr32(g: &mut dyn Guest, address: u32, v: u32) {
    if address != 0 {
        g.write(address, &v.to_le_bytes());
    }
}

fn put16(b: &mut [u8], at: usize, v: u16) {
    b[at..at + 2].copy_from_slice(&v.to_le_bytes());
}

pub const NRF_SUCCESS: u32 = 0;
pub const NRF_ERROR_NO_MEM: u32 = 4;
pub const NRF_ERROR_NOT_FOUND: u32 = 5;
pub const NRF_ERROR_NOT_SUPPORTED: u32 = 6;
pub const NRF_ERROR_INVALID_PARAM: u32 = 7;
pub const NRF_ERROR_INVALID_STATE: u32 = 8;
pub const NRF_ERROR_DATA_SIZE: u32 = 12;
pub const NRF_ERROR_SOC_MUTEX_ALREADY_TAKEN: u32 = 0x2000;
pub const BLE_ERROR_INVALID_CONN_HANDLE: u32 = 0x3002;
pub const BLE_ERROR_GATTS_INVALID_ATTR_TYPE: u32 = 0x3400;

/// The interrupt the stack pends when it has an event: SWI2/EGU2.
pub const EVENT_IRQ: u16 = 22;

/// `svc #0` is not the SoftDevice's: the MBR passes it to the bootloader,
/// whose SVCI handler dispatches on r12 (the SDK's `nrf_svci`, used by the
/// buttonless DFU service to set the bootloader's advertising name and peer
/// data, and by their `_init` calls at start-up).
const NRF_SVCI_SVC_NUM: u32 = 0;
const SD_SOFTDEVICE_ENABLE: u32 = 16;
const SD_SOFTDEVICE_DISABLE: u32 = 17;
const SD_SOFTDEVICE_IS_ENABLED: u32 = 18;
const SD_SOFTDEVICE_VECTOR_TABLE_BASE_SET: u32 = 19;
const SOC_FIRST: u32 = 32;
const SOC_LAST: u32 = 80;
const BLE_FIRST: u32 = 96;
const BLE_LAST: u32 = 191;

// Event identifiers.
const BLE_GAP_EVT_CONNECTED: u16 = 16;
const BLE_GAP_EVT_DISCONNECTED: u16 = 17;
const BLE_GAP_EVT_TIMEOUT: u16 = 27;
const BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP: u16 = 48;
const BLE_GATTC_EVT_CHAR_DISC_RSP: u16 = 50;
const BLE_GATTC_EVT_DESC_DISC_RSP: u16 = 51;
const BLE_GATTC_EVT_READ_RSP: u16 = 54;
const BLE_GATTC_EVT_WRITE_RSP: u16 = 56;
const BLE_GATTC_EVT_HVX: u16 = 57;
const BLE_GATTC_EVT_EXCHANGE_MTU_RSP: u16 = 58;
const BLE_GATTS_EVT_WRITE: u16 = 80;
const BLE_GATTS_EVT_HVN_TX_COMPLETE: u16 = 87;
const NRF_EVT_FLASH_OPERATION_SUCCESS: u32 = 2;

const BLE_GAP_ROLE_PERIPH: u8 = 1;
const BLE_GAP_ROLE_CENTRAL: u8 = 2;
const BLE_GAP_TIMEOUT_SRC_CONN: u8 = 2;
const BLE_GAP_ADDR_TYPE_RANDOM_STATIC: u8 = 1;
const BLE_GATT_OP_WRITE_REQ: u8 = 1;
const BLE_GATT_OP_WRITE_CMD: u8 = 2;
const BLE_GATT_HVX_NOTIFICATION: u8 = 1;
const BLE_GATT_STATUS_SUCCESS: u16 = 0;
const BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND: u16 = 0x010A;
const BLE_GATT_STATUS_ATTERR_INVALID_HANDLE: u16 = 0x0101;
const BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION: u8 = 0x16;
const BLE_UUID_TYPE_UNKNOWN: u8 = 0;
const BLE_UUID_TYPE_BLE: u8 = 1;
const BLE_UUID_TYPE_VENDOR_BEGIN: u8 = 2;
/// Characteristic properties.
pub const PROP_READ: u8 = 1 << 1;
pub const PROP_WRITE_WITHOUT_RESPONSE: u8 = 1 << 2;
pub const PROP_WRITE: u8 = 1 << 3;
pub const PROP_NOTIFY: u8 = 1 << 4;
pub const PROP_INDICATE: u8 = 1 << 5;

const UUID_PRIMARY_SERVICE: u16 = 0x2800;
const UUID_CHARACTERISTIC: u16 = 0x2803;
const UUID_CCCD: u16 = 0x2902;

/// The first handle of the application's attributes, after the stack's
/// GAP and GATT services.
const FIRST_USER_HANDLE: u16 = 12;
/// The ATT_MTU this side of a connection receives.
const SERVER_RX_MTU: u16 = 247;
/// What the board's end sends reaches the firmware at the next connection
/// event, this long after it is sent (picoseconds): never in the batch that
/// carried the connection itself.
const LINK_DELAY: u64 = 15_000_000_000;

/// A UUID: a Bluetooth SIG 16-bit one, or a vendor 128-bit one in the
/// order it travels in (little-endian).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Uuid {
    Sig(u16),
    Vendor([u8; 16]),
}

impl Uuid {
    /// A 128-bit UUID as written, `6e400001-b5a3-...` read as one number.
    pub const fn from_u128(v: u128) -> Self {
        Uuid::Vendor(v.to_le_bytes())
    }
}

/// The base of a vendor UUID: its bytes with the 16-bit alias (12, 13)
/// cleared.
fn base(uuid: &[u8; 16]) -> [u8; 16] {
    let mut b = *uuid;
    b[12] = 0;
    b[13] = 0;
    b
}

/// An event the board sees.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HostEvent {
    /// A connection, the firmware in the central role or not.
    Connected { central: bool },
    /// The connection ended, from either side.
    Disconnected,
    /// The firmware's GATT server notified a value (peripheral role).
    Notified { handle: u16, data: Vec<u8> },
    /// The firmware wrote a value of the peer (central role): with or
    /// without response.
    Written {
        handle: u16,
        data: Vec<u8>,
        command: bool,
    },
}

/// The firmware's advertising set.
#[derive(Clone, Debug, Default)]
pub struct Advertising {
    pub connectable: bool,
    pub data: Vec<u8>,
    pub scan_response: Vec<u8>,
}

#[derive(Clone, Copy, Debug)]
struct Connection {
    /// The firmware is the central.
    central: bool,
}

/// A connection the firmware is trying to make.
#[derive(Clone, Debug)]
struct Connect {
    targets: Vec<[u8; 6]>,
    params: [u8; 8],
    /// Picoseconds; `None` scans until cancelled.
    deadline: Option<u64>,
    /// The scan timeout in 10 ms units, turned into a deadline at the next
    /// poll.
    timeout: u16,
}

#[derive(Clone, Debug)]
enum Kind {
    Service,
    Declaration,
    Value { cccd: Option<u16> },
    Cccd,
    Descriptor,
}

/// An attribute of the firmware's GATT server.
#[derive(Clone, Debug)]
struct Attr {
    handle: u16,
    uuid: Uuid,
    kind: Kind,
    value: Vec<u8>,
}

#[derive(Clone, Debug)]
enum PeerKind {
    Service { end: u16 },
    Characteristic { props: u8, value: u16 },
    Value,
    Cccd,
}

#[derive(Clone, Debug)]
struct PeerAttr {
    handle: u16,
    uuid: Uuid,
    kind: PeerKind,
    value: Vec<u8>,
}

/// The device the firmware connects to as a central: its address and its
/// GATT table. It is reachable while `present` is set; with no address it
/// answers to the first one the firmware looks for.
#[derive(Clone, Debug)]
pub struct Peer {
    pub address: Option<[u8; 6]>,
    pub present: bool,
    attrs: Vec<PeerAttr>,
    next: u16,
}

impl Peer {
    pub fn new(address: Option<[u8; 6]>) -> Self {
        Self {
            address,
            present: false,
            attrs: Vec::new(),
            next: 0x10,
        }
    }

    /// A primary service with its characteristics (UUID, properties); a
    /// characteristic that notifies or indicates gets its CCCD.
    pub fn service(mut self, uuid: Uuid, chars: &[(Uuid, u8)]) -> Self {
        let start = self.next;
        let index = self.attrs.len();
        self.attrs.push(PeerAttr {
            handle: start,
            uuid,
            kind: PeerKind::Service { end: start },
            value: Vec::new(),
        });
        let mut h = start + 1;
        for &(uuid, props) in chars {
            self.attrs.push(PeerAttr {
                handle: h,
                uuid,
                kind: PeerKind::Characteristic {
                    props,
                    value: h + 1,
                },
                value: Vec::new(),
            });
            self.attrs.push(PeerAttr {
                handle: h + 1,
                uuid,
                kind: PeerKind::Value,
                value: Vec::new(),
            });
            h += 2;
            if props & (PROP_NOTIFY | PROP_INDICATE) != 0 {
                self.attrs.push(PeerAttr {
                    handle: h,
                    uuid: Uuid::Sig(UUID_CCCD),
                    kind: PeerKind::Cccd,
                    value: vec![0, 0],
                });
                h += 1;
            }
        }
        self.attrs[index].kind = PeerKind::Service { end: h - 1 };
        self.next = h;
        self
    }

    /// The value handle of a characteristic.
    pub fn value_handle(&self, uuid: Uuid) -> Option<u16> {
        self.attrs.iter().find_map(|a| match a.kind {
            PeerKind::Characteristic { value, .. } if a.uuid == uuid => Some(value),
            _ => None,
        })
    }

    /// The CCCD of a characteristic.
    pub fn cccd_handle(&self, uuid: Uuid) -> Option<u16> {
        let value = self.value_handle(uuid)?;
        self.attrs
            .iter()
            .find(|a| a.handle == value + 1 && matches!(a.kind, PeerKind::Cccd))
            .map(|a| a.handle)
    }

    /// Whether the firmware asked for notifications of `value_handle`.
    pub fn subscribed(&self, value_handle: u16) -> bool {
        self.attrs.iter().any(|a| {
            a.handle == value_handle + 1
                && matches!(a.kind, PeerKind::Cccd)
                && a.value.first().is_some_and(|v| v & 1 != 0)
        })
    }
}

pub struct SoftDevice {
    enabled: bool,
    events: VecDeque<Vec<u8>>,
    /// Events from the board's end of the link, with the time they arrive.
    arriving: VecDeque<(u64, Vec<u8>)>,
    /// The time of the last call or poll.
    now: u64,
    soc_events: VecDeque<u32>,
    /// An event was queued since the machine last looked.
    irq: bool,
    bases: Vec<[u8; 16]>,
    attrs: Vec<Attr>,
    next_handle: u16,
    address: Option<[u8; 6]>,
    adv: Advertising,
    advertising: bool,
    conn: Option<Connection>,
    connect: Option<Connect>,
    whitelist: Vec<[u8; 6]>,
    /// The size of the buffer the firmware last offered `sd_ble_evt_get`.
    capacity: usize,
    rng: u64,
    /// The device the firmware connects to as a central, if the board
    /// plays one.
    pub peer: Option<Peer>,
    /// What happened on the connection, for the board.
    pub host: VecDeque<HostEvent>,
    /// Calls not modelled, once each.
    pub notes: Vec<String>,
    /// The die temperature `sd_temp_get` reports, in 0.25 °C.
    pub quarter_degrees: i32,
}

impl Default for SoftDevice {
    fn default() -> Self {
        Self::new()
    }
}

impl SoftDevice {
    pub fn new() -> Self {
        Self {
            enabled: false,
            events: VecDeque::new(),
            arriving: VecDeque::new(),
            now: 0,
            soc_events: VecDeque::new(),
            irq: false,
            bases: Vec::new(),
            attrs: Vec::new(),
            next_handle: FIRST_USER_HANDLE,
            address: None,
            adv: Advertising::default(),
            advertising: false,
            conn: None,
            connect: None,
            whitelist: Vec::new(),
            capacity: 0,
            rng: 0x5EED_0001,
            peer: None,
            host: VecDeque::new(),
            notes: Vec::new(),
            quarter_degrees: 100,
        }
    }

    /// The stack after a chip reset: what the firmware set up is gone, the
    /// board's peer stays (and sees the connection drop).
    pub fn after_reset(self) -> Self {
        let mut fresh = Self::new();
        fresh.peer = self.peer;
        fresh.host = self.host;
        fresh.notes = self.notes;
        fresh.quarter_degrees = self.quarter_degrees;
        if self.conn.is_some() {
            fresh.host.push_back(HostEvent::Disconnected);
        }
        fresh
    }

    /// Whether an event was queued since the last call: the machine then
    /// pends [`EVENT_IRQ`].
    pub fn take_irq(&mut self) -> bool {
        std::mem::take(&mut self.irq)
    }

    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn connected(&self) -> bool {
        self.conn.is_some()
    }

    /// The firmware's advertising set while it advertises.
    pub fn advertising(&self) -> Option<&Advertising> {
        self.advertising.then_some(&self.adv)
    }

    /// The largest value a notification to the firmware (central role)
    /// may carry, so that its event fits the buffer it reads events into.
    pub fn max_notification(&self) -> usize {
        self.capacity.max(116) - 16
    }

    /// The largest value a write to the firmware (peripheral role) may
    /// carry, so that its event fits the buffer it reads events into.
    pub fn max_write(&self) -> usize {
        self.capacity.max(118) - 18
    }

    fn note(&mut self, s: String) {
        if !self.notes.contains(&s) {
            self.notes.push(s);
        }
    }

    fn push(&mut self, event: Vec<u8>) {
        self.events.push_back(event);
        self.irq = true;
    }

    /// An event the board's end of the link causes: it arrives at the next
    /// connection event, after those already on their way.
    fn arrive(&mut self, event: Vec<u8>) {
        let due = (self.now + LINK_DELAY).max(self.arriving.back().map_or(0, |a| a.0));
        self.arriving.push_back((due, event));
    }

    /// When the next event from the board's end arrives, for the machine to
    /// wake for it.
    pub fn next_arrival(&self) -> Option<u64> {
        self.arriving.front().map(|a| a.0)
    }

    /// An event of `size` bytes, header filled.
    fn event(id: u16, size: usize) -> Vec<u8> {
        let mut e = vec![0; size];
        put16(&mut e, 0, id);
        put16(&mut e, 2, size as u16);
        e
    }

    fn raw_uuid(&self, uuid: Uuid) -> (u16, u8) {
        match uuid {
            Uuid::Sig(u) => (u, BLE_UUID_TYPE_BLE),
            Uuid::Vendor(b) => {
                let alias = u16::from_le_bytes([b[12], b[13]]);
                match self.bases.iter().position(|x| *x == base(&b)) {
                    Some(i) => (alias, BLE_UUID_TYPE_VENDOR_BEGIN + i as u8),
                    None => (alias, BLE_UUID_TYPE_UNKNOWN),
                }
            }
        }
    }

    fn uuid_of(&self, uuid16: u16, kind: u8) -> Option<Uuid> {
        match kind {
            BLE_UUID_TYPE_BLE => Some(Uuid::Sig(uuid16)),
            k if k >= BLE_UUID_TYPE_VENDOR_BEGIN => {
                let mut b = *self.bases.get((k - BLE_UUID_TYPE_VENDOR_BEGIN) as usize)?;
                b[12..14].copy_from_slice(&uuid16.to_le_bytes());
                Some(Uuid::Vendor(b))
            }
            _ => None,
        }
    }

    /// A `ble_uuid_t` in memory.
    fn read_uuid(&self, g: &dyn Guest, address: u32) -> Option<Uuid> {
        self.uuid_of(rd16(g, address), rd8(g, address + 2))
    }

    fn put_uuid(&self, e: &mut [u8], at: usize, uuid: Uuid) {
        let (u, t) = self.raw_uuid(uuid);
        put16(e, at, u);
        e[at + 2] = t;
    }

    /// Answer supervisor call `svc`. Returns false when it is not a
    /// SoftDevice call (the core then carries on, as without one).
    pub fn call(&mut self, svc: u32, g: &mut dyn Guest, now: u64) -> bool {
        self.now = self.now.max(now);
        let r = match svc {
            SD_SOFTDEVICE_ENABLE => {
                self.enabled = true;
                if self.address.is_none() {
                    // FICR.DEVICEADDR, a random static address
                    let low = rd32(g, 0x1000_00A4);
                    let high = rd32(g, 0x1000_00A8) as u16 | 0xC000;
                    let mut a = [0; 6];
                    a[..4].copy_from_slice(&low.to_le_bytes());
                    a[4..].copy_from_slice(&high.to_le_bytes());
                    self.address = Some(a);
                }
                NRF_SUCCESS
            }
            SD_SOFTDEVICE_DISABLE => {
                self.enabled = false;
                NRF_SUCCESS
            }
            SD_SOFTDEVICE_IS_ENABLED => {
                g.write(g.reg(0), &[self.enabled as u8]);
                NRF_SUCCESS
            }
            SD_SOFTDEVICE_VECTOR_TABLE_BASE_SET => NRF_SUCCESS,
            // Answered as a bootloader that accepts every SVCI call; nothing
            // runs, so a DFU it would start never starts.
            NRF_SVCI_SVC_NUM => {
                let n = g.reg(12);
                self.note(format!("SVCI call {n:#x} answered as a bootloader would"));
                NRF_SUCCESS
            }
            SOC_FIRST..=SOC_LAST => self.soc(svc, g),
            BLE_FIRST..=BLE_LAST => self.ble(svc, g, now),
            _ => return false,
        };
        g.set_reg(0, r);
        true
    }

    fn unsupported(&mut self, what: &str) -> u32 {
        self.note(format!("SoftDevice call {what} not modelled"));
        NRF_ERROR_NOT_SUPPORTED
    }

    fn soc(&mut self, svc: u32, g: &mut dyn Guest) -> u32 {
        const PPI: u32 = 0x4001_F000;
        const POWER: u32 = 0x4000_0000;
        const NVMC: u32 = 0x4001_E000;
        let (a0, a1, a2) = (g.reg(0), g.reg(1), g.reg(2));
        match svc {
            32 => {
                let v = rd32(g, PPI + 0x500);
                wr32(g, a0, v);
            }
            33 => wr32(g, PPI + 0x504, a0),
            34 => wr32(g, PPI + 0x508, a0),
            35 => {
                wr32(g, PPI + 0x510 + 8 * a0, a1);
                wr32(g, PPI + 0x514 + 8 * a0, a2);
            }
            36 => wr32(g, PPI + 8 * a0, 1),
            37 => wr32(g, PPI + 4 + 8 * a0, 1),
            38 => wr32(g, PPI + 0x800 + 4 * a0, a1),
            39 => {
                let v = rd32(g, PPI + 0x800 + 4 * a0);
                wr32(g, a1, v);
            }
            // flash, through the NVMC; the result is a SoC event
            40 => {
                wr32(g, NVMC + 0x504, 2);
                wr32(g, NVMC + 0x508, a0 * 4096);
                wr32(g, NVMC + 0x504, 0);
                self.soc_events.push_back(NRF_EVT_FLASH_OPERATION_SUCCESS);
                self.irq = true;
            }
            41 => {
                let data = bytes(g, a1, 4 * a2 as usize);
                wr32(g, NVMC + 0x504, 1);
                g.write(a0, &data);
                wr32(g, NVMC + 0x504, 0);
                self.soc_events.push_back(NRF_EVT_FLASH_OPERATION_SUCCESS);
                self.irq = true;
            }
            43 => wr32(g, a0, a1),
            44 => {
                g.write(a0, &[0]);
            }
            45 => {
                if rd8(g, a0) != 0 {
                    return NRF_ERROR_SOC_MUTEX_ALREADY_TAKEN;
                }
                g.write(a0, &[1]);
            }
            46 => {
                g.write(a0, &[0]);
            }
            47 | 48 => {
                g.write(a0, &[64]);
            }
            49 => {
                let mut out = Vec::with_capacity(a1 as usize);
                for _ in 0..a1 {
                    self.rng ^= self.rng << 13;
                    self.rng ^= self.rng >> 7;
                    self.rng ^= self.rng << 17;
                    out.push((self.rng >> 24) as u8);
                }
                g.write(a0, &out);
            }
            50 | 54..=56 | 63 | 64 | 69 | 77..=79 => {}
            51 => wr32(g, POWER + 0x500, 1),
            52 => {
                let v = rd32(g, POWER + 0x400);
                wr32(g, a0, v);
            }
            53 => wr32(g, POWER + 0x400, a0),
            57 | 58 => {}
            59 => wr32(g, a1, 0),
            80 => wr32(g, a0, 0),
            60..=62 => {
                let register = POWER + if a0 == 0 { 0x51C } else { 0x520 };
                let v = rd32(g, register);
                match svc {
                    60 => wr32(g, register, v | a1),
                    61 => wr32(g, register, v & !a1),
                    _ => wr32(g, a1, v),
                }
            }
            65 => {}
            66 => wr32(g, POWER, 1),
            67 => wr32(g, POWER + 4, 1),
            68 => {
                let stat = rd32(g, POWER + 0x40C);
                wr32(g, a0, (stat >> 16) & stat & 1);
            }
            75 => match self.soc_events.pop_front() {
                Some(e) => wr32(g, a0, e),
                None => return NRF_ERROR_NOT_FOUND,
            },
            76 => wr32(g, a0, self.quarter_degrees as u32),
            70 => return self.unsupported("sd_ecb_block_encrypt"),
            71 => return self.unsupported("sd_ecb_blocks_encrypt"),
            72..=74 => return self.unsupported("the radio timeslot API"),
            _ => return self.unsupported(&format!("SVC {svc}")),
        }
        NRF_SUCCESS
    }

    fn ble(&mut self, svc: u32, g: &mut dyn Guest, now: u64) -> u32 {
        let (a0, a1, a2, a3) = (g.reg(0), g.reg(1), g.reg(2), g.reg(3));
        match svc {
            // sd_ble_enable, cfg_set, opt_set, user_mem_reply, uuid_vs_remove
            96 | 102 | 103 | 105 | 106 => NRF_SUCCESS,
            97 => self.evt_get(g, a0, a1),
            98 => {
                let b: [u8; 16] = bytes(g, a0, 16).try_into().unwrap();
                let b = base(&b);
                let i = match self.bases.iter().position(|x| *x == b) {
                    Some(i) => i,
                    None => {
                        self.bases.push(b);
                        self.bases.len() - 1
                    }
                };
                g.write(a1, &[BLE_UUID_TYPE_VENDOR_BEGIN + i as u8]);
                NRF_SUCCESS
            }
            99 => {
                // sd_ble_uuid_decode(len, p_uuid_le, p_uuid)
                let raw = bytes(g, a1, a0 as usize);
                let uuid = match a0 {
                    2 => Uuid::Sig(u16::from_le_bytes([raw[0], raw[1]])),
                    16 => Uuid::Vendor(raw.try_into().unwrap()),
                    _ => return NRF_ERROR_INVALID_PARAM,
                };
                let (u, t) = self.raw_uuid(uuid);
                let mut out = [0u8; 4];
                put16(&mut out, 0, u);
                out[2] = t;
                g.write(a2, &out[..3]);
                if t == BLE_UUID_TYPE_UNKNOWN {
                    NRF_ERROR_NOT_FOUND
                } else {
                    NRF_SUCCESS
                }
            }
            100 => {
                // sd_ble_uuid_encode(p_uuid, p_uuid_le_len, p_uuid_le)
                match self.read_uuid(g, a0) {
                    Some(Uuid::Sig(u)) => {
                        g.write(a1, &[2]);
                        g.write(a2, &u.to_le_bytes());
                        NRF_SUCCESS
                    }
                    Some(Uuid::Vendor(b)) => {
                        g.write(a1, &[16]);
                        g.write(a2, &b);
                        NRF_SUCCESS
                    }
                    None => NRF_ERROR_INVALID_PARAM,
                }
            }
            101 => {
                let mut v = [0u8; 6];
                put16(&mut v, 0, 0x0B);
                put16(&mut v, 2, 0x0059);
                put16(&mut v, 4, 0x0100);
                g.write(a0, &v);
                NRF_SUCCESS
            }
            104 => self.unsupported("sd_ble_opt_get"),
            108 => {
                let b = bytes(g, a0, 7);
                self.address = Some(b[1..7].try_into().unwrap());
                NRF_SUCCESS
            }
            109 => {
                let mut b = [0u8; 7];
                b[0] = BLE_GAP_ADDR_TYPE_RANDOM_STATIC << 1;
                b[1..].copy_from_slice(&self.address.unwrap_or([0x01, 0, 0, 0, 0, 0xC0]));
                g.write(a0, &b);
                NRF_SUCCESS
            }
            110 => {
                self.whitelist = (0..a1)
                    .map(|i| {
                        let p = rd32(g, a0 + 4 * i);
                        bytes(g, p + 1, 6).try_into().unwrap()
                    })
                    .collect();
                NRF_SUCCESS
            }
            111 | 112 => NRF_SUCCESS,
            113 => self.unsupported("sd_ble_gap_privacy_get"),
            114 => self.adv_set_configure(g, a0, a1, a2),
            115 => {
                self.advertising = true;
                NRF_SUCCESS
            }
            116 => {
                if !std::mem::take(&mut self.advertising) {
                    return NRF_ERROR_INVALID_STATE;
                }
                NRF_SUCCESS
            }
            // conn_param_update, tx_power_set, appearance_set, ppcp_set,
            // device_name_set
            117 | 119 | 120 | 122 | 124 => NRF_SUCCESS,
            118 => {
                if a0 != 0 || self.conn.is_none() {
                    return BLE_ERROR_INVALID_CONN_HANDLE;
                }
                self.drop_connection(BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION, false);
                NRF_SUCCESS
            }
            121 => {
                wr16(g, a0, 0);
                NRF_SUCCESS
            }
            123 => {
                g.write(a0, &[0; 8]);
                NRF_SUCCESS
            }
            125 => self.unsupported("sd_ble_gap_device_name_get"),
            126..=135 => self.unsupported("security (pairing, bonding, encryption)"),
            // rssi_start, rssi_stop, scan_start, scan_stop
            136..=139 => NRF_SUCCESS,
            140 => {
                if self.conn.is_some() || self.connect.is_some() {
                    return NRF_ERROR_INVALID_STATE;
                }
                let targets = if a0 != 0 {
                    vec![bytes(g, a0 + 1, 6).try_into().unwrap()]
                } else {
                    self.whitelist.clone()
                };
                self.connect = Some(Connect {
                    targets,
                    params: bytes(g, a2, 8).try_into().unwrap(),
                    deadline: None,
                    timeout: rd16(g, a1 + 6),
                });
                self.poll(now);
                NRF_SUCCESS
            }
            141 => {
                if self.connect.take().is_none() {
                    return NRF_ERROR_INVALID_STATE;
                }
                NRF_SUCCESS
            }
            142 => self.unsupported("sd_ble_gap_rssi_get"),
            // phy_update, data_length_update, and the rest of GAP
            143..=154 => NRF_SUCCESS,
            155 => self.gattc_primary_services(g, a0, a1, a2),
            157 => self.gattc_characteristics(g, a0, a1),
            158 => self.gattc_descriptors(g, a0, a1),
            161 => self.gattc_read(a0, a1 as u16, a2 as u16),
            163 => self.gattc_write(g, a0, a1),
            164 => NRF_SUCCESS,
            165 => {
                if !self.firmware_central(a0) {
                    return BLE_ERROR_INVALID_CONN_HANDLE;
                }
                let mut e = Self::event(BLE_GATTC_EVT_EXCHANGE_MTU_RSP, 12);
                put16(&mut e, 10, SERVER_RX_MTU.min(a1 as u16).max(23));
                self.push(e);
                NRF_SUCCESS
            }
            156..=167 => self.unsupported(&format!("GATT client SVC {svc}")),
            168 => {
                let Some(uuid) = self.read_uuid(g, a1) else {
                    return NRF_ERROR_INVALID_PARAM;
                };
                let h = self.add(uuid, Kind::Service, Vec::new());
                wr16(g, a2, h);
                NRF_SUCCESS
            }
            169 => {
                let h = self.add(Uuid::Sig(0x2802), Kind::Descriptor, Vec::new());
                wr16(g, a2, h);
                NRF_SUCCESS
            }
            170 => self.gatts_characteristic(g, a1, a2, a3),
            171 => {
                let Some(uuid) = self.read_uuid(g, rd32(g, a1)) else {
                    return NRF_ERROR_INVALID_PARAM;
                };
                let value = bytes(g, rd32(g, a1 + 16), rd16(g, a1 + 8) as usize);
                let h = self.add(uuid, Kind::Descriptor, value);
                wr16(g, a2, h);
                NRF_SUCCESS
            }
            172 => {
                // sd_ble_gatts_value_set(conn, handle, p_value)
                let (len, offset, p) = (rd16(g, a2), rd16(g, a2 + 2), rd32(g, a2 + 4));
                let data = bytes(g, p, len as usize);
                let Some(attr) = self.attrs.iter_mut().find(|x| x.handle == a1 as u16) else {
                    return BLE_ERROR_GATTS_INVALID_ATTR_TYPE;
                };
                let offset = offset as usize;
                attr.value.resize(attr.value.len().max(offset), 0);
                attr.value.truncate(offset);
                attr.value.extend_from_slice(&data);
                NRF_SUCCESS
            }
            173 => {
                let Some(attr) = self.attrs.iter().find(|x| x.handle == a1 as u16) else {
                    return BLE_ERROR_GATTS_INVALID_ATTR_TYPE;
                };
                let value = attr.value.clone();
                let (len, offset, p) = (rd16(g, a2), rd16(g, a2 + 2), rd32(g, a2 + 4));
                let rest = value.get(offset as usize..).unwrap_or(&[]);
                let n = rest.len().min(len as usize);
                if p != 0 {
                    g.write(p, &rest[..n]);
                }
                wr16(g, a2, if p == 0 { value.len() as u16 } else { n as u16 });
                NRF_SUCCESS
            }
            174 => self.gatts_hvx(g, a0, a1),
            175 => self.unsupported("sd_ble_gatts_service_changed"),
            // rw_authorize_reply, sys_attr_set, exchange_mtu_reply
            176 | 177 | 181 => NRF_SUCCESS,
            178 => {
                wr16(g, a2, 0);
                NRF_SUCCESS
            }
            179 => {
                wr16(g, a0, FIRST_USER_HANDLE);
                NRF_SUCCESS
            }
            180 => self.unsupported("sd_ble_gatts_attr_get"),
            182 | 183 => self.unsupported(&format!("GATT server SVC {svc}")),
            _ => self.unsupported("L2CAP channels"),
        }
    }

    /// sd_ble_evt_get(p_dest, p_len)
    fn evt_get(&mut self, g: &mut dyn Guest, dest: u32, p_len: u32) -> u32 {
        let Some(e) = self.events.front() else {
            return NRF_ERROR_NOT_FOUND;
        };
        let len = e.len() as u16;
        let room = rd16(g, p_len);
        self.capacity = room as usize;
        if dest == 0 {
            wr16(g, p_len, len);
            return NRF_SUCCESS;
        }
        if room < len {
            wr16(g, p_len, len);
            return NRF_ERROR_DATA_SIZE;
        }
        let e = self.events.pop_front().unwrap();
        g.write(dest, &e);
        wr16(g, p_len, len);
        NRF_SUCCESS
    }

    fn adv_set_configure(
        &mut self,
        g: &mut dyn Guest,
        p_handle: u32,
        p_data: u32,
        p_params: u32,
    ) -> u32 {
        if p_handle != 0 && rd8(g, p_handle) == 0xFF {
            g.write(p_handle, &[0]);
        }
        if p_data != 0 {
            let (ad, ad_len) = (rd32(g, p_data), rd16(g, p_data + 4));
            let (sr, sr_len) = (rd32(g, p_data + 8), rd16(g, p_data + 12));
            self.adv.data = bytes(g, ad, ad_len as usize);
            self.adv.scan_response = bytes(g, sr, sr_len as usize);
        }
        if p_params != 0 {
            self.adv.connectable = matches!(rd8(g, p_params), 1 | 2 | 3 | 6 | 7);
        }
        NRF_SUCCESS
    }

    fn add(&mut self, uuid: Uuid, kind: Kind, value: Vec<u8>) -> u16 {
        let handle = self.next_handle;
        self.next_handle += 1;
        self.attrs.push(Attr {
            handle,
            uuid,
            kind,
            value,
        });
        handle
    }

    /// sd_ble_gatts_characteristic_add(service, p_char_md, p_attr, p_handles)
    fn gatts_characteristic(&mut self, g: &mut dyn Guest, md: u32, attr: u32, out: u32) -> u32 {
        let props = rd8(g, md);
        let Some(uuid) = self.read_uuid(g, rd32(g, attr)) else {
            return NRF_ERROR_INVALID_PARAM;
        };
        let (init_len, p_value) = (rd16(g, attr + 8), rd32(g, attr + 16));
        let value = bytes(g, p_value, init_len as usize);
        self.add(uuid, Kind::Declaration, vec![props]);
        let value_handle = self.add(uuid, Kind::Value { cccd: None }, value);
        let mut handles = [0u8; 8];
        put16(&mut handles, 0, value_handle);
        let user_desc = rd32(g, md + 4);
        if user_desc != 0 {
            let len = rd16(g, md + 10);
            let text = bytes(g, user_desc, len as usize);
            put16(
                &mut handles,
                2,
                self.add(Uuid::Sig(0x2901), Kind::Descriptor, text),
            );
        }
        if props & (PROP_NOTIFY | PROP_INDICATE) != 0 {
            let cccd = self.add(Uuid::Sig(UUID_CCCD), Kind::Cccd, vec![0, 0]);
            put16(&mut handles, 4, cccd);
            if let Some(a) = self.attrs.iter_mut().find(|a| a.handle == value_handle) {
                a.kind = Kind::Value { cccd: Some(cccd) };
            }
        }
        g.write(out, &handles);
        NRF_SUCCESS
    }

    /// sd_ble_gatts_hvx(conn, p_hvx_params)
    fn gatts_hvx(&mut self, g: &mut dyn Guest, conn: u32, params: u32) -> u32 {
        if conn != 0 || self.conn.is_none_or(|c| c.central) {
            return BLE_ERROR_INVALID_CONN_HANDLE;
        }
        let (handle, kind) = (rd16(g, params), rd8(g, params + 2));
        let (p_len, p_data) = (rd32(g, params + 8), rd32(g, params + 12));
        let len = rd16(g, p_len);
        let data = bytes(g, p_data, len as usize);
        let Some(index) = self.attrs.iter().position(|a| a.handle == handle) else {
            return BLE_ERROR_GATTS_INVALID_ATTR_TYPE;
        };
        let Kind::Value { cccd: Some(cccd) } = self.attrs[index].kind else {
            return NRF_ERROR_INVALID_STATE;
        };
        if kind != BLE_GATT_HVX_NOTIFICATION {
            return self.unsupported("indications");
        }
        let on = self
            .attrs
            .iter()
            .find(|a| a.handle == cccd)
            .is_some_and(|a| a.value.first().is_some_and(|v| v & 1 != 0));
        if !on {
            return NRF_ERROR_INVALID_STATE;
        }
        self.attrs[index].value = data.clone();
        wr16(g, p_len, len);
        self.host.push_back(HostEvent::Notified { handle, data });
        let mut e = Self::event(BLE_GATTS_EVT_HVN_TX_COMPLETE, 8);
        e[6] = 1;
        self.push(e);
        NRF_SUCCESS
    }

    fn firmware_central(&self, conn: u32) -> bool {
        conn == 0 && self.conn.is_some_and(|c| c.central)
    }

    fn gattc_event(id: u16, status: u16, size: usize) -> Vec<u8> {
        let mut e = Self::event(id, size);
        put16(&mut e, 6, status);
        e
    }

    /// sd_ble_gattc_primary_services_discover(conn, start_handle, p_uuid)
    fn gattc_primary_services(
        &mut self,
        g: &mut dyn Guest,
        conn: u32,
        start: u32,
        p_uuid: u32,
    ) -> u32 {
        if !self.firmware_central(conn) {
            return BLE_ERROR_INVALID_CONN_HANDLE;
        }
        let wanted = self.read_uuid(g, p_uuid);
        let peer = self.peer.as_ref().unwrap();
        let found = peer.attrs.iter().find_map(|a| match a.kind {
            PeerKind::Service { end } if a.handle >= start as u16 && Some(a.uuid) == wanted => {
                Some((a.uuid, a.handle, end))
            }
            _ => None,
        });
        let e = match found {
            Some((uuid, first, end)) => {
                let mut e = Self::gattc_event(
                    BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP,
                    BLE_GATT_STATUS_SUCCESS,
                    20,
                );
                put16(&mut e, 10, 1);
                self.put_uuid(&mut e, 12, uuid);
                put16(&mut e, 16, first);
                put16(&mut e, 18, end);
                e
            }
            None => Self::gattc_event(
                BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP,
                BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND,
                12,
            ),
        };
        self.push(e);
        NRF_SUCCESS
    }

    /// sd_ble_gattc_characteristics_discover(conn, p_handle_range)
    fn gattc_characteristics(&mut self, g: &mut dyn Guest, conn: u32, range: u32) -> u32 {
        if !self.firmware_central(conn) {
            return BLE_ERROR_INVALID_CONN_HANDLE;
        }
        let (start, end) = (rd16(g, range), rd16(g, range + 2));
        let peer = self.peer.as_ref().unwrap();
        let chars: Vec<(Uuid, u8, u16, u16)> = peer
            .attrs
            .iter()
            .filter(|a| (start..=end).contains(&a.handle))
            .filter_map(|a| match a.kind {
                PeerKind::Characteristic { props, value } => Some((a.uuid, props, a.handle, value)),
                _ => None,
            })
            .collect();
        let e = if chars.is_empty() {
            Self::gattc_event(
                BLE_GATTC_EVT_CHAR_DISC_RSP,
                BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND,
                12,
            )
        } else {
            let mut e = Self::gattc_event(
                BLE_GATTC_EVT_CHAR_DISC_RSP,
                BLE_GATT_STATUS_SUCCESS,
                12 + 10 * chars.len(),
            );
            put16(&mut e, 10, chars.len() as u16);
            for (i, (uuid, props, decl, value)) in chars.into_iter().enumerate() {
                let at = 12 + 10 * i;
                self.put_uuid(&mut e, at, uuid);
                e[at + 4] = props;
                put16(&mut e, at + 6, decl);
                put16(&mut e, at + 8, value);
            }
            e
        };
        self.push(e);
        NRF_SUCCESS
    }

    /// sd_ble_gattc_descriptors_discover(conn, p_handle_range): every
    /// attribute in the range.
    fn gattc_descriptors(&mut self, g: &mut dyn Guest, conn: u32, range: u32) -> u32 {
        if !self.firmware_central(conn) {
            return BLE_ERROR_INVALID_CONN_HANDLE;
        }
        let (start, end) = (rd16(g, range), rd16(g, range + 2));
        let peer = self.peer.as_ref().unwrap();
        let descs: Vec<(u16, Uuid)> = peer
            .attrs
            .iter()
            .filter(|a| (start..=end).contains(&a.handle))
            .map(|a| {
                let uuid = match a.kind {
                    PeerKind::Service { .. } => Uuid::Sig(UUID_PRIMARY_SERVICE),
                    PeerKind::Characteristic { .. } => Uuid::Sig(UUID_CHARACTERISTIC),
                    _ => a.uuid,
                };
                (a.handle, uuid)
            })
            .collect();
        let e = if descs.is_empty() {
            Self::gattc_event(
                BLE_GATTC_EVT_DESC_DISC_RSP,
                BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND,
                12,
            )
        } else {
            let mut e = Self::gattc_event(
                BLE_GATTC_EVT_DESC_DISC_RSP,
                BLE_GATT_STATUS_SUCCESS,
                12 + 6 * descs.len(),
            );
            put16(&mut e, 10, descs.len() as u16);
            for (i, (handle, uuid)) in descs.into_iter().enumerate() {
                let at = 12 + 6 * i;
                put16(&mut e, at, handle);
                self.put_uuid(&mut e, at + 2, uuid);
            }
            e
        };
        self.push(e);
        NRF_SUCCESS
    }

    /// sd_ble_gattc_read(conn, handle, offset)
    fn gattc_read(&mut self, conn: u32, handle: u16, offset: u16) -> u32 {
        if !self.firmware_central(conn) {
            return BLE_ERROR_INVALID_CONN_HANDLE;
        }
        let value = self
            .peer
            .as_ref()
            .and_then(|p| p.attrs.iter().find(|a| a.handle == handle))
            .map(|a| a.value.get(offset as usize..).unwrap_or(&[]).to_vec());
        let e = match value {
            Some(v) => {
                let mut e = Self::gattc_event(
                    BLE_GATTC_EVT_READ_RSP,
                    BLE_GATT_STATUS_SUCCESS,
                    16 + v.len(),
                );
                put16(&mut e, 10, handle);
                put16(&mut e, 12, offset);
                put16(&mut e, 14, v.len() as u16);
                e[16..].copy_from_slice(&v);
                e
            }
            None => Self::gattc_event(
                BLE_GATTC_EVT_READ_RSP,
                BLE_GATT_STATUS_ATTERR_INVALID_HANDLE,
                16,
            ),
        };
        self.push(e);
        NRF_SUCCESS
    }

    /// sd_ble_gattc_write(conn, p_write_params)
    fn gattc_write(&mut self, g: &mut dyn Guest, conn: u32, params: u32) -> u32 {
        if !self.firmware_central(conn) {
            return BLE_ERROR_INVALID_CONN_HANDLE;
        }
        let (op, handle) = (rd8(g, params), rd16(g, params + 2));
        let (offset, len, p) = (
            rd16(g, params + 4),
            rd16(g, params + 6),
            rd32(g, params + 8),
        );
        let data = bytes(g, p, len as usize);
        let command = match op {
            BLE_GATT_OP_WRITE_REQ => false,
            BLE_GATT_OP_WRITE_CMD => true,
            _ => return self.unsupported(&format!("GATT write operation {op}")),
        };
        let peer = self.peer.as_mut().unwrap();
        let known = match peer.attrs.iter_mut().find(|a| a.handle == handle) {
            Some(a) => {
                a.value = data.clone();
                true
            }
            None => false,
        };
        if known {
            self.host.push_back(HostEvent::Written {
                handle,
                data: data.clone(),
                command,
            });
        }
        if !command {
            let status = if known {
                BLE_GATT_STATUS_SUCCESS
            } else {
                BLE_GATT_STATUS_ATTERR_INVALID_HANDLE
            };
            let mut e = Self::gattc_event(BLE_GATTC_EVT_WRITE_RSP, status, 18 + data.len());
            put16(&mut e, 10, handle);
            e[12] = op;
            put16(&mut e, 14, offset);
            put16(&mut e, 16, data.len() as u16);
            e[18..].copy_from_slice(&data);
            self.push(e);
        }
        NRF_SUCCESS
    }

    fn connected_event(&mut self, role: u8, peer: [u8; 6], params: [u8; 8]) {
        let mut e = Self::event(BLE_GAP_EVT_CONNECTED, 44);
        put16(&mut e, 4, 0);
        e[8] = BLE_GAP_ADDR_TYPE_RANDOM_STATIC << 1;
        e[9..15].copy_from_slice(&peer);
        e[15] = role;
        e[16..24].copy_from_slice(&params);
        self.push(e);
    }

    fn drop_connection(&mut self, reason: u8, from_link: bool) {
        if self.conn.take().is_none() {
            return;
        }
        for a in &mut self.attrs {
            if matches!(a.kind, Kind::Cccd) {
                a.value = vec![0, 0];
            }
        }
        if let Some(peer) = self.peer.as_mut() {
            for a in &mut peer.attrs {
                if matches!(a.kind, PeerKind::Cccd) {
                    a.value = vec![0, 0];
                }
            }
        }
        let mut e = Self::event(BLE_GAP_EVT_DISCONNECTED, 12);
        e[8] = reason;
        if from_link {
            self.arrive(e);
        } else {
            self.arriving.clear();
            self.push(e);
        }
        self.host.push_back(HostEvent::Disconnected);
    }

    /// Advance what waits on time or on the board: a connection the
    /// firmware asked for, made once the peer is present, or timed out.
    pub fn poll(&mut self, now: u64) {
        self.now = self.now.max(now);
        while self.arriving.front().is_some_and(|a| a.0 <= now) {
            let (_, e) = self.arriving.pop_front().unwrap();
            self.push(e);
        }
        let Some(connect) = self.connect.as_mut() else {
            return;
        };
        if connect.deadline.is_none() && connect.timeout != 0 {
            connect.deadline = Some(now + u64::from(connect.timeout) * 10_000_000_000);
        }
        if self.conn.is_none()
            && let Some(peer) = self.peer.as_ref().filter(|p| p.present)
        {
            let address = match peer.address {
                Some(a) => connect.targets.contains(&a).then_some(a),
                None => connect.targets.first().copied(),
            };
            if let Some(address) = address {
                let params = connect.params;
                self.connect = None;
                self.conn = Some(Connection { central: true });
                self.connected_event(BLE_GAP_ROLE_CENTRAL, address, params);
                self.host.push_back(HostEvent::Connected { central: true });
                return;
            }
        }
        if connect.deadline.is_some_and(|d| now >= d) {
            self.connect = None;
            let mut e = Self::event(BLE_GAP_EVT_TIMEOUT, 20);
            e[8] = BLE_GAP_TIMEOUT_SRC_CONN;
            self.push(e);
        }
    }

    /// A central on the board connects to the firmware's advertising: it
    /// must be connectable. Its address is `central`, random static.
    pub fn central_connect(&mut self, central: [u8; 6]) -> bool {
        if self.conn.is_some() || !self.advertising || !self.adv.connectable {
            return false;
        }
        self.advertising = false;
        self.conn = Some(Connection { central: false });
        // 30 to 50 ms, no latency, 4 s supervision timeout
        let mut params = [0u8; 8];
        put16(&mut params, 0, 24);
        put16(&mut params, 2, 40);
        put16(&mut params, 6, 400);
        self.connected_event(BLE_GAP_ROLE_PERIPH, central, params);
        self.host.push_back(HostEvent::Connected { central: false });
        true
    }

    /// The board's central writes an attribute of the firmware's GATT
    /// server: a write command, or a write request.
    pub fn central_write(&mut self, handle: u16, data: &[u8], command: bool) -> bool {
        if self.conn.is_none_or(|c| c.central) {
            return false;
        }
        let Some(attr) = self.attrs.iter_mut().find(|a| a.handle == handle) else {
            return false;
        };
        attr.value = data.to_vec();
        let uuid = attr.uuid;
        let mut e = Self::event(BLE_GATTS_EVT_WRITE, 18 + data.len());
        put16(&mut e, 6, handle);
        self.put_uuid(&mut e, 8, uuid);
        e[12] = if command {
            BLE_GATT_OP_WRITE_CMD
        } else {
            BLE_GATT_OP_WRITE_REQ
        };
        put16(&mut e, 16, data.len() as u16);
        e[18..].copy_from_slice(data);
        self.arrive(e);
        true
    }

    /// The value handle and the CCCD of a characteristic of the firmware's
    /// GATT server.
    pub fn characteristic(&self, uuid: Uuid) -> Option<(u16, Option<u16>)> {
        self.attrs.iter().find_map(|a| match a.kind {
            Kind::Value { cccd } if a.uuid == uuid => Some((a.handle, cccd)),
            _ => None,
        })
    }

    /// The peer (central role) notifies a value.
    pub fn peer_notify(&mut self, handle: u16, data: &[u8]) -> bool {
        if !self.firmware_central(0) {
            return false;
        }
        let mut e = Self::gattc_event(BLE_GATTC_EVT_HVX, BLE_GATT_STATUS_SUCCESS, 16 + data.len());
        put16(&mut e, 10, handle);
        e[12] = BLE_GATT_HVX_NOTIFICATION;
        put16(&mut e, 14, data.len() as u16);
        e[16..].copy_from_slice(data);
        self.arrive(e);
        true
    }

    /// The board's end of the connection goes away (out of range, closed):
    /// the firmware sees the link lost with `reason`, an HCI error code
    /// (0x08 a supervision timeout, 0x13 the remote user ending it).
    pub fn disconnect(&mut self, reason: u8) {
        self.drop_connection(reason, true);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 64 KB of RAM at 0x2000_0000 and R0-R12.
    struct Ram {
        mem: Vec<u8>,
        regs: [u32; 13],
    }

    const RAM: u32 = 0x2000_0000;

    impl Ram {
        fn new() -> Self {
            Self {
                mem: vec![0; 0x1_0000],
                regs: [0; 13],
            }
        }
        fn at(&self, a: u32) -> usize {
            (a - RAM) as usize
        }
        fn put(&mut self, a: u32, data: &[u8]) {
            let i = self.at(a);
            self.mem[i..i + data.len()].copy_from_slice(data);
        }
        fn get(&self, a: u32, len: usize) -> &[u8] {
            let i = self.at(a);
            &self.mem[i..i + len]
        }
        fn u16(&self, a: u32) -> u16 {
            u16::from_le_bytes(self.get(a, 2).try_into().unwrap())
        }
    }

    impl Guest for Ram {
        fn reg(&self, n: u8) -> u32 {
            self.regs[n as usize]
        }
        fn set_reg(&mut self, n: u8, value: u32) {
            self.regs[n as usize] = value;
        }
        fn read(&self, address: u32, len: usize) -> Option<Vec<u8>> {
            (address >= RAM && address + len as u32 <= RAM + 0x1_0000)
                .then(|| self.get(address, len).to_vec())
        }
        fn write(&mut self, address: u32, data: &[u8]) -> bool {
            if address < RAM || address + data.len() as u32 > RAM + 0x1_0000 {
                return false;
            }
            self.put(address, data);
            true
        }
    }

    fn svc(sd: &mut SoftDevice, ram: &mut Ram, n: u32, args: &[u32]) -> u32 {
        ram.regs = [0; 13];
        ram.regs[..args.len()].copy_from_slice(args);
        assert!(sd.call(n, ram, 0));
        ram.regs[0]
    }

    /// Let what the board's end sent arrive.
    fn settle(sd: &mut SoftDevice) {
        while let Some(t) = sd.next_arrival() {
            sd.poll(t);
        }
    }

    /// The next event into a 128-byte buffer at 0x2000_8000.
    fn next(sd: &mut SoftDevice, ram: &mut Ram) -> Option<Vec<u8>> {
        ram.put(0x2000_7FF0, &128u16.to_le_bytes());
        match svc(sd, ram, 97, &[0x2000_8000, 0x2000_7FF0]) {
            NRF_SUCCESS => {
                let len = ram.u16(0x2000_7FF0) as usize;
                Some(ram.get(0x2000_8000, len).to_vec())
            }
            NRF_ERROR_NOT_FOUND => None,
            e => panic!("sd_ble_evt_get {e:#x}"),
        }
    }

    const NUS: u128 = 0x6e400001_b5a3_f393_e0a9_e50e24dcca9e;
    const NUS_RX: u128 = 0x6e400002_b5a3_f393_e0a9_e50e24dcca9e;
    const NUS_TX: u128 = 0x6e400003_b5a3_f393_e0a9_e50e24dcca9e;

    /// Register a vendor UUID; its `ble_uuid_t` at `at`.
    fn vendor(sd: &mut SoftDevice, ram: &mut Ram, uuid: u128, at: u32) {
        let b = uuid.to_le_bytes();
        ram.put(0x2000_1000, &b);
        assert_eq!(svc(sd, ram, 98, &[0x2000_1000, 0x2000_1010]), NRF_SUCCESS);
        let t = ram.get(0x2000_1010, 1)[0];
        ram.put(at, &[b[12], b[13], t, 0]);
    }

    /// The firmware as a peripheral with the NUS: its characteristics, its
    /// advertising, a central connecting, writing, and being notified.
    #[test]
    fn a_peripheral_serves_a_central() {
        let (mut sd, mut ram) = (SoftDevice::new(), Ram::new());
        assert_eq!(svc(&mut sd, &mut ram, 16, &[0, 0]), NRF_SUCCESS);
        vendor(&mut sd, &mut ram, NUS, 0x2000_0100);
        assert_eq!(
            svc(&mut sd, &mut ram, 168, &[1, 0x2000_0100, 0x2000_0104]),
            NRF_SUCCESS
        );
        let service = ram.u16(0x2000_0104);
        assert_eq!(service, FIRST_USER_HANDLE);
        // RX: write, write without response; TX: read, notify
        let mut handles = [0u16; 2];
        for (i, (uuid, props)) in [
            (NUS_RX, PROP_WRITE | PROP_WRITE_WITHOUT_RESPONSE),
            (NUS_TX, PROP_READ | PROP_NOTIFY),
        ]
        .into_iter()
        .enumerate()
        {
            vendor(&mut sd, &mut ram, uuid, 0x2000_0200);
            ram.put(0x2000_0300, &[0; 28]);
            ram.put(0x2000_0300, &[props]);
            let mut attr = [0u8; 20];
            attr[..4].copy_from_slice(&0x2000_0200u32.to_le_bytes());
            put16(&mut attr, 12, 128);
            ram.put(0x2000_0400, &attr);
            assert_eq!(
                svc(
                    &mut sd,
                    &mut ram,
                    170,
                    &[service as u32, 0x2000_0300, 0x2000_0400, 0x2000_0500]
                ),
                NRF_SUCCESS
            );
            handles[i] = ram.u16(0x2000_0500);
            if i == 1 {
                assert_eq!(
                    ram.u16(0x2000_0504),
                    handles[1] + 1,
                    "the CCCD follows the value"
                );
            }
        }
        assert_eq!(
            sd.characteristic(Uuid::from_u128(NUS_TX)),
            Some((handles[1], Some(handles[1] + 1)))
        );
        assert_eq!(
            sd.characteristic(Uuid::from_u128(NUS_RX)),
            Some((handles[0], None))
        );

        // not advertising yet: nobody can connect
        assert!(!sd.central_connect([1, 2, 3, 4, 5, 0xC6]));
        ram.put(0x2000_0600, &[0xFF]);
        ram.put(0x2000_0610, &[0; 24]);
        ram.put(0x2000_0610, &[1]); // connectable scannable undirected
        assert_eq!(
            svc(&mut sd, &mut ram, 114, &[0x2000_0600, 0, 0x2000_0610]),
            NRF_SUCCESS
        );
        assert_eq!(ram.get(0x2000_0600, 1), &[0]);
        assert_eq!(svc(&mut sd, &mut ram, 115, &[0, 1]), NRF_SUCCESS);
        assert!(sd.central_connect([1, 2, 3, 4, 5, 0xC6]));
        assert!(sd.take_irq());
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[0], e[1]]), BLE_GAP_EVT_CONNECTED);
        assert_eq!(&e[9..15], &[1, 2, 3, 4, 5, 0xC6]);
        assert_eq!(e[15], BLE_GAP_ROLE_PERIPH);
        assert!(sd.advertising().is_none());

        // a notification before the central subscribes is refused
        ram.put(0x2000_0700, &3u16.to_le_bytes());
        ram.put(0x2000_0710, b"abc");
        let mut hvx = [0u8; 16];
        put16(&mut hvx, 0, handles[1]);
        hvx[2] = BLE_GATT_HVX_NOTIFICATION;
        hvx[8..12].copy_from_slice(&0x2000_0700u32.to_le_bytes());
        hvx[12..16].copy_from_slice(&0x2000_0710u32.to_le_bytes());
        ram.put(0x2000_0720, &hvx);
        assert_eq!(
            svc(&mut sd, &mut ram, 174, &[0, 0x2000_0720]),
            NRF_ERROR_INVALID_STATE
        );

        // subscribe, then write RX without response
        assert!(sd.central_write(handles[1] + 1, &[1, 0], false));
        assert!(sd.central_write(handles[0], b"\x02\x01\x1e\x00\x00\x03", true));
        settle(&mut sd);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[0], e[1]]), BLE_GATTS_EVT_WRITE);
        assert_eq!(u16::from_le_bytes([e[6], e[7]]), handles[1] + 1);
        assert_eq!(e[12], BLE_GATT_OP_WRITE_REQ);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[6], e[7]]), handles[0]);
        assert_eq!(&e[8..11], &[0x02, 0x00, 2], "the alias and the vendor type");
        assert_eq!(e[12], BLE_GATT_OP_WRITE_CMD);
        assert_eq!(u16::from_le_bytes([e[16], e[17]]), 6);
        assert_eq!(&e[18..], b"\x02\x01\x1e\x00\x00\x03");

        assert_eq!(svc(&mut sd, &mut ram, 174, &[0, 0x2000_0720]), NRF_SUCCESS);
        assert_eq!(
            sd.host.iter().last(),
            Some(&HostEvent::Notified {
                handle: handles[1],
                data: b"abc".to_vec()
            })
        );
        sd.disconnect(0x13);
        settle(&mut sd);
        let _tx_complete = next(&mut sd, &mut ram).unwrap();
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[0], e[1]]), BLE_GAP_EVT_DISCONNECTED);
        assert_eq!(e[8], 0x13);
        assert!(next(&mut sd, &mut ram).is_none());
    }

    /// The firmware as a central: it connects through its whitelist to the
    /// board's peer once present, discovers the NUS, subscribes, writes.
    #[test]
    fn a_central_discovers_and_writes_the_peer() {
        let (mut sd, mut ram) = (SoftDevice::new(), Ram::new());
        svc(&mut sd, &mut ram, 16, &[0, 0]);
        let link = [0x06, 0x6b, 0x71, 0x2c, 0xf5, 0xc0];
        sd.peer = Some(Peer::new(None).service(
            Uuid::from_u128(NUS),
            &[
                (
                    Uuid::from_u128(NUS_RX),
                    PROP_WRITE | PROP_WRITE_WITHOUT_RESPONSE,
                ),
                (Uuid::from_u128(NUS_TX), PROP_NOTIFY),
            ],
        ));
        let mut addr = [0u8; 7];
        addr[0] = BLE_GAP_ADDR_TYPE_RANDOM_STATIC << 1;
        addr[1..].copy_from_slice(&link);
        ram.put(0x2000_0100, &addr);
        ram.put(0x2000_0110, &0x2000_0100u32.to_le_bytes());
        assert_eq!(svc(&mut sd, &mut ram, 110, &[0x2000_0110, 1]), NRF_SUCCESS);
        ram.put(0x2000_0120, &[0; 14]);
        ram.put(0x2000_0140, &[40, 0, 200, 0, 0, 0, 0x90, 1]);
        assert_eq!(
            svc(&mut sd, &mut ram, 140, &[0, 0x2000_0120, 0x2000_0140, 1]),
            NRF_SUCCESS
        );
        // not present yet: nothing happens
        sd.poll(1);
        assert!(next(&mut sd, &mut ram).is_none());
        sd.peer.as_mut().unwrap().present = true;
        sd.poll(2);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[0], e[1]]), BLE_GAP_EVT_CONNECTED);
        assert_eq!(&e[9..15], &link);
        assert_eq!(e[15], BLE_GAP_ROLE_CENTRAL);
        assert_eq!(&e[16..24], &[40, 0, 200, 0, 0, 0, 0x90, 1]);

        // the service, by its UUID registered with the firmware's base
        vendor(&mut sd, &mut ram, NUS, 0x2000_0200);
        assert_eq!(
            svc(&mut sd, &mut ram, 155, &[0, 1, 0x2000_0200]),
            NRF_SUCCESS
        );
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(
            u16::from_le_bytes([e[0], e[1]]),
            BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP
        );
        assert_eq!(u16::from_le_bytes([e[6], e[7]]), BLE_GATT_STATUS_SUCCESS);
        assert_eq!(u16::from_le_bytes([e[10], e[11]]), 1);
        assert_eq!(&e[12..15], &[0x01, 0x00, 2]);
        let (start, end) = (
            u16::from_le_bytes([e[16], e[17]]),
            u16::from_le_bytes([e[18], e[19]]),
        );
        assert_eq!((start, end), (0x10, 0x15));

        let mut range = [0u8; 4];
        put16(&mut range, 0, start);
        put16(&mut range, 2, end);
        ram.put(0x2000_0300, &range);
        svc(&mut sd, &mut ram, 157, &[0, 0x2000_0300]);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[10], e[11]]), 2);
        assert_eq!(e[12 + 4], PROP_WRITE | PROP_WRITE_WITHOUT_RESPONSE);
        assert_eq!(u16::from_le_bytes([e[12 + 8], e[12 + 9]]), 0x12);
        assert_eq!(e[22 + 4], PROP_NOTIFY);
        assert_eq!(u16::from_le_bytes([e[22 + 8], e[22 + 9]]), 0x14);

        // the TX descriptors: its CCCD
        ram.put(0x2000_0300, &[0x15, 0, 0x15, 0]);
        svc(&mut sd, &mut ram, 158, &[0, 0x2000_0300]);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[10], e[11]]), 1);
        assert_eq!(&e[12..17], &[0x15, 0, 0x02, 0x29, BLE_UUID_TYPE_BLE]);

        // subscribe (request, answered), then write RX (command)
        let write = |ram: &mut Ram, op: u8, handle: u16, data: &[u8]| {
            ram.put(0x2000_0400, data);
            let mut p = [0u8; 12];
            p[0] = op;
            put16(&mut p, 2, handle);
            put16(&mut p, 6, data.len() as u16);
            p[8..].copy_from_slice(&0x2000_0400u32.to_le_bytes());
            ram.put(0x2000_0410, &p);
        };
        write(&mut ram, BLE_GATT_OP_WRITE_REQ, 0x15, &[1, 0]);
        assert_eq!(svc(&mut sd, &mut ram, 163, &[0, 0x2000_0410]), NRF_SUCCESS);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[0], e[1]]), BLE_GATTC_EVT_WRITE_RSP);
        assert!(sd.peer.as_ref().unwrap().subscribed(0x14));
        write(&mut ram, BLE_GATT_OP_WRITE_CMD, 0x12, b"\x02\x06\xe6");
        assert_eq!(svc(&mut sd, &mut ram, 163, &[0, 0x2000_0410]), NRF_SUCCESS);
        assert!(
            next(&mut sd, &mut ram).is_none(),
            "a command has no response"
        );
        assert_eq!(
            sd.host.iter().last(),
            Some(&HostEvent::Written {
                handle: 0x12,
                data: b"\x02\x06\xe6".to_vec(),
                command: true
            })
        );

        assert!(sd.peer_notify(0x14, b"xyz"));
        settle(&mut sd);
        let e = next(&mut sd, &mut ram).unwrap();
        assert_eq!(u16::from_le_bytes([e[0], e[1]]), BLE_GATTC_EVT_HVX);
        assert_eq!(u16::from_le_bytes([e[10], e[11]]), 0x14);
        assert_eq!(e[12], BLE_GATT_HVX_NOTIFICATION);
        assert_eq!(&e[16..], b"xyz");
    }

    /// An event larger than the buffer stays queued, its size reported.
    #[test]
    fn an_event_too_large_for_the_buffer_waits() {
        let (mut sd, mut ram) = (SoftDevice::new(), Ram::new());
        sd.push(SoftDevice::event(BLE_GATTC_EVT_HVX, 200));
        ram.put(0x2000_7FF0, &128u16.to_le_bytes());
        assert_eq!(
            svc(&mut sd, &mut ram, 97, &[0x2000_8000, 0x2000_7FF0]),
            NRF_ERROR_DATA_SIZE
        );
        assert_eq!(ram.u16(0x2000_7FF0), 200);
        assert_eq!(sd.max_notification(), 112);
    }

    /// `svc #0` is the bootloader's SVCI, which the MBR passes on: answered
    /// as a bootloader that takes the call, which the buttonless DFU
    /// service's start-up needs (or its error handler resets the chip).
    #[test]
    fn an_svci_call_is_taken_as_a_bootloader_would() {
        let mut sd = SoftDevice::new();
        let mut ram = Ram::new();
        assert_eq!(svc(&mut sd, &mut ram, 0, &[0x2000_0100]), NRF_SUCCESS);
        assert!(sd.notes.iter().any(|n| n.starts_with("SVCI call")));
    }
}
