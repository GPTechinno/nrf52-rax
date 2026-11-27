# nrf52-rax

nRF52 SoC emulator in Rust, one crate for the three chips of the family: nRF52832, nRF52833 and
nRF52840. The Cortex-M4F is [rax](https://github.com/Georges760/rax)'s Cortex-M core (its
`feat/cortex-m4f` branch, our additions not upstream yet: the Private Peripheral Bus as memory,
the Floating-point Extension with the Cortex-M4F's FPv4-SP-D16); around it the memory map and the
peripheral models of the nRF52 Product Specifications. Board emulators build on it: they drive the
pins, put bytes on the UART lines, attach devices to the I2C buses, and see the pin levels (with a
watcher called on every change), the bytes the UARTEs send and what the firmware leaves in the flash.

## Building

rax is not on crates.io yet, so this crate takes it from git: the `feat/cortex-m4f` branch of
[Georges760/rax](https://github.com/Georges760/rax). Cargo checks out the whole repository, its
research documents included: about 3.5 GB in `~/.cargo/git`. devenv (stable Rust).

## Use

```rust
let mut config = nrf52_rax::Config::new(nrf52_rax::Variant::Nrf52833);
config.images.push((0x0007_8000, bootloader)); // optional: images before the ELF (a bootloader)
config.boot_address = 0x2_7000; // optional, with the next line: an application above S140 7.x
config.softdevice = true; // and the SoftDevice model in place of S140
let mut chip = nrf52_rax::Nrf52::new(config, &elf)?;
chip.periph().uarte[0].receive(b"\x02\x01\x00", chip.now()); // the board sends on RXD
let event = chip.run_for(1_000_000_000); // picoseconds: 1 ms
let sent = std::mem::take(&mut chip.periph().uarte[0].line_out);
```

`Nrf52::new` loads the images, then the loadable segments of an ELF at their physical addresses (the
flash, the RAM, the UICR), and powers the chip on: the core boots from the vector table at
`Config::boot_address` (0, or an application's own address when the SoftDevice it is linked above is
absent). `run_for` runs in virtual time: one cycle of the 64 MHz clock per instruction; while the
core sleeps (WFI, WFE) time jumps to the next peripheral event or SysTick interrupt. A system reset
(SYSRESETREQ, the watchdog, a wake from System OFF by a pin) resets the core and the peripherals and
keeps the flash, the UICR, the RAM and the retained registers; `resets` lists them. `periph()` gives
the peripherals: `set_input`/`level` for the pins, `watcher` for their changes, `uarte[n]` for the
UART lines, `i2c` for the I2C buses (named by their SCL pin, so the firmware finds a device wherever
it routes the instance), `temp` for the die temperature, `log` and `trace` for the accesses.
`rtt_read` reads a SEGGER RTT up buffer (defmt). `softdevice()` gives the SoftDevice model (below).

The `run` example boots an ELF headless and prints the defmt logs, the UART bytes and the accesses to
peripherals not modelled:

```sh
cargo run --release --example run -- --variant 52833 --ms 2000 firmware.elf
cargo run --release --example run -- --variant 52840 --boot 26000 --i2c 0.20:3c app-above-s140.elf
cargo run --release --example run -- --variant 52833 --boot 27000 --softdevice ble-app.elf
```

## SoftDevice

The RADIO is not modelled, so a SoftDevice image cannot run. `Config::softdevice` puts a model of
one in its place, at its interface (S132/S140 7.x): an application linked above a SoftDevice boots
at its own address with no SoftDevice image, each `svc #n` it makes (the `sd_*` calls) is answered at
once, and the stack's events wait for `sd_ble_evt_get` with SWI2/EGU2 pended, as with the real one.
`softdevice()` gives the board the other end of the BLE connection:

- the firmware as a peripheral: its GATT server (`characteristic`), its advertising set
  (`advertising`), a central on the board that connects (`central_connect`) and writes
  (`central_write`), the notifications the firmware sends (`host`);
- the firmware as a central: the board's peripheral (`peer`, a `Peer` with its GATT table and its
  address, reachable while `present`), which the firmware connects to through its whitelist,
  discovers (primary services, characteristics, descriptors), reads, writes (`host`) and receives
  notifications from (`peer_notify`).

What the board's end sends reaches the firmware a connection interval (15 ms) later, in order. One
connection; GAP and GATT calls that change nothing the model keeps (connection parameters, PHY, data
length, TX power, configuration) succeed; the SoC calls act on the peripherals (PPI, POWER, CLOCK,
the flash through the NVMC). Not modelled: security (pairing, bonding, encryption), indications,
L2CAP channels, scan reports, the timeslot API, `sd_ecb_block_encrypt`: they return
`NRF_ERROR_NOT_SUPPORTED` and are logged. Checked with nrf-softdevice applications (embassy), as
peripheral and as central.

## Model

| Part | Status |
|---|---|
| Variants | nRF52832 (512 KB flash, 64 KB RAM, P0, 8-bit EasyDMA counts), nRF52833 (512 KB, 128 KB, P0 and P1.00-09, UARTE1), nRF52840 (1 MB, 256 KB, P0 and P1.00-15, UARTE1) |
| Memories | flash, data RAM and its code alias at 0x0080_0000, FICR (part, variant, package, sizes, device ID and address), UICR |
| NVMC | programming with WEN (bits only cleared), page, UICR and full erases, always ready |
| Peripheral interface | tasks, events, SHORTS, INTEN/INTENSET/INTENCLR, interrupt `n` for peripheral ID `n`, 32-bit registers with narrower accesses merged |
| CLOCK, POWER | HFXO and LFCLK start-up times (HFXODEBOUNCE, RC, crystal, synthesised), their status registers, RESETREAS, GPREGRET (kept by a reset), SYSTEMOFF |
| GPIO P0, P1 | outputs, directions, inputs connected or not, pulls, SENSE with LATCH and DETECTMODE; inputs from the board |
| GPIOTE | eight channels in event mode (edges) or task mode (OUT, SET, CLR), the PORT event from DETECT |
| PPI | 20 programmable and 12 pre-programmed channels, FORK, groups and their tasks |
| RTC0-2 | 24-bit counter on LFCLK through PRESCALER, TICK, OVRFLW and COMPARE events routed only when enabled in INTEN or EVTEN, compare when the counter becomes CC |
| TIMER0-4 | timer and counter modes, 8/16/24/32-bit, prescaler, compare with the CLEAR and STOP shorts, capture |
| UARTE0-1 | baud rate and frame from BAUDRATE and CONFIG, EasyDMA receive (RXDRDY, ENDRX, the 4-byte FIFO, STOPRX then RXTO, FLUSHRX, overrun) and transmit (TXDRDY, ENDTX, STOPTX) |
| TWIM0-1 | EasyDMA write and read phases with the address, their bus time from FREQUENCY, LASTTX/LASTRX and their shorts, SUSPEND, STOP, address and data NACKs |
| TWI0-1 (legacy) | a byte at a time through TXD and RXD on the same buses and devices as the TWIM, the BB_SUSPEND and BB_STOP shorts, address and data NACKs |
| SPI0-2 (legacy) | a byte at a time through the double-buffered TXD and RXD, at FREQUENCY, READY per byte; devices on the board's buses, each selected by the GPIO its chip select is wired to |
| EGU0-5, RNG, TEMP, WDT | triggers; random bytes every 30 µs (120 µs with bias correction); the board's temperature 36 µs after START; the watchdog's timeout and reset |
| Others | registers that keep what is written; the first access to each is logged with its PC |

Not modelled: the RADIO and the crypto blocks (ECB, CCM, AAR), so a SoftDevice image does not run
(its model above stands in for it); SPIM, SPIS, TWIS and the legacy UART; SAADC, PWM,
QDEC, COMP, PDM, I2S, NFCT, USBD, QSPI; the EasyDMA list mode; clock stretching; the MPU (stored, not
enforced) and the MWU; RAM retention settings in System OFF (the RAM is kept); the CPU stalls of
flash programming.
