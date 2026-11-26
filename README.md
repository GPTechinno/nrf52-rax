# nrf52-rax

nRF52 SoC emulator in Rust, one crate for the three chips of the family: nRF52832, nRF52833 and
nRF52840. The Cortex-M4F is [rax](https://github.com/Georges760/rax)'s Cortex-M core (its
`feat/cortex-m7` branch: the Private Peripheral Bus as memory, the Floating-point Extension with the
Cortex-M4F's single-precision identification); around it the memory map and the peripheral models of
the nRF52 Product Specifications. Board emulators build on it: they drive the pins, put bytes on the
UART lines, attach devices to the I2C buses, and see the pin levels (with a watcher called on every
change), the bytes the UARTEs send and what the firmware leaves in the flash.

## Building

rax is not on crates.io yet, so this crate takes it by path from `../../upstream/rax`: a checkout of
its `feat/cortex-m7` branch in an `upstream` directory beside the `emulator` directory holding this
crate. devenv (stable Rust).

## Use

```rust
let mut config = nrf52_rax::Config::new(nrf52_rax::Variant::Nrf52833);
config.images.push((0x0000_0000, softdevice)); // optional: an MBR and SoftDevice, a bootloader
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
`rtt_read` reads a SEGGER RTT up buffer (defmt).

The `run` example boots an ELF headless and prints the defmt logs, the UART bytes and the accesses to
peripherals not modelled:

```sh
cargo run --release --example run -- --variant 52833 --ms 2000 firmware.elf
cargo run --release --example run -- --variant 52840 --boot 26000 --i2c 0.20:3c app-above-s140.elf
```

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
| EGU0-5, RNG, TEMP, WDT | triggers; random bytes every 30 µs (120 µs with bias correction); the board's temperature 36 µs after START; the watchdog's timeout and reset |
| Others | registers that keep what is written; the first access to each is logged with its PC |

Not modelled: the RADIO and the crypto blocks (ECB, CCM, AAR), so a SoftDevice does not run yet; SPIM,
SPIS, TWIS and the legacy UART, TWI and SPI; SAADC, PWM, QDEC, COMP, PDM, I2S, NFCT, USBD, QSPI; the
EasyDMA list mode; clock stretching; the MPU (stored, not enforced) and the MWU; RAM retention
settings in System OFF (the RAM is kept); the CPU stalls of flash programming.
