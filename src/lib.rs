//! nRF52 SoC emulator: the Cortex-M4F is rax's Cortex-M core with its
//! Floating-point Extension and its Private Peripheral Bus mapped; around it
//! the memory map (flash, data RAM, FICR, UICR and the NVMC that programs
//! them) and the peripheral models. [`machine::Nrf52`] loads an ELF image
//! and boots it from the flash; the board around the chip talks to it
//! through the peripherals (GPIO levels and a watcher of their changes, the
//! UART lines, the devices on the I2C buses). [`softdevice`] answers a
//! SoftDevice's supervisor calls, for firmware linked above one, with the
//! board as the other end of its BLE connection.
//!
//! One crate for the three chips of the family, [`Variant`]: nRF52832,
//! nRF52833 and nRF52840.

pub mod devices;
pub mod machine;
pub mod softdevice;
pub mod variant;

pub use machine::{Config, Event, Nrf52};
pub use variant::Variant;
