//! STM32H7 SoC emulator: the Cortex-M7 is rax's Cortex-M core with its
//! Floating-point Extension and its Private Peripheral Bus mapped; around it
//! the memory map (flash, the TCMs, the SRAMs, the external memories of the
//! FMC) and the peripheral models. [`machine::Stm32h7`] loads an ELF image
//! and boots it from the flash; the board around the chip talks to it
//! through the peripherals (GPIO levels, the QSPI flash bytes, Ethernet
//! frames and the PHY behind MDIO, the panel on the DSI link, the LTDC's
//! picture).
//!
//! Modelled on the STM32H747, the Cortex-M7 core of the dual-core H745/H747
//! and the single-core H743/H753 memory map they share.

pub mod devices;
pub mod machine;

pub use machine::{Config, Event, Stm32h7};
