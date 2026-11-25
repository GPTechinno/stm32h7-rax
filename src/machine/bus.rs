//! The memory map of the Cortex-M7 (RM0399 section 2.3): the RAMs, the
//! flash (written only by loading an image), the system memory with the
//! unique ID, the external RAMs of the FMC, the QSPI window, and the
//! peripherals.

use std::fmt;
use std::sync::{Arc, Mutex};

use rax::isa::arm::common::cpu::AccessType;
use rax::isa::arm::common::memory::{ArmMemory, MemResult, MemoryError, MmioHandler};

use crate::devices::Peripherals;

pub const FLASH_BASE: u32 = 0x0800_0000;
pub const FLASH_SIZE: usize = 2 << 20;
const SYSTEM_BASE: u32 = 0x1FF0_0000;
const SYSTEM_SIZE: usize = 128 << 10;
/// The 96-bit unique ID, and the flash size in KB (16 bits).
const UID: u32 = 0x1FF1_E800;
const FLASH_SIZE_REG: u32 = 0x1FF1_E880;
const QSPI_WINDOW: u32 = 0x9000_0000;
const QSPI_WINDOW_END: u32 = 0xA000_0000;

/// RAMs and ROMs: (base, bytes, writable).
const MEMORIES: [(u32, usize, bool); 8] = [
    (FLASH_BASE, FLASH_SIZE, false),
    (0x2400_0000, 512 << 10, true),    // AXI SRAM
    (0x2000_0000, 128 << 10, true),    // DTCM
    (0x0000_0000, 64 << 10, true),     // ITCM
    (0x3000_0000, 288 << 10, true),    // SRAM1, SRAM2, SRAM3
    (0x3800_0000, 64 << 10, true),     // SRAM4
    (0x3880_0000, 4 << 10, true),      // backup SRAM
    (SYSTEM_BASE, SYSTEM_SIZE, false), // system memory
];

struct Region {
    base: u32,
    bytes: Vec<u8>,
    writable: bool,
}

impl Region {
    fn offset(&self, address: u64, len: usize) -> Option<usize> {
        let offset = address.checked_sub(u64::from(self.base))? as usize;
        (offset + len <= self.bytes.len()).then_some(offset)
    }
}

pub struct Bus {
    regions: Vec<Region>,
    periph: Arc<Mutex<Peripherals>>,
    exclusive: Option<(u64, u8)>,
}

impl fmt::Debug for Bus {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Bus")
            .field("regions", &self.regions.len())
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

impl Bus {
    /// The on-chip memories, the external RAMs (base, bytes) and the
    /// peripherals.
    pub fn new(periph: Arc<Mutex<Peripherals>>, external: &[(u32, usize)], uid: [u8; 12]) -> Self {
        let mut regions: Vec<Region> = MEMORIES
            .iter()
            .map(|&(base, size, writable)| Region {
                base,
                bytes: vec![if base == FLASH_BASE { 0xFF } else { 0 }; size],
                writable,
            })
            .collect();
        regions.extend(external.iter().map(|&(base, size)| Region {
            base,
            bytes: vec![0; size],
            writable: true,
        }));
        let mut bus = Self {
            regions,
            periph,
            exclusive: None,
        };
        bus.load(UID, &uid);
        bus.load(FLASH_SIZE_REG, &((FLASH_SIZE >> 10) as u16).to_le_bytes());
        bus
    }

    fn region(&self, address: u64, len: usize) -> Option<(usize, usize)> {
        self.regions
            .iter()
            .enumerate()
            .find_map(|(i, r)| r.offset(address, len).map(|o| (i, o)))
    }

    /// Writes `bytes` at `address` whatever the region (loading an image).
    pub fn load(&mut self, address: u32, bytes: &[u8]) -> bool {
        match self.region(u64::from(address), bytes.len()) {
            Some((i, o)) => {
                self.regions[i].bytes[o..o + bytes.len()].copy_from_slice(bytes);
                true
            }
            None => false,
        }
    }

    fn mmio_read(&self, address: u64, buf: &mut [u8]) -> MemResult<()> {
        let len = buf.len();
        let fail = || unmapped(address, len, AccessType::Read);
        let address32 = u32::try_from(address).map_err(|_| fail())?;
        if (QSPI_WINDOW..QSPI_WINDOW_END).contains(&address32) {
            let p = self.periph.lock().map_err(|_| fail())?;
            return if p.quadspi.mapped_read(address32 - QSPI_WINDOW, buf) {
                Ok(())
            } else {
                Err(fail())
            };
        }
        let mut p = self.periph.lock().map_err(|_| fail())?;
        let mut done = 0;
        while done < buf.len() {
            let size = match buf.len() - done {
                n if n >= 4 && (address32 + done as u32).is_multiple_of(4) => 4,
                n if n >= 2 && (address32 + done as u32).is_multiple_of(2) => 2,
                _ => 1,
            };
            let value = p
                .read(address32 + done as u32, size as u32)
                .ok_or_else(fail)?;
            buf[done..done + size].copy_from_slice(&value.to_le_bytes()[..size]);
            done += size;
        }
        Ok(())
    }

    fn mmio_write(&mut self, address: u64, data: &[u8]) -> MemResult<()> {
        let len = data.len();
        let fail = || unmapped(address, len, AccessType::Write);
        let address32 = u32::try_from(address).map_err(|_| fail())?;
        let mut p = self.periph.lock().map_err(|_| fail())?;
        let mut done = 0;
        while done < data.len() {
            let size = match data.len() - done {
                n if n >= 4 && (address32 + done as u32).is_multiple_of(4) => 4,
                n if n >= 2 && (address32 + done as u32).is_multiple_of(2) => 2,
                _ => 1,
            };
            let mut word = [0u8; 4];
            word[..size].copy_from_slice(&data[done..done + size]);
            if !p.write(
                address32 + done as u32,
                size as u32,
                u32::from_le_bytes(word),
            ) {
                return Err(fail());
            }
            done += size;
        }
        Ok(())
    }
}

impl ArmMemory for Bus {
    fn read(&self, address: u64, buf: &mut [u8]) -> MemResult<()> {
        match self.region(address, buf.len()) {
            Some((i, o)) => {
                buf.copy_from_slice(&self.regions[i].bytes[o..o + buf.len()]);
                Ok(())
            }
            None => self.mmio_read(address, buf),
        }
    }

    fn write(&mut self, address: u64, data: &[u8]) -> MemResult<()> {
        match self.region(address, data.len()) {
            Some((i, o)) => {
                let region = &mut self.regions[i];
                if region.writable {
                    region.bytes[o..o + data.len()].copy_from_slice(data);
                }
                // the flash is programmed through its interface, not written
                Ok(())
            }
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
