//! FMC: the registers keep what is written; the SDRAM controller is never
//! busy and reports its banks in normal mode. The external memories
//! themselves are RAM regions of the bus.

use super::regfile::{RegFile, sub_read};

/// SDSR: BUSY (bit 5) clear, MODES1 and MODES2 normal.
const SDSR: u32 = 0x158;

pub struct Fmc {
    regs: RegFile,
    /// SDRAM commands written to SDCMR (mode, target banks, auto-refresh,
    /// mode register), for the board to check the init sequence.
    pub commands: Vec<u32>,
}

impl Fmc {
    pub fn new() -> Self {
        Self {
            // BCR1: the reset value of bank 1 (MBKEN, MWID 16-bit, WREN...)
            regs: RegFile::with(&[(0x000, 0x0000_30DB), (0x008, 0x0000_30D2)]),
            commands: Vec::new(),
        }
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        match offset & !3 {
            SDSR => sub_read(0, offset, size),
            o => self.regs.read(o | (offset & 3), size),
        }
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        if offset & !3 == 0x150 {
            self.commands.push(value);
        }
        self.regs.write(offset, size, value);
    }

    pub fn regs(&mut self) -> &mut RegFile {
        &mut self.regs
    }
}

impl Default for Fmc {
    fn default() -> Self {
        Self::new()
    }
}
