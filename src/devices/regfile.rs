//! Registers that keep what is written: the peripherals, or parts of them,
//! not modelled further. The first access to each register is reported, to
//! find what a firmware expects from a peripheral.

use std::collections::{HashMap, HashSet};

#[derive(Default)]
pub struct RegFile {
    regs: HashMap<u32, u32>,
    seen: HashSet<u32>,
}

impl RegFile {
    /// Registers with these reset values (offset, value), zero elsewhere.
    pub fn with(resets: &[(u32, u32)]) -> Self {
        Self {
            regs: resets.iter().copied().collect(),
            seen: HashSet::new(),
        }
    }

    pub fn get(&self, offset: u32) -> u32 {
        self.regs.get(&(offset & !3)).copied().unwrap_or(0)
    }

    pub fn set(&mut self, offset: u32, value: u32) {
        self.regs.insert(offset & !3, value);
    }

    /// Whether this is the first access to the register at `offset`.
    pub fn first(&mut self, offset: u32) -> bool {
        self.seen.insert(offset & !3)
    }

    /// A read of `size` bytes at `offset`.
    pub fn read(&self, offset: u32, size: u32) -> u32 {
        sub_read(self.get(offset), offset, size)
    }

    /// A write of `size` bytes at `offset`, into the other bytes of the
    /// register.
    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        let word = sub_write(self.get(offset), offset, size, value);
        self.set(offset, word);
    }
}

/// The `size` bytes at `offset` of `word`, the register at `offset & !3`.
pub fn sub_read(word: u32, offset: u32, size: u32) -> u32 {
    let shift = 8 * (offset & 3);
    match size {
        4 => word,
        _ => (word >> shift) & ((1 << (8 * size)) - 1),
    }
}

/// `word` with its `size` bytes at `offset` replaced by `value`.
pub fn sub_write(word: u32, offset: u32, size: u32, value: u32) -> u32 {
    if size == 4 {
        return value;
    }
    let shift = 8 * (offset & 3);
    let mask = ((1u32 << (8 * size)) - 1) << shift;
    (word & !mask) | ((value << shift) & mask)
}
