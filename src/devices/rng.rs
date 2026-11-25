//! RNG: a new random word is ready whenever RNGEN is set; no seed or clock
//! error. The words come from a xorshift generator seeded by the machine.

use super::regfile::{RegFile, sub_read};

const CR: u32 = 0x00;
const SR: u32 = 0x04;
const DR: u32 = 0x08;
const RNGEN: u32 = 1 << 2;
const IE: u32 = 1 << 3;
/// CONDRST: held while set, then the conditioning restarts.
const CONDRST: u32 = 1 << 30;

pub struct Rng {
    regs: RegFile,
    state: u64,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            regs: RegFile::default(),
            state: seed | 1,
        }
    }

    fn ready(&self) -> bool {
        let cr = self.regs.get(CR);
        cr & RNGEN != 0 && cr & CONDRST == 0
    }

    pub fn irq(&self) -> bool {
        self.ready() && self.regs.get(CR) & IE != 0
    }

    fn next(&mut self) -> u32 {
        // xorshift64*
        self.state ^= self.state >> 12;
        self.state ^= self.state << 25;
        self.state ^= self.state >> 27;
        (self.state.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 32) as u32
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            SR => u32::from(self.ready()),
            DR if self.ready() => self.next(),
            DR => 0,
            o => self.regs.get(o),
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        if offset & !3 != SR {
            self.regs.write(offset, size, value);
        }
    }
}
