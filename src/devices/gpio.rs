//! GPIO ports: the board drives the input levels, and sees the output
//! levels and the alternate functions.

use super::regfile::{sub_read, sub_write};

const MODER: u32 = 0x00;
const OTYPER: u32 = 0x04;
const OSPEEDR: u32 = 0x08;
const PUPDR: u32 = 0x0C;
const IDR: u32 = 0x10;
const ODR: u32 = 0x14;
const BSRR: u32 = 0x18;
const LCKR: u32 = 0x1C;
const AFRL: u32 = 0x20;
const AFRH: u32 = 0x24;

pub struct Gpio {
    pub moder: u32,
    otyper: u32,
    ospeedr: u32,
    pupdr: u32,
    pub odr: u32,
    lckr: u32,
    pub afr: [u32; 2],
    /// The levels the board drives on the pins.
    pub input: u32,
}

impl Gpio {
    /// Port `index` (A = 0) at reset: GPIOA and GPIOB keep their debug pins.
    pub fn new(index: usize) -> Self {
        let (moder, ospeedr, pupdr) = match index {
            0 => (0xABFF_FFFF, 0x0C00_0000, 0x6400_0000),
            1 => (0xFFFF_FEBF, 0x0000_00C0, 0x0000_0100),
            _ => (0xFFFF_FFFF, 0, 0),
        };
        Self {
            moder,
            otyper: 0,
            ospeedr,
            pupdr,
            odr: 0,
            lckr: 0,
            afr: [0; 2],
            input: 0,
        }
    }

    /// The mode of `pin`: 0 input, 1 output, 2 alternate, 3 analog.
    pub fn mode(&self, pin: u32) -> u32 {
        (self.moder >> (2 * pin)) & 3
    }

    /// The level of `pin`: the output's on an output, else the board's.
    pub fn level(&self, pin: u32) -> bool {
        if self.mode(pin) == 1 {
            self.odr & (1 << pin) != 0
        } else {
            self.input & (1 << pin) != 0
        }
    }

    fn idr(&self) -> u32 {
        (0..16).fold(0, |idr, pin| idr | (u32::from(self.level(pin)) << pin))
    }

    pub fn read(&self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            MODER => self.moder,
            OTYPER => self.otyper,
            OSPEEDR => self.ospeedr,
            PUPDR => self.pupdr,
            IDR => self.idr(),
            ODR => self.odr,
            LCKR => self.lckr,
            AFRL => self.afr[0],
            AFRH => self.afr[1],
            _ => 0,
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        let reg = |word: u32| sub_write(word, offset, size, value);
        match offset & !3 {
            MODER => self.moder = reg(self.moder),
            OTYPER => self.otyper = reg(self.otyper),
            OSPEEDR => self.ospeedr = reg(self.ospeedr),
            PUPDR => self.pupdr = reg(self.pupdr),
            ODR => self.odr = reg(self.odr) & 0xFFFF,
            BSRR => {
                // a narrow write sets or resets only the bytes it covers
                let bsrr = sub_write(0, offset, size, value);
                self.odr = (self.odr | (bsrr & 0xFFFF)) & !(bsrr >> 16);
            }
            LCKR => self.lckr = reg(self.lckr),
            AFRL => self.afr[0] = reg(self.afr[0]),
            AFRH => self.afr[1] = reg(self.afr[1]),
            _ => {}
        }
    }
}
