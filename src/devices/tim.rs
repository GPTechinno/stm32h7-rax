//! General-purpose and basic timers, up-counting: the counter runs from the
//! timer kernel clock through the prescaler while CEN is set; the update
//! (overflow at ARR) and the compare matches of the four channels set their
//! flags, and the interrupt line is high while a flag and its enable are.
//! Time is in picoseconds. Not modelled: down and center-aligned counting,
//! the repetition counter, slave modes, input capture, the outputs.

use super::regfile::{RegFile, sub_read, sub_write};

const CR1: u32 = 0x00;
const DIER: u32 = 0x0C;
const SR: u32 = 0x10;
const EGR: u32 = 0x14;
const CNT: u32 = 0x24;
const PSC: u32 = 0x28;
const ARR: u32 = 0x2C;
const CCR1: u32 = 0x34;

const CEN: u32 = 1 << 0;
const URS: u32 = 1 << 2;
const UIF: u32 = 1 << 0;
const PS_PER_S: u128 = 1_000_000_000_000;

pub struct Timer {
    /// 32-bit counter (TIM2, TIM5), else 16-bit.
    wide: bool,
    /// On APB2 (TIM1, TIM8, TIM15-17), else APB1.
    pub apb2: bool,
    regs: RegFile,
    cr1: u32,
    dier: u32,
    sr: u32,
    psc: u32,
    /// The prescaler in use, PSC as of the last update event.
    psc_active: u32,
    arr: u32,
    ccr: [u32; 4],
    /// The counter at `since`, a time in picoseconds.
    cnt: u32,
    since: u64,
    /// Kernel clock, Hz.
    clock: u64,
}

impl Timer {
    pub fn new(wide: bool, apb2: bool) -> Self {
        Self {
            wide,
            apb2,
            regs: RegFile::default(),
            cr1: 0,
            dier: 0,
            sr: 0,
            psc: 0,
            psc_active: 0,
            arr: if wide { u32::MAX } else { 0xFFFF },
            ccr: [0; 4],
            cnt: 0,
            since: 0,
            clock: 1,
        }
    }

    fn running(&self) -> bool {
        self.cr1 & CEN != 0 && self.clock > 0
    }

    /// Picoseconds per counter tick, as a fraction (numerator, denominator).
    fn ticks_between(&self, from: u64, to: u64) -> u128 {
        let ps = u128::from(to.saturating_sub(from));
        ps * u128::from(self.clock) / (u128::from(self.psc_active) + 1) / PS_PER_S
    }

    /// The time at which `ticks` ticks after `since` have elapsed.
    fn time_after(&self, ticks: u128) -> u64 {
        let ps =
            (ticks * (u128::from(self.psc_active) + 1) * PS_PER_S).div_ceil(u128::from(self.clock));
        self.since
            .saturating_add(ps.min(u128::from(u64::MAX)) as u64)
    }

    /// Advances the counter to `now`, setting the flags of what it passed,
    /// and sets the kernel clock for what follows.
    pub fn update(&mut self, now: u64, clock: u64) {
        if self.running() {
            let ticks = self.ticks_between(self.since, now);
            if ticks > 0 {
                let period = u128::from(self.arr) + 1;
                let start = u128::from(self.cnt);
                let end = start + ticks;
                if end >= period {
                    self.sr |= UIF;
                    self.psc_active = self.psc;
                }
                for (i, &ccr) in self.ccr.iter().enumerate() {
                    let ccr = u128::from(ccr);
                    // a match at `ccr` happens at start < t <= end, modulo
                    // the period
                    let next = if ccr > start { ccr } else { ccr + period };
                    if ccr < period && next <= end {
                        self.sr |= 1 << (i + 1);
                    }
                }
                self.cnt = (end % period) as u32;
                // keep the remainder of a partial tick
                self.since = self.time_after(ticks);
            }
        } else {
            self.since = now;
        }
        if clock != self.clock {
            self.clock = clock.max(1);
        }
    }

    /// The time of the next event whose interrupt is enabled and flag clear.
    pub fn next_event(&self) -> Option<u64> {
        if !self.running() {
            return None;
        }
        let period = u128::from(self.arr) + 1;
        let cnt = u128::from(self.cnt);
        let mut ticks: Option<u128> = None;
        let mut consider = |t: u128| ticks = Some(ticks.map_or(t, |old| old.min(t)));
        if self.dier & UIF != 0 && self.sr & UIF == 0 {
            consider(period - cnt);
        }
        for (i, &ccr) in self.ccr.iter().enumerate() {
            let bit = 1 << (i + 1);
            if self.dier & bit != 0 && self.sr & bit == 0 && u128::from(ccr) < period {
                let ccr = u128::from(ccr);
                consider(if ccr > cnt {
                    ccr - cnt
                } else {
                    ccr + period - cnt
                });
            }
        }
        ticks.map(|t| self.time_after(t))
    }

    /// The interrupt line: a flag and its enable.
    pub fn irq(&self) -> bool {
        self.sr & self.dier & 0x1F != 0
    }

    fn mask(&self) -> u32 {
        if self.wide { u32::MAX } else { 0xFFFF }
    }

    pub fn read(&mut self, offset: u32, size: u32, now: u64) -> u32 {
        self.update(now, self.clock);
        let word = match offset & !3 {
            CR1 => self.cr1,
            DIER => self.dier,
            SR => self.sr,
            CNT => self.cnt,
            PSC => self.psc,
            ARR => self.arr,
            o @ CCR1..=0x40 => self.ccr[((o - CCR1) / 4) as usize],
            o => self.regs.get(o),
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32, now: u64) {
        self.update(now, self.clock);
        let reg = |word: u32| sub_write(word, offset, size, value);
        match offset & !3 {
            CR1 => {
                let cr1 = reg(self.cr1);
                if cr1 & CEN != 0 && self.cr1 & CEN == 0 {
                    self.since = now;
                }
                self.cr1 = cr1;
            }
            DIER => self.dier = reg(self.dier),
            // rc_w0: written zeros clear
            SR => self.sr &= reg(u32::MAX),
            EGR => {
                if value & 1 != 0 {
                    // UG: counter and prescaler reloaded; UIF unless URS
                    self.cnt = 0;
                    self.since = now;
                    self.psc_active = self.psc;
                    if self.cr1 & URS == 0 {
                        self.sr |= UIF;
                    }
                }
                // CCxG
                self.sr |= value & 0x1E;
            }
            CNT => {
                self.cnt = reg(self.cnt) & self.mask();
                self.since = now;
            }
            PSC => self.psc = reg(self.psc) & 0xFFFF,
            ARR => self.arr = reg(self.arr) & self.mask(),
            o @ CCR1..=0x40 => {
                let i = ((o - CCR1) / 4) as usize;
                self.ccr[i] = reg(self.ccr[i]) & self.mask();
            }
            o => self.regs.write(o, size, value),
        }
    }
}
