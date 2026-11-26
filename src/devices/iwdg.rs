//! IWDG1, the Cortex-M7's independent watchdog (RM0399 section 47). Once started by `0xCCCC` in KR
//! its 12-bit counter counts down from `0xFFF` on the 32 kHz LSI through the prescaler of PR; each
//! `0xAAAA` reloads it from RLR, and one count after zero it resets the system (a timeout of
//! RLR + 1 counts). `0x5555` unlocks PR, RLR and
//! WINR, whose updates take effect at once (SR reads 0); writing WINR reloads the counter. In
//! window mode (WINR below `0xFFF`) a reload while the counter is above WINR resets the system too.
//! Nothing stops it once started.

use super::regfile::sub_read;

const KR: u32 = 0x00;
const PR: u32 = 0x04;
const RLR: u32 = 0x08;
const WINR: u32 = 0x10;
const KEY_UNLOCK: u32 = 0x5555;
const KEY_RELOAD: u32 = 0xAAAA;
const KEY_START: u32 = 0xCCCC;
const LSI_HZ: u64 = 32_000;
const PS_PER_S: u64 = 1_000_000_000_000;
const MAX: u32 = 0xFFF;

pub struct Iwdg {
    pr: u32,
    rlr: u32,
    winr: u32,
    unlocked: bool,
    /// Once started: when the counter was last loaded, and with what.
    loaded: Option<(u64, u32)>,
    /// The system is to be reset: the counter ran out, or a reload came outside the window.
    pub expired: bool,
}

impl Default for Iwdg {
    fn default() -> Self {
        Self::new()
    }
}

impl Iwdg {
    pub fn new() -> Self {
        Self {
            pr: 0,
            rlr: MAX,
            winr: MAX,
            unlocked: false,
            loaded: None,
            expired: false,
        }
    }

    /// One count of the counter, picoseconds: the LSI divided by 4 to 256.
    fn period(&self) -> u64 {
        PS_PER_S * (4 << self.pr.min(6)) / LSI_HZ
    }

    fn counter(&self, now: u64) -> Option<u32> {
        let (at, value) = self.loaded?;
        let counts = now.saturating_sub(at) / self.period();
        Some(value.saturating_sub(counts.min(u64::from(MAX)) as u32))
    }

    /// When the counter runs out, once started: one count after zero.
    fn deadline(&self) -> Option<u64> {
        let (at, value) = self.loaded?;
        Some(at + (u64::from(value) + 1) * self.period())
    }

    /// Advances to `now`: whether the counter ran out, and when it will.
    pub fn update(&mut self, now: u64) -> Option<u64> {
        let deadline = self.deadline()?;
        if now >= deadline {
            self.expired = true;
        }
        Some(deadline)
    }

    fn reload(&mut self, now: u64) {
        if let Some(counter) = self.counter(now) {
            if self.winr < MAX && counter > self.winr {
                self.expired = true;
            }
            self.loaded = Some((now, self.rlr));
        }
    }

    pub fn read(&self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            PR => self.pr,
            RLR => self.rlr,
            WINR => self.winr,
            _ => 0,
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, value: u32, now: u64) {
        match offset & !3 {
            KR => match value & 0xFFFF {
                KEY_UNLOCK => self.unlocked = true,
                KEY_RELOAD => {
                    self.unlocked = false;
                    self.reload(now);
                }
                KEY_START => {
                    self.unlocked = false;
                    if self.loaded.is_none() {
                        self.loaded = Some((now, MAX));
                    }
                }
                _ => self.unlocked = false,
            },
            PR if self.unlocked => self.pr = value & 7,
            RLR if self.unlocked => self.rlr = value & MAX,
            WINR if self.unlocked => {
                self.winr = value & MAX;
                if self.loaded.is_some() {
                    self.loaded = Some((now, self.rlr));
                }
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: u64 = PS_PER_S;

    /// What embassy-stm32's driver writes for a 32 s timeout: prescaler 256, reload 4000.
    fn configured() -> Iwdg {
        let mut w = Iwdg::new();
        w.write(KR, KEY_UNLOCK, 0);
        w.write(PR, 6, 0);
        w.write(RLR, 3999, 0);
        w
    }

    #[test]
    fn it_counts_only_once_started() {
        let mut w = configured();
        assert_eq!(w.update(100 * S), None);
        assert!(!w.expired);
    }

    #[test]
    fn started_it_resets_after_0x1000_counts_then_rlr_plus_1_after_a_reload() {
        let mut w = configured();
        w.write(KR, KEY_START, 0);
        // 0xFFF + 1 counts of 8 ms
        assert_eq!(w.update(0), Some(4096 * 8 * S / 1000));
        w.update(32 * S);
        assert!(!w.expired);
        w.write(KR, KEY_RELOAD, 32 * S);
        // 4000 counts of 8 ms from the reload
        assert_eq!(w.update(32 * S), Some(64 * S));
        w.update(64 * S - 1);
        assert!(!w.expired);
        w.update(64 * S);
        assert!(w.expired);
    }

    #[test]
    fn a_locked_register_keeps_its_value() {
        let mut w = configured();
        w.write(KR, KEY_START, 0);
        w.write(PR, 0, 0);
        w.write(RLR, 1, 0);
        assert_eq!((w.read(PR, 4), w.read(RLR, 4)), (6, 3999));
    }

    #[test]
    fn a_reload_above_the_window_resets() {
        let mut w = configured();
        w.write(KR, KEY_START, 0);
        w.write(KR, KEY_UNLOCK, 0);
        w.write(WINR, 1000, 0);
        // just after the reload WINR's write made, the counter is near 4000, above 1000
        w.write(KR, KEY_RELOAD, S);
        assert!(w.expired);
    }

    #[test]
    fn a_reload_below_the_window_is_taken() {
        let mut w = configured();
        w.write(KR, KEY_START, 0);
        w.write(KR, KEY_UNLOCK, 0);
        w.write(WINR, 1000, 0);
        // 3500 counts of 8 ms later the counter is 500
        w.write(KR, KEY_RELOAD, 28 * S);
        assert!(!w.expired);
    }
}
