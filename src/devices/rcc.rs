//! RCC and PWR: registers that keep what is written, with the ready flags
//! following their enables at once (oscillators, PLLs, voltage scaling) and
//! the clock tree computed from them (RM0399 section 8). The PLL source and
//! dividers keep their value while their PLL runs, as the RCC ignores those
//! writes. The reset flags (RSR, and C1_RSR for the Cortex-M7) are those of a
//! power-on until the board sets others; RMVF clears them.

use super::regfile::RegFile;

const CR: u32 = 0x00;
const CFGR: u32 = 0x10;
const D1CFGR: u32 = 0x18;
const D2CFGR: u32 = 0x1C;
const PLLCKSELR: u32 = 0x28;
const PLLCFGR: u32 = 0x2C;
const PLL1DIVR: u32 = 0x30;
const PLL2DIVR: u32 = 0x38;
const PLL3DIVR: u32 = 0x40;
const BDCR: u32 = 0x70;
const CSR: u32 = 0x74;
const RSR: u32 = 0xD0;
const C1_RSR: u32 = 0x130;
const RMVF: u32 = 1 << 16;
const CPURSTF: u32 = 1 << 17;
const D1RSTF: u32 = 1 << 19;
const D2RSTF: u32 = 1 << 20;
const BORRSTF: u32 = 1 << 21;
const PINRSTF: u32 = 1 << 22;
const PORRSTF: u32 = 1 << 23;
const IWDG1RSTF: u32 = 1 << 26;

/// The reset flags of a power-on.
pub const RESET_POWER_ON: u32 = CPURSTF | D1RSTF | D2RSTF | BORRSTF | PINRSTF | PORRSTF;
/// Those of a reset by the NRST pin (the board's RESET button).
pub const RESET_PIN: u32 = CPURSTF | PINRSTF;
/// Those of a reset by IWDG1, which also pulls NRST.
pub const RESET_IWDG1: u32 = CPURSTF | PINRSTF | IWDG1RSTF;

const HSI_HZ: u64 = 64_000_000;
const CSI_HZ: u64 = 4_000_000;

pub struct Rcc {
    regs: RegFile,
    hse_hz: u64,
    reset_flags: u32,
}

impl Rcc {
    pub fn new(hse_hz: u32) -> Self {
        Self {
            regs: RegFile::with(&[
                // HSION, HSIRDY, HSIDIVF
                (CR, 0x0000_0025),
                // DIVM1..3 = 32
                (PLLCKSELR, 0x0202_0200),
                (PLLCFGR, 0x01FF_0000),
                // DIVN = 129, DIVP = DIVQ = DIVR = 2
                (PLL1DIVR, 0x0101_0280),
                (PLL2DIVR, 0x0101_0280),
                (PLL3DIVR, 0x0101_0280),
            ]),
            hse_hz: u64::from(hse_hz),
            reset_flags: RESET_POWER_ON,
        }
    }

    /// The reset flags RSR reads.
    pub fn reset_flags(&self) -> u32 {
        self.reset_flags
    }

    /// Sets the reset flags, for a chip the board has just reset: those of
    /// the reset, with those the firmware had not cleared.
    pub fn set_reset_flags(&mut self, flags: u32) {
        self.reset_flags = flags;
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            CR => {
                let cr = self.regs.get(CR);
                // HSIRDY, CSIRDY, HSI48RDY, HSERDY, PLL1-3RDY follow their
                // enables; D1CKRDY, D2CKRDY, HSIDIVF are set.
                let ready = ((cr & 1) << 2)
                    | ((cr >> 7 & 1) << 8)
                    | ((cr >> 12 & 1) << 13)
                    | ((cr >> 16 & 1) << 17)
                    | ((cr >> 24 & 1) << 25)
                    | ((cr >> 26 & 1) << 27)
                    | ((cr >> 28 & 1) << 29);
                cr | ready | (1 << 14) | (1 << 15) | (1 << 5)
            }
            CFGR => {
                // SWS follows SW
                let cfgr = self.regs.get(CFGR);
                (cfgr & !0x38) | ((cfgr & 7) << 3)
            }
            BDCR => {
                let bdcr = self.regs.get(BDCR);
                bdcr | ((bdcr & 1) << 1)
            }
            CSR => {
                let csr = self.regs.get(CSR);
                csr | ((csr & 1) << 1)
            }
            RSR | C1_RSR => self.reset_flags,
            _ => return self.regs.read(offset, size),
        };
        super::regfile::sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        if matches!(offset & !3, RSR | C1_RSR) {
            if super::regfile::sub_write(0, offset, size, value) & RMVF != 0 {
                self.reset_flags = 0;
            }
            return;
        }
        let old = self.regs.get(offset);
        let new = super::regfile::sub_write(old, offset, size, value);
        let on = |n: u32| self.regs.get(CR) & (1 << (22 + 2 * n)) != 0;
        let kept = match offset & !3 {
            // PLLSRC with any PLL on, DIVMx with PLLx on
            PLLCKSELR => {
                let mut kept = 0;
                if (1..=3).any(on) {
                    kept |= 3;
                }
                for n in 1..=3 {
                    if on(n) {
                        kept |= 0x3F << (4 + 8 * (n - 1));
                    }
                }
                kept
            }
            o @ (PLL1DIVR | PLL2DIVR | PLL3DIVR) if on((o - PLL1DIVR) / 8 + 1) => u32::MAX,
            _ => 0,
        };
        self.regs.set(offset, (old & kept) | (new & !kept));
    }

    pub fn regs(&mut self) -> &mut RegFile {
        &mut self.regs
    }

    fn hsi(&self) -> u64 {
        HSI_HZ >> ((self.regs.get(CR) >> 3) & 3)
    }

    /// The VCO of PLL `n` (1 to 3) divided by its output divider at `shift`
    /// in PLLxDIVR (9: P, 16: Q, 24: R); 0 when off.
    fn pll_output(&self, n: u32, shift: u32) -> u64 {
        let cr = self.regs.get(CR);
        if cr & (1 << (22 + 2 * n)) == 0 {
            return 0;
        }
        let sel = self.regs.get(PLLCKSELR);
        let source = match sel & 3 {
            0 => self.hsi(),
            1 => CSI_HZ,
            2 => self.hse_hz,
            _ => 0,
        };
        let m = u64::from((sel >> (4 + 8 * (n - 1))) & 0x3F);
        let divr = self.regs.get(PLL1DIVR + 8 * (n - 1));
        let multiplier = u64::from((divr & 0x1FF) + 1);
        let divider = u64::from(((divr >> shift) & 0x7F) + 1);
        if m == 0 {
            return 0;
        }
        source * multiplier / m / divider
    }

    /// The output P of PLL `n` (1 to 3), 0 when off.
    fn pll_p(&self, n: u32) -> u64 {
        self.pll_output(n, 9)
    }

    /// The LTDC's pixel clock, `pll3_r_ck`: 0 while PLL3 or its output R
    /// (PLLCFGR.DIVR3EN) is off.
    pub fn ltdc_clock(&self) -> u64 {
        if self.regs.get(PLLCFGR) & (1 << 24) == 0 {
            return 0;
        }
        self.pll_output(3, 24)
    }

    /// SYSCLK, from SW.
    pub fn sysclk(&self) -> u64 {
        match self.regs.get(CFGR) & 7 {
            0 => self.hsi(),
            1 => CSI_HZ,
            2 => self.hse_hz,
            3 => self.pll_p(1),
            _ => self.hsi(),
        }
    }

    /// The divisor of a D1CPRE or HPRE field: 0xxx /1, then /2, /4 ... /512
    /// skipping /32.
    fn ahb_divisor(field: u32) -> u64 {
        match field & 0xF {
            f if f < 8 => 1,
            f if f < 12 => 2 << (f - 8),
            f => 64 << (f - 12),
        }
    }

    /// The divisor of a D1PPRE, D2PPRE1/2 or D3PPRE field: 0xx /1, then /2
    /// to /16.
    fn apb_divisor(field: u32) -> u64 {
        match field & 7 {
            f if f < 4 => 1,
            f => 2 << (f - 4),
        }
    }

    /// The Cortex-M7 clock, `sys_d1cpre_ck`.
    pub fn cpu_clock(&self) -> u64 {
        self.sysclk() / Self::ahb_divisor(self.regs.get(D1CFGR) >> 8)
    }

    /// The AXI and AHB clock, `rcc_hclk`.
    pub fn hclk(&self) -> u64 {
        self.cpu_clock() / Self::ahb_divisor(self.regs.get(D1CFGR))
    }

    /// The timer clock of APB1 (`apb2`: of APB2): the APB clock, doubled
    /// when its prescaler divides (TIMPRE clear) or up to HCLK (TIMPRE set).
    pub fn timer_clock(&self, apb2: bool) -> u64 {
        let field = self.regs.get(D2CFGR) >> if apb2 { 8 } else { 4 };
        let divisor = Self::apb_divisor(field);
        let timpre = self.regs.get(CFGR) & (1 << 15) != 0;
        let hclk = self.hclk();
        match (divisor, timpre) {
            (1, _) => hclk,
            (d, false) => 2 * hclk / d,
            (d, true) if d <= 4 => hclk,
            (d, true) => 4 * hclk / d,
        }
    }
}

const PWR_CSR1: u32 = 0x04;
const PWR_CR3: u32 = 0x0C;
const PWR_D3CR: u32 = 0x18;

/// PWR: the voltage scaling is ready at once, as is the USB regulator.
pub struct Pwr {
    regs: RegFile,
}

impl Pwr {
    pub fn new() -> Self {
        Self {
            // CR3: SDEN, LDOEN, SMPSLEVEL as at reset; D3CR: VOS3
            regs: RegFile::with(&[
                (0x00, 0xF000_C000),
                (PWR_CR3, 0x0000_0046),
                (PWR_D3CR, 0x4000),
            ]),
        }
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            // ACTVOSRDY, ACTVOS = D3CR.VOS
            PWR_CSR1 => {
                (self.regs.get(PWR_CSR1) & !0xC000) | (1 << 13) | (self.regs.get(PWR_D3CR) & 0xC000)
            }
            // USB33RDY follows USB33DEN; SMPSEXTRDY
            PWR_CR3 => {
                let cr3 = self.regs.get(PWR_CR3);
                cr3 | ((cr3 >> 24 & 1) << 26) | (1 << 16)
            }
            // VOSRDY
            PWR_D3CR => self.regs.get(PWR_D3CR) | (1 << 13),
            _ => return self.regs.read(offset, size),
        };
        super::regfile::sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        self.regs.write(offset, size, value);
    }

    pub fn regs(&mut self) -> &mut RegFile {
        &mut self.regs
    }
}

impl Default for Pwr {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_reset_flags_read_at_both_addresses_until_rmvf_clears_them() {
        let mut rcc = Rcc::new(25_000_000);
        assert_eq!(rcc.read(RSR, 4), RESET_POWER_ON);
        assert_eq!(rcc.read(C1_RSR, 4), RESET_POWER_ON);
        rcc.write(C1_RSR, 4, RMVF);
        assert_eq!(rcc.read(RSR, 4), 0);
        rcc.set_reset_flags(RESET_IWDG1);
        assert_ne!(rcc.read(RSR, 4) & IWDG1RSTF, 0);
    }
}
