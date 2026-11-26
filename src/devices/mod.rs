//! The peripherals and their place in the memory map (RM0399 section 2.3),
//! their interrupt lines, and the virtual time they run on.
//!
//! The peripherals not modelled are registers that keep what is written; the
//! first access to each is reported in [`Peripherals::log`], with the PC
//! that made it.

pub mod dsi;
pub mod eth;
pub mod fmc;
pub mod gpio;
pub mod ltdc;
pub mod quadspi;
pub mod rcc;
pub mod regfile;
pub mod rng;
pub mod tim;

use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::atomic::{AtomicU32, AtomicU64, Ordering};

use dsi::Dsi;
use eth::Eth;
use fmc::Fmc;
use gpio::Gpio;
use ltdc::Ltdc;
use quadspi::{NorFlash, Quadspi};
use rcc::{Pwr, Rcc};
use regfile::RegFile;
use rng::Rng;
use tim::Timer;

/// The virtual time in picoseconds, and the PC of the instruction running,
/// shared by the machine with the peripherals.
#[derive(Default)]
pub struct Clock {
    pub now: AtomicU64,
    pub pc: AtomicU32,
}

impl Clock {
    pub fn now(&self) -> u64 {
        self.now.load(Ordering::Relaxed)
    }

    pub fn pc(&self) -> u32 {
        self.pc.load(Ordering::Relaxed)
    }
}

/// The timers: base address, 32-bit counter, on APB2, interrupt line (the
/// update and compare interrupts of one timer share it here).
const TIMERS: [(u32, bool, bool, Option<u16>); 14] = [
    (0x4000_0000, true, false, Some(28)),  // TIM2
    (0x4000_0400, false, false, Some(29)), // TIM3
    (0x4000_0800, false, false, Some(30)), // TIM4
    (0x4000_0C00, true, false, Some(50)),  // TIM5
    (0x4000_1000, false, false, Some(54)), // TIM6
    (0x4000_1400, false, false, Some(55)), // TIM7
    (0x4000_1800, false, false, Some(43)), // TIM12 (TIM8_BRK_TIM12)
    (0x4000_1C00, false, false, Some(44)), // TIM13 (TIM8_UP_TIM13)
    (0x4000_2000, false, false, Some(45)), // TIM14 (TIM8_TRG_COM_TIM14)
    (0x4001_0000, false, true, None),      // TIM1
    (0x4001_0400, false, true, None),      // TIM8
    (0x4001_4000, false, true, Some(116)), // TIM15
    (0x4001_4400, false, true, Some(117)), // TIM16
    (0x4001_4800, false, true, Some(118)), // TIM17
];

const DSI_BASE: u32 = 0x5000_0000;
const LTDC_BASE: u32 = 0x5000_1000;
const ETH_BASE: u32 = 0x4002_8000;
const ETH_SIZE: u32 = 0x1400;
const RNG_BASE: u32 = 0x4802_1800;
const FLASH_BASE: u32 = 0x5200_2000;
const FMC_BASE: u32 = 0x5200_4000;
const QUADSPI_BASE: u32 = 0x5200_5000;
const EXTI_BASE: u32 = 0x5800_0000;
const SYSCFG_BASE: u32 = 0x5800_0400;
const GPIO_BASE: u32 = 0x5802_0000;
const GPIO_PORTS: usize = 11;
const RCC_BASE: u32 = 0x5802_4400;
const PWR_BASE: u32 = 0x5802_4800;
const HSEM_BASE: u32 = 0x5802_6400;
const DBGMCU_BASE: u32 = 0x5C00_1000;
/// DBGMCU_IDCODE of the STM32H745/H747/H755/H757: DEV_ID 0x450, revision V.
const IDCODE: u32 = 0x2003_6450;

pub const IRQ_ETH: u16 = 61;
pub const IRQ_RNG: u16 = 80;
pub const IRQ_LTDC: u16 = 88;

/// SYSCFG_CCCSR: READY (bit 8) follows EN (bit 0).
const SYSCFG_CCCSR: u32 = 0x20;

pub struct Peripherals {
    pub clock: Arc<Clock>,
    pub rcc: Rcc,
    pub pwr: Pwr,
    pub flash: RegFile,
    pub syscfg: RegFile,
    pub exti: RegFile,
    pub hsem: RegFile,
    pub dbgmcu: RegFile,
    pub gpio: Vec<Gpio>,
    /// Indexed like `TIMERS`.
    pub timers: Vec<Timer>,
    pub rng: Rng,
    pub fmc: Fmc,
    pub quadspi: Quadspi,
    pub eth: Eth,
    pub ltdc: Ltdc,
    pub dsi: Dsi,
    /// The other blocks of 1 KB, by base address.
    pub other: BTreeMap<u32, RegFile>,
    /// First accesses to registers not modelled, and the traced accesses.
    pub log: Vec<String>,
    /// Accesses in this range of addresses (start, end) are all logged.
    pub trace: Option<(u32, u32)>,
}

impl Peripherals {
    pub fn new(clock: Arc<Clock>, hse_hz: u32, qspi_flash: NorFlash, seed: u64) -> Self {
        Self {
            clock,
            rcc: Rcc::new(hse_hz),
            pwr: Pwr::new(),
            flash: RegFile::default(),
            syscfg: RegFile::default(),
            exti: RegFile::default(),
            hsem: RegFile::default(),
            dbgmcu: RegFile::with(&[(0, IDCODE)]),
            gpio: (0..GPIO_PORTS).map(Gpio::new).collect(),
            timers: TIMERS
                .iter()
                .map(|&(_, wide, apb2, _)| Timer::new(wide, apb2))
                .collect(),
            rng: Rng::new(seed),
            fmc: Fmc::new(),
            quadspi: Quadspi::new(qspi_flash),
            eth: Eth::new(),
            ltdc: Ltdc::new(),
            dsi: Dsi::new(),
            other: BTreeMap::new(),
            log: Vec::new(),
            trace: None,
        }
    }

    /// Advances the timers and the LTDC to `now`; returns the time of their
    /// next event.
    pub fn update(&mut self, now: u64) -> Option<u64> {
        let apb1 = self.rcc.timer_clock(false);
        let apb2 = self.rcc.timer_clock(true);
        let ltdc = self.ltdc.update(now, self.rcc.ltdc_clock());
        self.timers
            .iter_mut()
            .filter_map(|t| {
                t.update(now, if t.apb2 { apb2 } else { apb1 });
                t.next_event()
            })
            .chain(ltdc)
            .min()
    }

    /// The interrupt lines that are high.
    pub fn irq_lines(&self) -> Vec<u16> {
        let mut lines: Vec<u16> = self
            .timers
            .iter()
            .zip(TIMERS.iter())
            .filter_map(|(t, &(_, _, _, line))| line.filter(|_| t.irq()))
            .collect();
        if self.eth.irq() {
            lines.push(IRQ_ETH);
        }
        if self.rng.irq() {
            lines.push(IRQ_RNG);
        }
        if self.ltdc.irq() {
            lines.push(IRQ_LTDC);
        }
        lines
    }

    /// The Cortex-M7 clock, Hz.
    pub fn cpu_clock(&self) -> u64 {
        self.rcc.cpu_clock()
    }

    fn note(&mut self, kind: &str, address: u32) {
        let pc = self.clock.pc();
        self.log.push(format!(
            "{kind} {address:#010x} (pc {pc:#010x}): not modelled"
        ));
    }

    fn timer(address: u32) -> Option<usize> {
        TIMERS
            .iter()
            .position(|&(base, ..)| (base..base + 0x400).contains(&address))
    }

    fn traced(&self, address: u32) -> bool {
        self.trace
            .is_some_and(|(start, end)| (start..end).contains(&address))
    }

    /// A read of `size` bytes at `address`; `None` outside the peripherals.
    pub fn read(&mut self, address: u32, size: u32) -> Option<u32> {
        let value = self.read_inner(address, size);
        if self.traced(address) {
            let pc = self.clock.pc();
            self.log.push(format!(
                "read{} {address:#010x} = {:#x} (pc {pc:#010x})",
                8 * size,
                value.unwrap_or(0)
            ));
        }
        value
    }

    /// A write of `size` bytes at `address`; `false` outside the
    /// peripherals.
    pub fn write(&mut self, address: u32, size: u32, value: u32) -> bool {
        if self.traced(address) {
            let pc = self.clock.pc();
            self.log.push(format!(
                "write{} {address:#010x} = {value:#x} (pc {pc:#010x})",
                8 * size
            ));
        }
        self.write_inner(address, size, value)
    }

    fn read_inner(&mut self, address: u32, size: u32) -> Option<u32> {
        let now = self.clock.now();
        if let Some(i) = Self::timer(address) {
            return Some(self.timers[i].read(address & 0x3FF, size, now));
        }
        Some(match address {
            a if (DSI_BASE..DSI_BASE + 0x1000).contains(&a) => self.dsi.read(a - DSI_BASE, size),
            a if (LTDC_BASE..LTDC_BASE + 0x1000).contains(&a) => {
                self.ltdc.read(a - LTDC_BASE, size, now)
            }
            a if (ETH_BASE..ETH_BASE + ETH_SIZE).contains(&a) => self.eth.read(a - ETH_BASE, size),
            a if (RNG_BASE..RNG_BASE + 0x400).contains(&a) => self.rng.read(a - RNG_BASE, size),
            a if (FLASH_BASE..FLASH_BASE + 0x1000).contains(&a) => {
                self.flash.read(a - FLASH_BASE, size)
            }
            a if (FMC_BASE..FMC_BASE + 0x1000).contains(&a) => self.fmc.read(a - FMC_BASE, size),
            a if (QUADSPI_BASE..QUADSPI_BASE + 0x1000).contains(&a) => {
                self.quadspi.read(a - QUADSPI_BASE, size)
            }
            a if (EXTI_BASE..EXTI_BASE + 0x400).contains(&a) => self.exti.read(a - EXTI_BASE, size),
            a if (SYSCFG_BASE..SYSCFG_BASE + 0x400).contains(&a) => {
                let offset = a - SYSCFG_BASE;
                if offset & !3 == SYSCFG_CCCSR {
                    let cccsr = self.syscfg.get(SYSCFG_CCCSR);
                    regfile::sub_read(cccsr | ((cccsr & 1) << 8), offset, size)
                } else {
                    self.syscfg.read(offset, size)
                }
            }
            a if (GPIO_BASE..GPIO_BASE + 0x400 * GPIO_PORTS as u32).contains(&a) => {
                let port = ((a - GPIO_BASE) / 0x400) as usize;
                self.gpio[port].read(a & 0x3FF, size)
            }
            a if (RCC_BASE..RCC_BASE + 0x400).contains(&a) => self.rcc.read(a - RCC_BASE, size),
            a if (PWR_BASE..PWR_BASE + 0x400).contains(&a) => self.pwr.read(a - PWR_BASE, size),
            a if (HSEM_BASE..HSEM_BASE + 0x400).contains(&a) => {
                if self.hsem.first(a - HSEM_BASE) {
                    self.note("HSEM read", a);
                }
                self.hsem.read(a - HSEM_BASE, size)
            }
            a if (DBGMCU_BASE..DBGMCU_BASE + 0x400).contains(&a) => {
                self.dbgmcu.read(a - DBGMCU_BASE, size)
            }
            a if (0x4000_0000..0x6000_0000).contains(&a) => {
                let base = a & !0x3FF;
                let first = self.other.entry(base).or_default().first(a & 0x3FF);
                if first {
                    self.note("read", a);
                }
                self.other[&base].read(a & 0x3FF, size)
            }
            _ => return None,
        })
    }

    fn write_inner(&mut self, address: u32, size: u32, value: u32) -> bool {
        let now = self.clock.now();
        if let Some(i) = Self::timer(address) {
            self.timers[i].write(address & 0x3FF, size, value, now);
            return true;
        }
        match address {
            a if (DSI_BASE..DSI_BASE + 0x1000).contains(&a) => {
                self.dsi.write(a - DSI_BASE, size, value)
            }
            a if (LTDC_BASE..LTDC_BASE + 0x1000).contains(&a) => {
                self.ltdc.write(a - LTDC_BASE, size, value, now)
            }
            a if (ETH_BASE..ETH_BASE + ETH_SIZE).contains(&a) => {
                self.eth.write(a - ETH_BASE, size, value)
            }
            a if (RNG_BASE..RNG_BASE + 0x400).contains(&a) => {
                self.rng.write(a - RNG_BASE, size, value)
            }
            a if (FLASH_BASE..FLASH_BASE + 0x1000).contains(&a) => {
                self.flash.write(a - FLASH_BASE, size, value)
            }
            a if (FMC_BASE..FMC_BASE + 0x1000).contains(&a) => {
                self.fmc.write(a - FMC_BASE, size, value)
            }
            a if (QUADSPI_BASE..QUADSPI_BASE + 0x1000).contains(&a) => {
                self.quadspi.write(a - QUADSPI_BASE, size, value)
            }
            a if (EXTI_BASE..EXTI_BASE + 0x400).contains(&a) => {
                self.exti.write(a - EXTI_BASE, size, value)
            }
            a if (SYSCFG_BASE..SYSCFG_BASE + 0x400).contains(&a) => {
                self.syscfg.write(a - SYSCFG_BASE, size, value)
            }
            a if (GPIO_BASE..GPIO_BASE + 0x400 * GPIO_PORTS as u32).contains(&a) => {
                let port = ((a - GPIO_BASE) / 0x400) as usize;
                self.gpio[port].write(a & 0x3FF, size, value)
            }
            a if (RCC_BASE..RCC_BASE + 0x400).contains(&a) => {
                self.rcc.write(a - RCC_BASE, size, value)
            }
            a if (PWR_BASE..PWR_BASE + 0x400).contains(&a) => {
                self.pwr.write(a - PWR_BASE, size, value)
            }
            a if (HSEM_BASE..HSEM_BASE + 0x400).contains(&a) => {
                if self.hsem.first(a - HSEM_BASE) {
                    self.note("HSEM write", a);
                }
                self.hsem.write(a - HSEM_BASE, size, value)
            }
            a if (DBGMCU_BASE..DBGMCU_BASE + 0x400).contains(&a) => {
                if a & 0x3FF != 0 {
                    self.dbgmcu.write(a - DBGMCU_BASE, size, value)
                }
            }
            a if (0x4000_0000..0x6000_0000).contains(&a) => {
                let base = a & !0x3FF;
                let first = self.other.entry(base).or_default().first(a & 0x3FF);
                if first {
                    self.note("write", a);
                }
                if let Some(regs) = self.other.get_mut(&base) {
                    regs.write(a & 0x3FF, size, value);
                }
            }
            _ => return false,
        }
        true
    }
}
