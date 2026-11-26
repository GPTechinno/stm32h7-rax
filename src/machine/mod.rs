//! The chip: the Cortex-M7 on the bus, booted from an ELF image, run in
//! virtual time.
//!
//! Time advances by one cycle of the core clock (from the RCC) per
//! instruction. While the core sleeps (WFI, WFE), time jumps to the next
//! timer event or to the end of the run. Between slices of instructions the
//! timers advance, their interrupt lines reach the NVIC (as levels: an
//! interrupt is pended while its line is high and it is neither pending nor
//! active), and the Ethernet DMA moves its frames.

mod bus;

use std::sync::{Arc, Mutex, MutexGuard};

use goblin::elf::{Elf, program_header::PT_LOAD};
use rax::isa::arm::common::cpu::{ArmCpu, CpuExit};
use rax::isa::arm::common::memory::ArmMemory;
use rax::isa::arm::cortex_m::ppb::CacheIdentification;
use rax::isa::arm::cortex_m::{CortexMCpu, CortexMVariant};

use bus::Bus;
pub use bus::{FLASH_BASE, FLASH_SIZE};

use crate::devices::eth::{DmaDone, DmaJob, RI, TI, fcs};
use crate::devices::ltdc::Frame;
use crate::devices::quadspi::NorFlash;
use crate::devices::{Clock, Peripherals};

/// Instructions between two looks at the peripherals.
const SLICE: u32 = 256;
const PS_PER_S: u64 = 1_000_000_000_000;

/// The STM32H7's caches: 16 KB 2-way for instructions, 16 KB 4-way for
/// data, 32-byte lines. CCSIDR: write-through, write-back, read- and
/// write-allocate (bits 31:28), sets - 1 (27:13), ways - 1 (12:3),
/// log2(words per line) - 2 (2:0).
const CACHES: CacheIdentification = CacheIdentification {
    clidr: 0x0900_0003,
    ctr: 0x8303_C003,
    ccsidr_data: 0xF000_0000 | (127 << 13) | (3 << 3) | 1,
    ccsidr_instruction: 0xF000_0000 | (255 << 13) | (1 << 3) | 1,
};

pub struct Config {
    /// The HSE crystal or oscillator, Hz.
    pub hse_hz: u32,
    pub uid: [u8; 12],
    /// RAM on the FMC: (base address, bytes), the board's SDRAM.
    pub external_ram: Vec<(u32, usize)>,
    /// The flash on the QUADSPI, and whether it is two chips side by side
    /// (dual-flash mode).
    pub qspi_flash: Vec<u8>,
    pub qspi_dual: bool,
    /// Where the Cortex-M7 boots: its vector table.
    pub boot_address: u32,
    /// Seed of the RNG.
    pub seed: u64,
}

/// Why a run stopped early.
#[derive(Debug, Clone)]
pub enum Event {
    /// A BKPT instruction, with its immediate.
    Breakpoint { pc: u32, imm: u32 },
    /// An instruction the core reports instead of taking a fault: undefined
    /// or unimplemented, a bus error, a lockup.
    Fault { pc: u32, message: String },
    /// IWDG1 resets the system: its counter reached zero, or it was reloaded
    /// outside its window. The board restarts the chip, keeping what its
    /// memories keep, with the flags of [`RESET_IWDG1`](crate::devices::rcc::RESET_IWDG1).
    WatchdogReset { pc: u32 },
}

pub struct Stm32h7 {
    cpu: CortexMCpu,
    periph: Arc<Mutex<Peripherals>>,
    clock: Arc<Clock>,
    boot_address: u32,
    /// Picoseconds.
    now: u64,
    ps_per_cycle: u64,
    next_event: Option<u64>,
    pub instructions: u64,
    /// Time spent asleep, picoseconds.
    pub asleep: u64,
}

impl Stm32h7 {
    /// The chip with `elf` loaded (its loadable segments, at their physical
    /// addresses), reset.
    pub fn new(cfg: Config, elf: &[u8]) -> Result<Self, String> {
        let clock = Arc::new(Clock::default());
        let flash = NorFlash::new(cfg.qspi_flash, cfg.qspi_dual);
        let periph = Arc::new(Mutex::new(Peripherals::new(
            clock.clone(),
            cfg.hse_hz,
            flash,
            cfg.seed,
        )));
        let mut bus = Bus::new(periph.clone(), &cfg.external_ram, cfg.uid);
        let image = Elf::parse(elf).map_err(|e| format!("ELF: {e}"))?;
        for ph in image
            .program_headers
            .iter()
            .filter(|ph| ph.p_type == PT_LOAD)
        {
            if ph.p_filesz == 0 {
                continue;
            }
            let start = ph.p_offset as usize;
            let bytes = elf
                .get(start..start + ph.p_filesz as usize)
                .ok_or("ELF: segment beyond the file")?;
            if !bus.load(ph.p_paddr as u32, bytes) {
                return Err(format!(
                    "ELF: segment of {} bytes at {:#010x} outside the memories",
                    bytes.len(),
                    ph.p_paddr
                ));
            }
        }
        let mut cpu = CortexMCpu::new(CortexMVariant::CortexM7, Box::new(bus));
        cpu.set_floating_point(true);
        cpu.map_private_peripheral_bus(true);
        cpu.set_cache_identification(CACHES);
        let mut machine = Self {
            cpu,
            periph,
            clock,
            boot_address: cfg.boot_address,
            now: 0,
            ps_per_cycle: 1,
            next_event: None,
            instructions: 0,
            asleep: 0,
        };
        machine.reset();
        Ok(machine)
    }

    /// Resets the core, which boots from the vector table at the boot
    /// address (the reset value of VTOR).
    pub fn reset(&mut self) {
        self.cpu.set_initial_vtor(self.boot_address);
        self.cpu.reset();
        self.service();
    }

    pub fn periph(&self) -> MutexGuard<'_, Peripherals> {
        self.periph.lock().unwrap_or_else(|e| e.into_inner())
    }

    pub fn cpu(&self) -> &CortexMCpu {
        &self.cpu
    }

    /// Virtual time, picoseconds.
    pub fn now(&self) -> u64 {
        self.now
    }

    pub fn pc(&self) -> u32 {
        self.cpu.get_pc() as u32
    }

    /// R0-R12, SP, LR, PC.
    pub fn registers(&self) -> [u32; 16] {
        let mut regs = [0; 16];
        for (i, r) in regs.iter_mut().enumerate().take(13) {
            *r = self.cpu.get_gpr(i as u8) as u32;
        }
        regs[13] = self.cpu.get_sp() as u32;
        regs[14] = self.cpu.get_lr() as u32;
        regs[15] = self.pc();
        regs
    }

    pub fn read_bytes(&self, address: u32, len: usize) -> Option<Vec<u8>> {
        let mut buf = vec![0; len];
        self.cpu
            .memory()
            .read(u64::from(address), &mut buf)
            .ok()
            .map(|()| buf)
    }

    pub fn read_u32(&self, address: u32) -> Option<u32> {
        let bytes = self.read_bytes(address, 4)?;
        Some(u32::from_le_bytes(bytes.try_into().ok()?))
    }

    pub fn write_u32(&mut self, address: u32, value: u32) -> bool {
        self.cpu
            .memory_mut()
            .write(u64::from(address), &value.to_le_bytes())
            .is_ok()
    }

    /// Runs for `duration` picoseconds of virtual time, or until an event.
    pub fn run_for(&mut self, duration: u64) -> Option<Event> {
        let end = self.now.saturating_add(duration);
        while self.now < end {
            let target = self.next_event.map_or(end, |t| t.clamp(self.now + 1, end));
            let mut slice = 0;
            while self.now < target && slice < SLICE {
                let pc = self.pc();
                self.clock
                    .pc
                    .store(pc, std::sync::atomic::Ordering::Relaxed);
                match self.cpu.step() {
                    Ok(CpuExit::Wfi | CpuExit::Wfe) => {
                        let wake = target.max(self.now + self.ps_per_cycle);
                        self.asleep += wake - self.now;
                        self.now = wake;
                        break;
                    }
                    Ok(CpuExit::Breakpoint(imm)) => return Some(Event::Breakpoint { pc, imm }),
                    Ok(_) => {}
                    Err(e) => {
                        return Some(Event::Fault {
                            pc,
                            message: e.to_string(),
                        });
                    }
                }
                self.now += self.ps_per_cycle;
                self.instructions += 1;
                slice += 1;
                self.clock
                    .now
                    .store(self.now, std::sync::atomic::Ordering::Relaxed);
            }
            self.clock
                .now
                .store(self.now, std::sync::atomic::Ordering::Relaxed);
            self.service();
            if self.periph().iwdg.expired {
                return Some(Event::WatchdogReset { pc: self.pc() });
            }
        }
        None
    }

    /// Advances the peripherals, raises their interrupts, runs the DMA.
    fn service(&mut self) {
        let periph = self.periph.clone();
        let (lines, cpu_hz, job) = {
            let mut p = periph.lock().unwrap_or_else(|e| e.into_inner());
            self.next_event = p.update(self.now);
            (p.irq_lines(), p.cpu_clock(), p.eth.dma_job())
        };
        self.ps_per_cycle = (PS_PER_S / cpu_hz.max(1)).max(1);
        if let Some(job) = job {
            let done = eth_dma(self.cpu.memory_mut(), job);
            self.periph().eth.dma_done(done);
        }
        let nvic = self.cpu.nvic_mut();
        for irq in lines {
            if !nvic.is_pending(irq) && !nvic.is_active(irq) {
                nvic.set_pending(irq);
            }
        }
    }

    /// The LTDC's picture, composed from the frame buffers in memory; `None`
    /// while the LTDC is off.
    pub fn ltdc_frame(&self) -> Option<Frame> {
        let composition = self.periph().ltdc.composition()?;
        let memory = self.cpu.memory();
        Some(composition.compose(&mut |address, buf| memory.read(u64::from(address), buf).is_ok()))
    }

    /// The bytes waiting in RTT up buffer `channel` of the control block at
    /// `control_block`, which are then consumed.
    pub fn rtt_read(&mut self, control_block: u32, channel: u32) -> Vec<u8> {
        // Control block: id[16], max_up, max_down, then up buffers of 24
        // bytes: name, buffer, size, write offset, read offset, flags.
        let Some(max_up) = self.read_u32(control_block + 16) else {
            return Vec::new();
        };
        if channel >= max_up {
            return Vec::new();
        }
        let desc = control_block + 24 + 24 * channel;
        let field = |m: &Self, i: u32| m.read_u32(desc + 4 * i).unwrap_or(0);
        let (buf, size, wr, rd) = (
            field(self, 1),
            field(self, 2),
            field(self, 3),
            field(self, 4),
        );
        if size == 0 || wr >= size || rd >= size || wr == rd {
            return Vec::new();
        }
        let out = if wr > rd {
            self.read_bytes(buf + rd, (wr - rd) as usize)
                .unwrap_or_default()
        } else {
            let mut out = self
                .read_bytes(buf + rd, (size - rd) as usize)
                .unwrap_or_default();
            out.extend(self.read_bytes(buf, wr as usize).unwrap_or_default());
            out
        };
        self.write_u32(desc + 16, wr);
        out
    }
}

/// Moves frames between the descriptor rings in memory and the MAC.
fn eth_dma(memory: &mut dyn ArmMemory, job: DmaJob) -> DmaDone {
    const OWN: u32 = 1 << 31;
    const FD: u32 = 1 << 29;
    const LD: u32 = 1 << 28;
    const TDES2_IOC: u32 = 1 << 31;
    const RDES3_IOC: u32 = 1 << 30;
    let word = |m: &dyn ArmMemory, a: u32| {
        let mut b = [0u8; 4];
        m.read(u64::from(a), &mut b)
            .ok()
            .map(|()| u32::from_le_bytes(b))
    };
    let mut done = DmaDone::default();

    if let Some((list, entries, mut next)) = job.tx {
        let mut frame = Vec::new();
        for _ in 0..entries {
            let desc = list + 16 * next;
            let Some(tdes3) = word(memory, desc + 12) else {
                break;
            };
            if tdes3 & OWN == 0 {
                break;
            }
            let (buf, tdes2) = (
                word(memory, desc).unwrap_or(0),
                word(memory, desc + 8).unwrap_or(0),
            );
            let mut bytes = vec![0; (tdes2 & 0x3FFF) as usize];
            if memory.read(u64::from(buf), &mut bytes).is_err() {
                break;
            }
            if tdes3 & FD != 0 {
                frame.clear();
            }
            frame.extend_from_slice(&bytes);
            if tdes3 & LD != 0 {
                done.sent.push(std::mem::take(&mut frame));
            }
            let _ = memory.write(u64::from(desc + 12), &(tdes3 & (FD | LD)).to_le_bytes());
            if tdes2 & TDES2_IOC != 0 {
                done.status |= TI;
            }
            next = (next + 1) % entries;
        }
        done.tx_next = Some(next);
    }

    if let Some((list, entries, mut next, buffer_size)) = job.rx {
        let mut frames = job.frames.into_iter();
        while let Some(frame) = frames.next() {
            let desc = list + 16 * next;
            let rdes3 = word(memory, desc + 12).unwrap_or(0);
            if rdes3 & OWN == 0 {
                // suspended until the firmware gives a descriptor back
                done.waiting.push(frame);
                done.waiting.extend(frames);
                break;
            }
            let mut bytes = frame;
            let check = fcs(&bytes);
            bytes.extend_from_slice(&check);
            bytes.truncate(buffer_size as usize);
            let buf = word(memory, desc).unwrap_or(0);
            if memory.write(u64::from(buf), &bytes).is_err() {
                done.dropped += 1;
                continue;
            }
            let _ = memory.write(u64::from(desc + 4), &0u32.to_le_bytes());
            let _ = memory.write(u64::from(desc + 8), &0u32.to_le_bytes());
            let status = FD | LD | (bytes.len() as u32 & 0x7FFF);
            let _ = memory.write(u64::from(desc + 12), &status.to_le_bytes());
            if rdes3 & RDES3_IOC != 0 {
                done.status |= RI;
            }
            done.received += 1;
            next = (next + 1) % entries;
        }
        done.rx_next = Some(next);
    }
    done
}

#[cfg(test)]
mod tests {
    use rax::isa::arm::common::memory::FlatMemory;

    use super::*;
    use crate::devices::eth::{Eth, RBU, RI};

    const RAM: u32 = 0x2400_0000;
    const RING: u32 = RAM;
    const BUFFERS: u32 = RAM + 0x1000;
    /// RDES3 of a descriptor given to the DMA: OWN, IOC, BUF1V.
    const READY: u32 = (1 << 31) | (1 << 30) | (1 << 24);

    fn give(memory: &mut FlatMemory, i: u32) {
        let desc = u64::from(RING + 16 * i);
        memory
            .write(desc, &(BUFFERS + 0x800 * i).to_le_bytes())
            .unwrap();
        memory.write(desc + 12, &READY.to_le_bytes()).unwrap();
    }

    fn owned_by_dma(memory: &FlatMemory, i: u32) -> bool {
        let mut b = [0; 4];
        memory.read(u64::from(RING + 16 * i + 12), &mut b).unwrap();
        u32::from_le_bytes(b) & (1 << 31) != 0
    }

    /// A MAC receiving into a ring of `n` descriptors, its interrupts enabled.
    fn receiving(n: u32) -> Eth {
        let mut eth = Eth::new();
        eth.write(0x0000, 4, 1); // MACCR.RE
        eth.write(0x111C, 4, RING); // DMACRXDLAR
        eth.write(0x1130, 4, n - 1); // DMACRXRLR
        eth.write(0x1134, 4, (1 << 15) | (1 << 14) | RBU | RI); // DMACIER
        eth.write(0x1108, 4, (1536 << 1) | 1); // DMACRXCR: buffer size, start
        eth
    }

    fn service(eth: &mut Eth, memory: &mut FlatMemory) {
        if let Some(job) = eth.dma_job() {
            let done = eth_dma(memory, job);
            eth.dma_done(done);
        }
    }

    #[test]
    fn frames_wait_for_a_free_descriptor() {
        let mut memory = FlatMemory::new(u64::from(RAM), 0x4000);
        let mut eth = receiving(2);
        give(&mut memory, 0);
        give(&mut memory, 1);
        eth.incoming.extend((0..5u8).map(|i| vec![i; 60]));

        service(&mut eth, &mut memory);
        assert!(!owned_by_dma(&memory, 0) && !owned_by_dma(&memory, 1));
        assert_eq!(
            eth.incoming.len(),
            3,
            "the frames without a descriptor wait"
        );
        assert_eq!(eth.incoming[0][0], 2, "in order");
        assert_eq!(eth.read(0x1160, 4) & (RI | RBU), RI | RBU);
        assert_eq!(eth.dropped, 0);

        // RBU is raised once per suspension
        eth.write(0x1160, 4, RI | RBU | (1 << 15) | (1 << 14));
        service(&mut eth, &mut memory);
        assert_eq!(eth.read(0x1160, 4), 0);
        assert!(!eth.irq());

        // the firmware gives descriptor 0 back: the next frame goes there
        give(&mut memory, 0);
        service(&mut eth, &mut memory);
        assert!(!owned_by_dma(&memory, 0));
        assert_eq!(eth.incoming.len(), 2);
        let mut first = [0; 1];
        memory.read(u64::from(BUFFERS), &mut first).unwrap();
        assert_eq!(first[0], 2);
        assert_eq!(eth.read(0x1160, 4) & (RI | RBU), RI | RBU);
        assert_eq!(eth.dropped, 0);
    }
}
