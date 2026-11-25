//! QUADSPI and the serial NOR flash on it (the MT25Q command set, which the
//! common JEDEC commands share).
//!
//! In dual-flash mode (DFM) the two chips are one flash here: the QUADSPI
//! sends each chip half the address and takes the even bytes from the
//! first, the odd ones from the second, so one byte array in the combined
//! address order holds both, and a chip's page (256 bytes), subsector (4 KB)
//! and sector (64 KB) are twice as large in it. A register read returns the
//! register of each chip in turn.
//!
//! Indirect mode: a command starts when its CCR is written (without
//! address or, in read mode, without data), when AR is written (with an
//! address), or once its data is in (write mode); it then completes at once
//! (TCF). Memory-mapped mode reads the flash through the window at
//! `0x9000_0000`. Auto-polling matches at once. Programs and erases take no
//! time: the chips are never busy.

use std::collections::VecDeque;

use super::regfile::{sub_read, sub_write};

const CR: u32 = 0x00;
const DCR: u32 = 0x04;
const SR: u32 = 0x08;
const FCR: u32 = 0x0C;
const DLR: u32 = 0x10;
const CCR: u32 = 0x14;
const AR: u32 = 0x18;
const ABR: u32 = 0x1C;
const DR: u32 = 0x20;
const PSMKR: u32 = 0x24;
const PSMAR: u32 = 0x28;
const PIR: u32 = 0x2C;
const LPTR: u32 = 0x30;

const CR_ABORT: u32 = 1 << 1;
const CR_DFM: u32 = 1 << 6;
const SR_TCF: u32 = 1 << 1;
const SR_SMF: u32 = 1 << 3;
const SR_BUSY: u32 = 1 << 5;

const FMODE_WRITE: u32 = 0;
const FMODE_READ: u32 = 1;
const FMODE_POLL: u32 = 2;
const FMODE_MAPPED: u32 = 3;

/// The serial NOR flash: one chip, or two side by side in dual-flash mode.
pub struct NorFlash {
    pub bytes: Vec<u8>,
    /// Two chips side by side.
    pub dual: bool,
    write_enabled: bool,
    reset_enabled: bool,
    /// Programs and erases since the start, and the mistakes a real chip
    /// would not report: programs without write enable, bits set back to 1,
    /// programs wrapping in their page.
    pub programs: u64,
    pub erases: u64,
    pub warnings: Vec<String>,
}

impl NorFlash {
    pub fn new(bytes: Vec<u8>, dual: bool) -> Self {
        Self {
            bytes,
            dual,
            write_enabled: false,
            reset_enabled: false,
            programs: 0,
            erases: 0,
            warnings: Vec::new(),
        }
    }

    fn scale(&self, size: usize) -> usize {
        if self.dual { 2 * size } else { size }
    }

    fn erase(&mut self, address: u32, size: usize) {
        if !self.write_enabled {
            self.warnings.push(format!(
                "erase at {address:#x} without write enable: ignored"
            ));
            return;
        }
        let size = self.scale(size).min(self.bytes.len());
        let start = (address as usize % self.bytes.len()) / size * size;
        self.bytes[start..start + size].fill(0xFF);
        self.erases += 1;
        self.write_enabled = false;
    }

    fn program(&mut self, address: u32, data: &[u8]) {
        if !self.write_enabled {
            self.warnings.push(format!(
                "program at {address:#x} without write enable: ignored"
            ));
            return;
        }
        let page = self.scale(256);
        let address = address as usize % self.bytes.len();
        let base = address / page * page;
        if address % page + data.len() > page {
            self.warnings.push(format!(
                "program of {} bytes at {address:#x} wraps in its {page}-byte page",
                data.len()
            ));
        }
        let mut set_back = 0;
        for (i, &byte) in data.iter().enumerate() {
            let at = base + (address - base + i) % page;
            if self.bytes[at] & byte != byte {
                set_back += 1;
            }
            self.bytes[at] &= byte;
        }
        if set_back > 0 {
            self.warnings.push(format!(
                "program at {address:#x}: {set_back} bytes need an erase first"
            ));
        }
        self.programs += 1;
        self.write_enabled = false;
    }

    /// Runs `instruction` with its address and data; returns `len` bytes
    /// read.
    pub fn command(
        &mut self,
        instruction: u8,
        address: Option<u32>,
        data: &[u8],
        len: usize,
    ) -> Vec<u8> {
        let reset_enabled = std::mem::take(&mut self.reset_enabled);
        let address = address.unwrap_or(0);
        match instruction {
            0x66 => self.reset_enabled = true,
            0x99 if reset_enabled => self.write_enabled = false,
            0x06 => self.write_enabled = true,
            0x04 => self.write_enabled = false,
            // status: WEL, never WIP
            0x05 => return vec![u8::from(self.write_enabled) << 1; len],
            // flag status: ready
            0x70 => return vec![0x80; len],
            0x50 | 0xB7 | 0xE9 => {}
            // JEDEC ID of an MT25QL512: 0x20, 0xBA, 0x20, of each chip in turn
            0x9F => {
                let id = [0x20, 0xBA, 0x20];
                let per = if self.dual { 2 } else { 1 };
                return (0..len).map(|i| id[(i / per) % 3]).collect();
            }
            0x03 | 0x0B | 0x13 | 0x0C | 0x3B | 0x3C | 0x6B | 0x6C | 0xBB | 0xBC | 0xEB | 0xEC => {
                return (0..len)
                    .map(|i| self.bytes[(address as usize + i) % self.bytes.len()])
                    .collect();
            }
            0x02 | 0x12 | 0x32 | 0x34 | 0x38 | 0x3E => self.program(address, data),
            0x20 | 0x21 => self.erase(address, 4 << 10),
            0x52 | 0x5C => self.erase(address, 32 << 10),
            0xD8 | 0xDC => self.erase(address, 64 << 10),
            0x60 | 0xC7 | 0xC4 => {
                let len = self.bytes.len();
                self.erase(0, len / self.scale(1));
            }
            other => self
                .warnings
                .push(format!("command {other:#04x} not modelled")),
        }
        vec![0; len]
    }
}

pub struct Quadspi {
    cr: u32,
    dcr: u32,
    flags: u32,
    dlr: u32,
    ccr: u32,
    ar: u32,
    abr: u32,
    psmkr: u32,
    psmar: u32,
    pir: u32,
    lptr: u32,
    fifo: VecDeque<u8>,
    /// Data of an indirect write being collected.
    collecting: Option<Vec<u8>>,
    pub flash: NorFlash,
}

impl Quadspi {
    pub fn new(flash: NorFlash) -> Self {
        Self {
            cr: 0,
            dcr: 0,
            flags: 0,
            dlr: 0,
            ccr: 0,
            ar: 0,
            abr: 0,
            psmkr: 0,
            psmar: 0,
            pir: 0,
            lptr: 0,
            fifo: VecDeque::new(),
            collecting: None,
            flash,
        }
    }

    fn fmode(&self) -> u32 {
        (self.ccr >> 26) & 3
    }

    fn has_address(&self) -> bool {
        (self.ccr >> 10) & 3 != 0
    }

    fn has_data(&self) -> bool {
        (self.ccr >> 24) & 3 != 0
    }

    fn instruction(&self) -> u8 {
        self.ccr as u8
    }

    /// In memory-mapped mode.
    pub fn mapped(&self) -> bool {
        self.fmode() == FMODE_MAPPED
    }

    fn data_len(&self) -> usize {
        self.dlr as usize + 1
    }

    /// Starts the command of CCR and AR. Without an instruction, address or
    /// data phase there is nothing to do.
    fn start(&mut self) {
        if (self.ccr >> 8) & 3 == 0 && !self.has_address() && !self.has_data() {
            return;
        }
        let address = self.has_address().then_some(self.ar);
        match self.fmode() {
            FMODE_READ => {
                let len = if self.has_data() { self.data_len() } else { 0 };
                let bytes = self.flash.command(self.instruction(), address, &[], len);
                self.fifo.extend(bytes);
                self.flags |= SR_TCF;
            }
            FMODE_WRITE if self.has_data() => self.collecting = Some(Vec::new()),
            FMODE_WRITE => {
                self.flash.command(self.instruction(), address, &[], 0);
                self.flags |= SR_TCF;
            }
            FMODE_POLL => self.flags |= SR_SMF | SR_TCF,
            _ => {}
        }
    }

    fn abort(&mut self) {
        self.collecting = None;
        self.fifo.clear();
        self.ccr &= !(3 << 26);
    }

    /// A read of the memory-mapped window at `offset`, if it is mapped.
    pub fn mapped_read(&self, offset: u32, buf: &mut [u8]) -> bool {
        if !self.mapped() {
            return false;
        }
        let len = self.flash.bytes.len();
        for (i, byte) in buf.iter_mut().enumerate() {
            *byte = self.flash.bytes[(offset as usize + i) % len];
        }
        true
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            CR => self.cr,
            DCR => self.dcr,
            SR => {
                let busy = if self.collecting.is_some() {
                    SR_BUSY
                } else {
                    0
                };
                let level = (self.fifo.len().min(32) as u32) << 8;
                self.flags | busy | level | if self.fifo.is_empty() { 0 } else { 1 << 2 }
            }
            DLR => self.dlr,
            CCR => self.ccr,
            AR => self.ar,
            ABR => self.abr,
            DR => {
                let mut word = 0;
                for i in 0..size {
                    word |= u32::from(self.fifo.pop_front().unwrap_or(0)) << (8 * i);
                }
                return word;
            }
            PSMKR => self.psmkr,
            PSMAR => self.psmar,
            PIR => self.pir,
            LPTR => self.lptr,
            _ => 0,
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        let reg = |word: u32| sub_write(word, offset, size, value);
        match offset & !3 {
            CR => {
                let cr = reg(self.cr);
                if cr & CR_ABORT != 0 {
                    self.abort();
                }
                self.cr = cr & !CR_ABORT;
                if self.cr & CR_DFM != 0 && !self.flash.dual {
                    self.flash
                        .warnings
                        .push("dual-flash mode on a single flash".into());
                }
            }
            DCR => self.dcr = reg(self.dcr),
            FCR => self.flags &= !(reg(0) & 0x1B),
            DLR => self.dlr = reg(self.dlr),
            CCR => {
                self.ccr = reg(self.ccr);
                self.collecting = None;
                let mode = self.fmode();
                let starts = !self.has_address()
                    && (mode == FMODE_READ
                        || mode == FMODE_POLL
                        || !self.has_data()
                        || mode == FMODE_WRITE);
                if mode != FMODE_MAPPED && starts {
                    self.start();
                }
            }
            AR => {
                self.ar = reg(self.ar);
                if self.has_address() && self.fmode() != FMODE_MAPPED {
                    self.start();
                }
            }
            ABR => self.abr = reg(self.abr),
            DR => {
                let len = self.data_len();
                if let Some(data) = &mut self.collecting {
                    for i in 0..size {
                        data.push((value >> (8 * i)) as u8);
                    }
                    if data.len() >= len {
                        let data = self.collecting.take().unwrap_or_default();
                        let address = self.has_address().then_some(self.ar);
                        self.flash
                            .command(self.instruction(), address, &data[..len], 0);
                        self.flags |= SR_TCF;
                    }
                }
            }
            PSMKR => self.psmkr = reg(self.psmkr),
            PSMAR => self.psmar = reg(self.psmar),
            PIR => self.pir = reg(self.pir),
            LPTR => self.lptr = reg(self.lptr),
            _ => {}
        }
    }
}
