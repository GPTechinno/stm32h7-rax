//! Ethernet MAC: the registers, MDIO to the board's PHY, and the state of
//! the DMA channel; the machine moves the frames between the descriptor
//! rings in memory and the queues here ([`Eth::dma_job`],
//! [`Eth::dma_done`]).
//!
//! Descriptors are the 16-byte normal descriptors (RM0399 section 58.10):
//! TX read format TDES0 buffer 1, TDES2 buffer 1 length and IOC, TDES3 OWN,
//! FD, LD; RX read format RDES0 buffer 1, RDES3 OWN, IOC, BUF1V; RX
//! write-back RDES3 FD, LD and the packet length, which counts the 4-byte
//! FCS the MAC keeps on type frames (CST clear). Not modelled: buffer 2,
//! context descriptors, checksum offload, the MTL queues, the MMC counters
//! (they keep what is written), descriptor skip lengths.

use std::collections::VecDeque;

use super::regfile::{RegFile, sub_read};

const MACCR: u32 = 0x0000;
const MACMDIOAR: u32 = 0x0200;
const MACMDIODR: u32 = 0x0204;
const DMAMR: u32 = 0x1000;
const DMACTXCR: u32 = 0x1104;
const DMACRXCR: u32 = 0x1108;
const DMACTXDLAR: u32 = 0x1114;
const DMACRXDLAR: u32 = 0x111C;
const DMACTXDTPR: u32 = 0x1120;
const DMACRXDTPR: u32 = 0x1128;
const DMACTXRLR: u32 = 0x112C;
const DMACRXRLR: u32 = 0x1130;
const DMACIER: u32 = 0x1134;
const DMACCATXDR: u32 = 0x1144;
const DMACCARXDR: u32 = 0x114C;
const DMACSR: u32 = 0x1160;

const MACCR_RE: u32 = 1 << 0;
const MACCR_TE: u32 = 1 << 1;
const DMA_START: u32 = 1;
const MDIO_BUSY: u32 = 1 << 0;
/// DMACSR / DMACIER bits: TI, RI, RBU (receive buffer unavailable), NIS,
/// AIS.
pub const TI: u32 = 1 << 0;
pub const RI: u32 = 1 << 6;
pub const RBU: u32 = 1 << 7;
const AIS: u32 = 1 << 14;
const NIS: u32 = 1 << 15;

/// Clause 22 MDIO to the PHYs on the board.
pub trait Mdio: Send {
    /// Register `reg` of the PHY at `phy`; all ones without one there.
    fn read(&mut self, phy: u8, reg: u8) -> u16;
    fn write(&mut self, phy: u8, reg: u8, value: u16);
}

/// What the machine does with memory for the DMA.
pub struct DmaJob {
    /// TX ring (list address, entries) and the next descriptor, while
    /// transmission runs.
    pub tx: Option<(u32, u32, u32)>,
    /// RX ring (list address, entries), next descriptor, buffer size, and the
    /// frames to receive, while reception runs.
    pub rx: Option<(u32, u32, u32, u32)>,
    pub frames: Vec<Vec<u8>>,
}

/// What the DMA did.
#[derive(Default)]
pub struct DmaDone {
    pub tx_next: Option<u32>,
    pub rx_next: Option<u32>,
    /// Frames transmitted.
    pub sent: Vec<Vec<u8>>,
    /// Frames not written, their buffer outside memory.
    pub dropped: usize,
    /// Frames received.
    pub received: usize,
    /// Frames waiting for a free descriptor: reception is suspended.
    pub waiting: Vec<Vec<u8>>,
    /// DMACSR bits to set.
    pub status: u32,
}

pub struct Eth {
    regs: RegFile,
    pub phy: Option<Box<dyn Mdio>>,
    tx_next: u32,
    rx_next: u32,
    status: u32,
    /// Transmission to look at: the tail pointer was written.
    tx_kick: bool,
    /// Frames from the board, waiting for reception. With no free descriptor
    /// reception suspends and they wait: none is lost (the link partner keeps
    /// them, as a switch port does).
    pub incoming: VecDeque<Vec<u8>>,
    /// Reception suspended for lack of a free descriptor (RBU was raised).
    rx_suspended: bool,
    /// Frames from the chip, for the board.
    pub outgoing: VecDeque<Vec<u8>>,
    pub dropped: u64,
}

impl Default for Eth {
    fn default() -> Self {
        Self::new()
    }
}

impl Eth {
    pub fn new() -> Self {
        Self {
            // MACCR reset: 0x8000; MACVR: 0x4142 (Synopsys version 4.2)
            regs: RegFile::with(&[(MACCR, 0x0000_8000), (0x0110, 0x0000_4142)]),
            phy: None,
            tx_next: 0,
            rx_next: 0,
            status: 0,
            tx_kick: false,
            incoming: VecDeque::new(),
            rx_suspended: false,
            outgoing: VecDeque::new(),
            dropped: 0,
        }
    }

    pub fn irq(&self) -> bool {
        let enabled = self.regs.get(DMACIER);
        let normal = self.status & enabled & (TI | RI) != 0 && enabled & NIS != 0;
        let abnormal = self.status & enabled & RBU != 0 && enabled & AIS != 0;
        normal || abnormal
    }

    fn summary(status: u32) -> u32 {
        let mut status = status;
        if status & (TI | RI) != 0 {
            status |= NIS;
        }
        if status & RBU != 0 {
            status |= AIS;
        }
        status
    }

    fn transmitting(&self) -> bool {
        self.regs.get(MACCR) & MACCR_TE != 0 && self.regs.get(DMACTXCR) & DMA_START != 0
    }

    fn receiving(&self) -> bool {
        self.regs.get(MACCR) & MACCR_RE != 0 && self.regs.get(DMACRXCR) & DMA_START != 0
    }

    /// Work for the DMA, if any.
    pub fn dma_job(&mut self) -> Option<DmaJob> {
        let tx = (self.transmitting() && self.tx_kick).then(|| {
            (
                self.regs.get(DMACTXDLAR),
                (self.regs.get(DMACTXRLR) & 0x3FF) + 1,
                self.tx_next,
            )
        });
        let rx = (self.receiving() && !self.incoming.is_empty()).then(|| {
            (
                self.regs.get(DMACRXDLAR),
                (self.regs.get(DMACRXRLR) & 0x3FF) + 1,
                self.rx_next,
                (self.regs.get(DMACRXCR) >> 1) & 0x3FFF,
            )
        });
        if tx.is_none() && rx.is_none() {
            return None;
        }
        self.tx_kick = false;
        let frames = if rx.is_some() {
            self.incoming.drain(..).collect()
        } else {
            Vec::new()
        };
        Some(DmaJob { tx, rx, frames })
    }

    pub fn dma_done(&mut self, done: DmaDone) {
        if let Some(next) = done.tx_next {
            self.tx_next = next;
        }
        if let Some(next) = done.rx_next {
            self.rx_next = next;
        }
        self.outgoing.extend(done.sent);
        self.dropped += done.dropped as u64;
        self.status |= Self::summary(done.status);
        if done.rx_next.is_some() {
            let suspended = !done.waiting.is_empty();
            // RBU on entering the suspended state only
            if suspended && (!self.rx_suspended || done.received > 0) {
                self.status |= Self::summary(RBU);
            }
            self.rx_suspended = suspended;
            for frame in done.waiting.into_iter().rev() {
                self.incoming.push_front(frame);
            }
        }
    }

    fn reset(&mut self) {
        let phy = self.phy.take();
        let incoming = std::mem::take(&mut self.incoming);
        *self = Self::new();
        self.phy = phy;
        self.incoming = incoming;
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            // SWR clears once the reset is done, at once
            DMAMR => self.regs.get(DMAMR) & !1,
            DMACSR => self.status,
            DMACCATXDR => self.regs.get(DMACTXDLAR) + 16 * self.tx_next,
            DMACCARXDR => self.regs.get(DMACRXDLAR) + 16 * self.rx_next,
            o => self.regs.get(o),
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        match offset & !3 {
            DMAMR if value & 1 != 0 => self.reset(),
            DMACSR => self.status &= !value,
            MACMDIOAR => {
                self.regs.write(offset, size, value);
                let ar = self.regs.get(MACMDIOAR);
                if ar & MDIO_BUSY != 0 {
                    let phy = ((ar >> 21) & 0x1F) as u8;
                    let reg = ((ar >> 16) & 0x1F) as u8;
                    match ((ar >> 2) & 3, &mut self.phy) {
                        (0b01, Some(p)) => p.write(phy, reg, self.regs.get(MACMDIODR) as u16),
                        (0b11, Some(p)) => {
                            let v = p.read(phy, reg);
                            self.regs.set(MACMDIODR, u32::from(v));
                        }
                        (0b11, None) => self.regs.set(MACMDIODR, 0xFFFF),
                        _ => {}
                    }
                    self.regs.set(MACMDIOAR, ar & !MDIO_BUSY);
                }
            }
            DMACTXDLAR => {
                self.regs.write(offset, size, value);
                self.tx_next = 0;
            }
            DMACRXDLAR => {
                self.regs.write(offset, size, value);
                self.rx_next = 0;
            }
            DMACTXDTPR => {
                self.regs.write(offset, size, value);
                self.tx_kick = true;
            }
            DMACRXDTPR => self.regs.write(offset, size, value),
            DMACTXCR => {
                self.regs.write(offset, size, value);
                self.tx_kick = true;
            }
            _ => self.regs.write(offset, size, value),
        }
    }
}

/// The Ethernet FCS (CRC-32, IEEE) of `frame`, as the MAC appends it.
pub fn fcs(frame: &[u8]) -> [u8; 4] {
    let mut crc = !0u32;
    for &byte in frame {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xEDB8_8320 & (crc & 1).wrapping_neg());
        }
    }
    (!crc).to_le_bytes()
}
