//! DSI host (RM0399 section 35): the wrapper's regulator and PLL, ready as
//! soon as they are enabled; the D-PHY in its stop state once enabled; the
//! generic packet interface, whose FIFOs are never full: each packet goes to
//! the device on the link (the board's panel, [`DsiDevice`]) when its header
//! is written, and the answer of a read waits in the payload read FIFO. The
//! board reads [`Dsi::streaming`] to know whether the LTDC's frames reach the
//! panel (video mode).
//!
//! Not modelled: adapted command mode, the interrupts, the timings of the
//! link (packets take no time), errors on the link.

use std::collections::VecDeque;

use super::regfile::{RegFile, sub_read};

const VR: u32 = 0x00;
const CR: u32 = 0x04;
const MCR: u32 = 0x34;
const GHCR: u32 = 0x6C;
const GPDR: u32 = 0x70;
const GPSR: u32 = 0x74;
const PCTLR: u32 = 0xA0;
const PSR: u32 = 0xB0;
const ISR1: u32 = 0xC0;
const WCFGR: u32 = 0x400;
const WCR: u32 = 0x404;
const WISR: u32 = 0x40C;
const WIFCR: u32 = 0x410;
const WRPCR: u32 = 0x430;

/// VR: the version of the DSI host core.
const VERSION: u32 = 0x3133_302A;
/// WRPCR: PLL enable, regulator enable.
const PLLEN: u32 = 1;
const REGEN: u32 = 1 << 24;
/// WISR: PLL locked, regulator ready.
const PLLLS: u32 = 1 << 8;
const RRS: u32 = 1 << 12;
/// GPSR: command FIFO empty, payload write FIFO empty, payload read FIFO
/// empty.
const CMDFE: u32 = 1;
const PWRFE: u32 = 1 << 2;
const PRDFE: u32 = 1 << 4;
/// PSR: PHY down, stop states of the clock lane and data lanes 0 and 1.
const PD: u32 = 1 << 1;
const PSS: u32 = (1 << 2) | (1 << 4) | (1 << 7);
/// ISR1: low-power reception timeout (a read without an answer).
const TOLPRX: u32 = 1 << 1;

/// Packet data types (MIPI DSI): set maximum return packet size, and the
/// long packets the host sends.
const SET_MAXIMUM_RETURN_PACKET_SIZE: u8 = 0x37;
const READS: [u8; 4] = [0x04, 0x14, 0x24, 0x06];

fn is_long(data_type: u8) -> bool {
    matches!(data_type, 0x09 | 0x19 | 0x29 | 0x39)
}

/// A device on the DSI link.
pub trait DsiDevice: Send {
    /// A packet on virtual channel `channel`: its data type, and its
    /// payload (the two data bytes of a short packet, the bytes of a long
    /// one). Returns the answer of a read.
    fn packet(&mut self, data_type: u8, channel: u8, payload: &[u8]) -> Option<Vec<u8>>;
}

pub struct Dsi {
    regs: RegFile,
    /// The payload write FIFO, bytes in order.
    payload: Vec<u8>,
    /// The payload read FIFO.
    answer: VecDeque<u8>,
    max_return: usize,
    isr1: u32,
    pub device: Option<Box<dyn DsiDevice>>,
    /// Packets that went nowhere, for the board's log.
    pub notes: Vec<String>,
}

impl Dsi {
    pub fn new() -> Self {
        Self {
            regs: RegFile::with(&[(VR, VERSION)]),
            payload: Vec::new(),
            answer: VecDeque::new(),
            max_return: 1,
            isr1: 0,
            device: None,
            notes: Vec::new(),
        }
    }

    /// Whether the LTDC's frames go to the panel: the host and its wrapper
    /// enabled, in video mode.
    pub fn streaming(&self) -> bool {
        self.regs.get(CR) & 1 != 0
            && self.regs.get(WCR) & (1 << 3) != 0
            && self.regs.get(MCR) & 1 == 0
            && self.regs.get(WCFGR) & 1 == 0
    }

    /// The color coding of the video stream (LCOLCR.COLC).
    pub fn color_coding(&self) -> u32 {
        self.regs.get(0x10) & 0xF
    }

    pub fn read(&mut self, offset: u32, size: u32) -> u32 {
        let word = match offset & !3 {
            GPDR => {
                let mut bytes = [0; 4];
                for b in &mut bytes {
                    *b = self.answer.pop_front().unwrap_or(0);
                }
                u32::from_le_bytes(bytes)
            }
            GPSR => {
                let mut gpsr = CMDFE;
                if self.payload.is_empty() {
                    gpsr |= PWRFE;
                }
                if self.answer.is_empty() {
                    gpsr |= PRDFE;
                }
                gpsr
            }
            PSR => {
                if self.regs.get(PCTLR) & 6 == 6 {
                    PSS
                } else {
                    PD
                }
            }
            ISR1 => std::mem::take(&mut self.isr1),
            WISR => {
                let wrpcr = self.regs.get(WRPCR);
                let mut wisr = self.regs.get(WISR);
                if wrpcr & REGEN != 0 {
                    wisr |= RRS;
                    if wrpcr & PLLEN != 0 {
                        wisr |= PLLLS;
                    }
                }
                wisr
            }
            o => self.regs.get(o),
        };
        sub_read(word, offset, size)
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32) {
        match offset & !3 {
            VR | GPSR | PSR | ISR1 | WISR => {}
            GPDR => {
                let word = super::regfile::sub_write(0, offset, size, value);
                self.payload
                    .extend_from_slice(&word.to_le_bytes()[..size as usize]);
            }
            GHCR => self.header(super::regfile::sub_write(0, offset, size, value)),
            WIFCR => {
                let wisr = self.regs.get(WISR) & !value;
                self.regs.set(WISR, wisr);
            }
            _ => self.regs.write(offset, size, value),
        }
    }

    /// A packet header written: the packet goes out.
    fn header(&mut self, ghcr: u32) {
        let data_type = (ghcr & 0x3F) as u8;
        let channel = ((ghcr >> 6) & 3) as u8;
        let (lsb, msb) = ((ghcr >> 8) as u8, (ghcr >> 16) as u8);
        let payload = if is_long(data_type) {
            let count = usize::from(u16::from_le_bytes([lsb, msb]));
            if self.payload.len() < count {
                self.notes.push(format!(
                    "DSI: long packet {data_type:#04x} of {count} bytes with {} in the FIFO",
                    self.payload.len()
                ));
            }
            let mut bytes = std::mem::take(&mut self.payload);
            bytes.resize(count, 0);
            bytes
        } else {
            vec![lsb, msb]
        };
        if self.regs.get(CR) & 1 == 0 {
            self.notes.push(format!(
                "DSI: packet {data_type:#04x} while the host is disabled: not sent"
            ));
            return;
        }
        if data_type == SET_MAXIMUM_RETURN_PACKET_SIZE {
            self.max_return = usize::from(u16::from_le_bytes([lsb, msb])).max(1);
            return;
        }
        let answer = self
            .device
            .as_mut()
            .and_then(|d| d.packet(data_type, channel, &payload));
        if READS.contains(&data_type) {
            match answer {
                Some(mut bytes) => {
                    bytes.truncate(self.max_return);
                    self.answer.extend(bytes);
                }
                None => self.isr1 |= TOLPRX,
            }
        }
    }
}

impl Default for Dsi {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;

    /// The packets a device got: data type and payload.
    type Packets = Arc<Mutex<Vec<(u8, Vec<u8>)>>>;

    #[derive(Default)]
    struct Recorder(Packets);

    impl DsiDevice for Recorder {
        fn packet(&mut self, data_type: u8, _: u8, payload: &[u8]) -> Option<Vec<u8>> {
            self.0.lock().unwrap().push((data_type, payload.to_vec()));
            (data_type == 0x06 && payload[0] == 0xDA).then(|| vec![0x40, 0x41])
        }
    }

    #[test]
    fn packets_reach_the_device_and_reads_answer() {
        let log = Arc::new(Mutex::new(Vec::new()));
        let mut dsi = Dsi::new();
        dsi.device = Some(Box::new(Recorder(log.clone())));
        dsi.write(WRPCR, 4, REGEN);
        assert_eq!(dsi.read(WISR, 4) & (RRS | PLLLS), RRS);
        dsi.write(WRPCR, 4, REGEN | PLLEN | (100 << 2) | (5 << 11));
        assert_eq!(dsi.read(WISR, 4) & PLLLS, PLLLS);

        // nothing goes out before the host is enabled
        dsi.write(GHCR, 4, 0x15 | (0x11 << 8));
        assert!(log.lock().unwrap().is_empty());
        dsi.write(CR, 4, 1);

        // a DCS long write of 5 bytes: the command and 4 parameters
        dsi.write(GPDR, 4, u32::from_le_bytes([0x2A, 0x00, 0x00, 0x03]));
        assert_eq!(dsi.read(GPSR, 4) & PWRFE, 0);
        dsi.write(GPDR, 4, 0x1F);
        dsi.write(GHCR, 4, 0x39 | (5 << 8));
        assert_eq!(dsi.read(GPSR, 4) & (CMDFE | PWRFE), CMDFE | PWRFE);
        assert_eq!(
            log.lock().unwrap().last().unwrap(),
            &(0x39, vec![0x2A, 0x00, 0x00, 0x03, 0x1F])
        );

        // a DCS read of 1 byte
        dsi.write(GHCR, 4, 0x37 | (1 << 8));
        dsi.write(GHCR, 4, 0x06 | (0xDA << 8));
        assert_eq!(dsi.read(GPSR, 4) & PRDFE, 0);
        assert_eq!(dsi.read(GPDR, 4), 0x40);
        assert_eq!(dsi.read(GPSR, 4) & PRDFE, PRDFE);

        // a read without an answer times out
        dsi.write(GHCR, 4, 0x06 | (0x0A << 8));
        assert_eq!(dsi.read(GPSR, 4) & PRDFE, PRDFE);
        assert_eq!(dsi.read(ISR1, 4), TOLPRX);
    }
}
