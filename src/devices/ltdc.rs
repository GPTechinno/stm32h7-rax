//! LTDC (RM0399 section 34): the timing registers, the two layers behind
//! their shadow registers (taken on a reload, at once or at the next
//! vertical blanking), the reload interrupt, and the current position from
//! virtual time and the pixel clock. The picture is composed on demand from
//! the frame buffers in memory: [`Ltdc::composition`] takes what it needs
//! from the registers, [`Composition::compose`] reads the memory.
//!
//! Not modelled: the line interrupt, color keying, dithering, the FIFO
//! underrun and transfer error flags.

use super::regfile::{RegFile, sub_read, sub_write};

const SSCR: u32 = 0x08;
const BPCR: u32 = 0x0C;
const AWCR: u32 = 0x10;
const TWCR: u32 = 0x14;
const GCR: u32 = 0x18;
const SRCR: u32 = 0x24;
const BCCR: u32 = 0x2C;
const IER: u32 = 0x34;
const ISR: u32 = 0x38;
const ICR: u32 = 0x3C;
const CPSR: u32 = 0x44;
const CDSR: u32 = 0x48;
const LAYER: u32 = 0x84;
const LAYER_SIZE: u32 = 0x80;
const LAYERS: usize = 2;
/// The layer registers, by word in a layer (their offset over 4): CR 0x00,
/// WHPCR 0x04, WVPCR 0x08, PFCR 0x10, CACR 0x14, DCCR 0x18, BFCR 0x1C,
/// CFBAR 0x28, CFBLR 0x2C, CFBLNR 0x30.
const L_CR: usize = 0;
const L_WHPCR: usize = 1;
const L_WVPCR: usize = 2;
const L_PFCR: usize = 4;
const L_CACR: usize = 5;
const L_DCCR: usize = 6;
const L_BFCR: usize = 7;
const L_CFBAR: usize = 10;
const L_CFBLR: usize = 11;
const L_CFBLNR: usize = 12;
const L_CLUTWR: u32 = 0x40;
const LAYER_WORDS: usize = 0x40 / 4;

/// GCR at reset: data widths of 8 bits per color.
const GCR_RESET: u32 = 0x2220;
const LTDCEN: u32 = 1;
const IMR: u32 = 1;
const VBR: u32 = 2;
/// ISR: register reload.
const RRIF: u32 = 1 << 3;
/// LxCR: layer enable, CLUT enable.
const LEN: u32 = 1;
const CLUTEN: u32 = 1 << 4;

const PS_PER_S: u128 = 1_000_000_000_000;

/// A picture: `width` × `height` pixels, RGBA, row by row.
#[derive(Clone, Default)]
pub struct Frame {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

pub struct Ltdc {
    regs: RegFile,
    /// The layer registers as written, and as used.
    shadow: [[u32; LAYER_WORDS]; LAYERS],
    active: [[u32; LAYER_WORDS]; LAYERS],
    clut: [[u32; 256]; LAYERS],
    isr: u32,
    /// The time of a reload asked for at the next vertical blanking.
    reload_at: Option<u64>,
    /// When the LTDC was enabled: the start of its first frame.
    started: Option<u64>,
    /// The pixel clock, Hz, as last given to [`Ltdc::update`].
    pixel_hz: u64,
    /// Reloads done, to notice a new picture.
    pub reloads: u64,
}

impl Ltdc {
    pub fn new() -> Self {
        Self {
            regs: RegFile::with(&[(GCR, GCR_RESET)]),
            shadow: [[0; LAYER_WORDS]; LAYERS],
            active: [[0; LAYER_WORDS]; LAYERS],
            clut: [[0; 256]; LAYERS],
            isr: 0,
            reload_at: None,
            started: None,
            pixel_hz: 0,
            reloads: 0,
        }
    }

    fn enabled(&self) -> bool {
        self.regs.get(GCR) & LTDCEN != 0
    }

    /// Total width and height of a frame, in pixel clocks and lines.
    fn totals(&self) -> (u64, u64) {
        let twcr = self.regs.get(TWCR);
        (
            u64::from((twcr >> 16) & 0xFFF) + 1,
            u64::from(twcr & 0x7FF) + 1,
        )
    }

    /// `clocks` of the pixel clock, picoseconds.
    fn clocks_ps(&self, clocks: u64) -> u64 {
        (u128::from(clocks) * PS_PER_S / u128::from(self.pixel_hz.max(1))) as u64
    }

    /// A frame's length, and the vertical blanking's start in it (the first
    /// line after the active area), picoseconds; `None` without a pixel
    /// clock.
    fn frame_ps(&self) -> Option<(u64, u64)> {
        let (w, h) = self.totals();
        let blanking = u64::from(self.regs.get(AWCR) & 0x7FF) + 1;
        let frame = self.clocks_ps(w * h);
        (self.pixel_hz > 0 && frame > 0).then(|| (frame, self.clocks_ps(w * blanking.min(h))))
    }

    /// The position in the frame at `now`: pixel clock and line.
    fn position(&self, now: u64) -> Option<(u64, u64)> {
        let started = self.started?;
        let (w, h) = self.totals();
        let clocks = u128::from(now.saturating_sub(started)) * u128::from(self.pixel_hz) / PS_PER_S;
        let pixel = (clocks % u128::from(w * h)) as u64;
        Some((pixel % w, pixel / w))
    }

    fn reload(&mut self) {
        self.active = self.shadow;
        self.reload_at = None;
        self.isr |= RRIF;
        self.reloads += 1;
    }

    /// Advances to `now` with the pixel clock `pixel_hz`; returns the time of
    /// the next event (a reload at vertical blanking).
    pub fn update(&mut self, now: u64, pixel_hz: u64) -> Option<u64> {
        self.pixel_hz = pixel_hz;
        if self.reload_at.is_some_and(|t| t <= now) {
            self.reload();
        }
        self.reload_at
    }

    pub fn irq(&self) -> bool {
        self.isr & self.regs.get(IER) & 0xF != 0
    }

    pub fn read(&mut self, offset: u32, size: u32, now: u64) -> u32 {
        let word = match offset & !3 {
            SRCR => {
                if self.reload_at.is_some() {
                    VBR
                } else {
                    0
                }
            }
            ISR => self.isr,
            CPSR => self
                .position(now)
                .map_or(0, |(x, y)| ((x as u32) << 16) | y as u32),
            CDSR => self.position(now).map_or(0, |(x, y)| {
                let (sscr, bpcr, awcr) = (
                    self.regs.get(SSCR),
                    self.regs.get(BPCR),
                    self.regs.get(AWCR),
                );
                let (x, y) = (x as u32, y as u32);
                let vdes = y > bpcr & 0x7FF && y <= awcr & 0x7FF;
                let hdes = x > (bpcr >> 16) & 0xFFF && x <= (awcr >> 16) & 0xFFF;
                let vsyncs = y <= sscr & 0x7FF;
                let hsyncs = x <= (sscr >> 16) & 0xFFF;
                u32::from(vdes)
                    | (u32::from(hdes) << 1)
                    | (u32::from(vsyncs) << 2)
                    | (u32::from(hsyncs) << 3)
            }),
            o if (LAYER..LAYER + LAYER_SIZE * LAYERS as u32).contains(&o) => {
                let (layer, index) = Self::layer_word(o);
                match index {
                    Some(i) => self.shadow[layer][i],
                    None => 0,
                }
            }
            o => self.regs.get(o),
        };
        sub_read(word, offset, size)
    }

    /// The layer and the word in it of a layer register's offset; `None`
    /// for CLUTWR and the reserved words.
    fn layer_word(offset: u32) -> (usize, Option<usize>) {
        let layer = ((offset - LAYER) / LAYER_SIZE) as usize;
        let inner = (offset - LAYER) % LAYER_SIZE;
        (layer, (inner < L_CLUTWR).then_some((inner / 4) as usize))
    }

    pub fn write(&mut self, offset: u32, size: u32, value: u32, now: u64) {
        let o = offset & !3;
        match o {
            SRCR => {
                let srcr = sub_write(0, offset, size, value);
                if srcr & IMR != 0 || (srcr & VBR != 0 && !self.enabled()) {
                    self.reload();
                } else if srcr & VBR != 0 {
                    match (self.frame_ps(), self.started) {
                        (Some((frame, blanking)), Some(started)) => {
                            // the next start of a vertical blanking
                            let first = started + blanking;
                            let frames = now.saturating_sub(first) / frame + 1;
                            let next = if now < first {
                                first
                            } else {
                                first + frames * frame
                            };
                            self.reload_at = Some(next);
                        }
                        // no pixel clock: no blanking to wait for
                        _ => self.reload(),
                    }
                }
            }
            ICR => self.isr &= !sub_write(0, offset, size, value),
            ISR | CPSR | CDSR => {}
            GCR => {
                let was = self.enabled();
                self.regs.write(offset, size, value);
                match (was, self.enabled()) {
                    (false, true) => self.started = Some(now),
                    (true, false) => self.started = None,
                    _ => {}
                }
            }
            o if (LAYER..LAYER + LAYER_SIZE * LAYERS as u32).contains(&o) => {
                let (layer, index) = Self::layer_word(o);
                match index {
                    Some(i) => {
                        self.shadow[layer][i] =
                            sub_write(self.shadow[layer][i], offset, size, value)
                    }
                    None if (o - LAYER) % LAYER_SIZE == L_CLUTWR => {
                        let entry = sub_write(0, offset, size, value);
                        self.clut[layer][(entry >> 24) as usize] = entry & 0xFF_FFFF;
                    }
                    None => {}
                }
            }
            _ => self.regs.write(offset, size, value),
        }
    }

    /// What composing the picture needs, `None` while the LTDC is off.
    pub fn composition(&self) -> Option<Composition> {
        if !self.enabled() {
            return None;
        }
        let (bpcr, awcr) = (self.regs.get(BPCR), self.regs.get(AWCR));
        let (ahbp, avbp) = ((bpcr >> 16) & 0xFFF, bpcr & 0x7FF);
        let (aaw, aah) = ((awcr >> 16) & 0xFFF, awcr & 0x7FF);
        let (width, height) = (aaw.checked_sub(ahbp)?, aah.checked_sub(avbp)?);
        if width == 0 || height == 0 {
            return None;
        }
        Some(Composition {
            width,
            height,
            ahbp,
            avbp,
            background: self.regs.get(BCCR) & 0xFF_FFFF,
            layers: self.active,
            clut: self.clut,
        })
    }
}

impl Default for Ltdc {
    fn default() -> Self {
        Self::new()
    }
}

/// The active registers of the LTDC, to compose a picture from memory.
#[derive(Clone)]
pub struct Composition {
    width: u32,
    height: u32,
    ahbp: u32,
    avbp: u32,
    background: u32,
    layers: [[u32; LAYER_WORDS]; LAYERS],
    clut: [[u32; 256]; LAYERS],
}

/// Bytes per pixel of a pixel format (LxPFCR).
fn bytes_per_pixel(format: u32) -> usize {
    match format {
        0 => 4,
        1 => 3,
        2..=4 | 7 => 2,
        _ => 1,
    }
}

/// 5 or 6 bits to 8.
fn expand(value: u32, bits: u32) -> u32 {
    (value << (8 - bits)) | (value >> (2 * bits - 8))
}

impl Composition {
    /// The picture, the layers blended over the background color in order,
    /// their pixels read with `read(address, buffer)` (false on a failed
    /// read, which leaves the layer's default color).
    pub fn compose(&self, read: &mut dyn FnMut(u32, &mut [u8]) -> bool) -> Frame {
        let (w, h) = (self.width as usize, self.height as usize);
        let bg = self.background;
        let mut rgb: Vec<u32> = vec![bg; w * h];
        let mut line = Vec::new();
        for (layer, regs) in self.layers.iter().enumerate() {
            if regs[L_CR] & LEN == 0 {
                continue;
            }
            let (whpcr, wvpcr) = (regs[L_WHPCR], regs[L_WVPCR]);
            // window positions count from the start of the line, sync included
            let x0 = (whpcr & 0xFFF).saturating_sub(self.ahbp + 1) as usize;
            let x1 = (((whpcr >> 16) & 0xFFF).saturating_sub(self.ahbp) as usize).min(w);
            let y0 = (wvpcr & 0x7FF).saturating_sub(self.avbp + 1) as usize;
            let y1 = (((wvpcr >> 16) & 0x7FF).saturating_sub(self.avbp) as usize).min(h);
            if x0 >= x1 || y0 >= y1 {
                continue;
            }
            let format = regs[L_PFCR] & 7;
            let bpp = bytes_per_pixel(format);
            let pitch = ((regs[L_CFBLR] >> 16) & 0x1FFF) as usize;
            let lines = (regs[L_CFBLNR] & 0x7FF) as usize;
            let constant = regs[L_CACR] & 0xFF;
            let default = regs[L_DCCR];
            let bfcr = regs[L_BFCR];
            let pixel_alpha = (bfcr >> 8) & 7 == 6;
            let clut = (regs[L_CR] & CLUTEN != 0).then_some(&self.clut[layer]);
            for y in y0..y1 {
                let row = y - y0;
                line.resize((x1 - x0) * bpp, 0);
                let loaded = row < lines
                    && read(regs[L_CFBAR].wrapping_add((row * pitch) as u32), &mut line);
                for x in x0..x1 {
                    let argb = if loaded {
                        let i = (x - x0) * bpp;
                        decode(format, &line[i..i + bpp], clut)
                    } else {
                        default
                    };
                    let alpha = if pixel_alpha {
                        (argb >> 24) * constant / 255
                    } else {
                        constant
                    };
                    let below = rgb[y * w + x];
                    rgb[y * w + x] = blend(argb, below, alpha);
                }
            }
        }
        let mut rgba = Vec::with_capacity(w * h * 4);
        for p in rgb {
            rgba.extend_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, p as u8, 0xFF]);
        }
        Frame {
            width: self.width,
            height: self.height,
            rgba,
        }
    }
}

/// A pixel of `format` from its bytes, as ARGB8888.
fn decode(format: u32, b: &[u8], clut: Option<&[u32; 256]>) -> u32 {
    let lookup = |l: u32| match clut {
        Some(table) => table[l as usize],
        None => (l << 16) | (l << 8) | l,
    };
    match format {
        0 => u32::from_le_bytes([b[0], b[1], b[2], b[3]]),
        1 => 0xFF00_0000 | (u32::from(b[2]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[0]),
        2 => {
            let p = u32::from(u16::from_le_bytes([b[0], b[1]]));
            0xFF00_0000
                | (expand(p >> 11, 5) << 16)
                | (expand((p >> 5) & 0x3F, 6) << 8)
                | expand(p & 0x1F, 5)
        }
        3 => {
            let p = u32::from(u16::from_le_bytes([b[0], b[1]]));
            let a = if p & 0x8000 != 0 { 0xFF } else { 0 };
            (a << 24)
                | (expand((p >> 10) & 0x1F, 5) << 16)
                | (expand((p >> 5) & 0x1F, 5) << 8)
                | expand(p & 0x1F, 5)
        }
        4 => {
            let p = u32::from(u16::from_le_bytes([b[0], b[1]]));
            let n = |v: u32| (v & 0xF) * 0x11;
            (n(p >> 12) << 24) | (n(p >> 8) << 16) | (n(p >> 4) << 8) | n(p)
        }
        5 => 0xFF00_0000 | lookup(u32::from(b[0])),
        6 => ((u32::from(b[0] >> 4) * 0x11) << 24) | lookup(u32::from(b[0] & 0xF)),
        _ => (u32::from(b[1]) << 24) | lookup(u32::from(b[0])),
    }
}

/// `argb` over the RGB `below` with `alpha` (0 to 255).
fn blend(argb: u32, below: u32, alpha: u32) -> u32 {
    let channel = |shift: u32| {
        let (c, d) = ((argb >> shift) & 0xFF, (below >> shift) & 0xFF);
        ((c * alpha + d * (255 - alpha)) / 255) << shift
    };
    channel(16) | channel(8) | channel(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 4 × 2 pixels, with the timings the STM32H747I-DISCO's display uses:
    /// sync 2, back porch 34 horizontally; 120, 150 vertically.
    fn configured() -> Ltdc {
        let mut ltdc = Ltdc::new();
        ltdc.write(SSCR, 4, (1 << 16) | 119, 0);
        ltdc.write(BPCR, 4, (35 << 16) | 269, 0);
        ltdc.write(AWCR, 4, (39 << 16) | 271, 0);
        ltdc.write(TWCR, 4, (73 << 16) | 421, 0);
        ltdc.write(BCCR, 4, 0x00_00FF, 0);
        let layer = LAYER;
        ltdc.write(layer + 0x04, 4, (39 << 16) | 36, 0); // WHPCR: x 0..4
        ltdc.write(layer + 0x08, 4, (271 << 16) | 270, 0); // WVPCR: y 0..2
        ltdc.write(layer + 0x10, 4, 2, 0); // RGB565
        ltdc.write(layer + 0x14, 4, 0xFF, 0);
        ltdc.write(layer + 0x1C, 4, 0x0607, 0);
        ltdc.write(layer + 0x28, 4, 0xD000_0000, 0);
        ltdc.write(layer + 0x2C, 4, (8 << 16) | 15, 0);
        ltdc.write(layer + 0x30, 4, 2, 0);
        ltdc.write(layer, 4, LEN, 0);
        ltdc
    }

    #[test]
    fn layers_take_effect_on_a_reload() {
        let mut ltdc = configured();
        ltdc.write(GCR, 4, GCR_RESET | LTDCEN, 0);
        assert!(ltdc.composition().unwrap().layers[0][L_CR] & LEN == 0);
        ltdc.write(SRCR, 4, IMR, 0);
        assert_eq!(ltdc.read(ISR, 4, 0) & RRIF, RRIF);
        let frame = ltdc.composition().unwrap().compose(&mut |address, buf| {
            assert_eq!(address & !0xF, 0xD000_0000);
            // red, green, blue, white; then the second row 8 bytes on
            let row = [0xF800u16, 0x07E0, 0x001F, 0xFFFF];
            for (i, p) in row.iter().enumerate() {
                buf[2 * i..2 * i + 2].copy_from_slice(&p.to_le_bytes());
            }
            true
        });
        assert_eq!((frame.width, frame.height), (4, 2));
        let pixel = |i: usize| &frame.rgba[4 * i..4 * i + 3];
        assert_eq!(pixel(0), [0xFF, 0, 0]);
        assert_eq!(pixel(1), [0, 0xFF, 0]);
        assert_eq!(pixel(2), [0, 0, 0xFF]);
        assert_eq!(pixel(7), [0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn a_vertical_blanking_reload_waits_for_the_line_after_the_active_area() {
        let mut ltdc = configured();
        ltdc.write(GCR, 4, GCR_RESET | LTDCEN, 1_000);
        // lines of 74 clocks at 1 MHz, 422 lines a frame, the active area
        // ending with line 271
        let line = 74 * 1_000_000;
        let (frame, blanking) = (422 * line, 272 * line);
        assert_eq!(ltdc.update(5_000_000, 1_000_000), None);
        ltdc.write(SRCR, 4, VBR, 5_000_000);
        assert_eq!(ltdc.read(SRCR, 4, 5_000_000), VBR);
        assert_eq!(ltdc.update(5_000_001, 1_000_000), Some(1_000 + blanking));
        assert_eq!(ltdc.read(ISR, 4, 0) & RRIF, 0);
        assert_eq!(ltdc.update(1_000 + blanking, 1_000_000), None);
        assert_eq!(ltdc.read(SRCR, 4, 1_000 + blanking), 0);
        assert_eq!(ltdc.read(ISR, 4, 0) & RRIF, RRIF);
        ltdc.write(ICR, 4, RRIF, 0);
        assert_eq!(ltdc.read(ISR, 4, 0), 0);

        // asked for during the blanking: the next frame's
        let now = 1_000 + blanking + 10 * line;
        ltdc.write(SRCR, 4, VBR, now);
        assert_eq!(ltdc.update(now, 1_000_000), Some(1_000 + blanking + frame));
    }
}
