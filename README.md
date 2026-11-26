# stm32h7-rax

STM32H7 SoC emulator in Rust. The Cortex-M7 is [rax](https://github.com/Georges760/rax)'s Cortex-M
core (its `feat/cortex-m7` branch: the Private Peripheral Bus as memory, the Floating-point Extension
(FPv5-D16), the reset value of VTOR), around it the memory map and the peripheral models of the
STM32H747's Cortex-M7 (RM0399). Board emulators build on it: they give it the external memories, the
QSPI flash bytes, the PHY behind MDIO, the panel on the DSI link, and see the GPIO levels, the
Ethernet frames and the LTDC's picture.

## Building

rax is not on crates.io yet, so this crate takes it by path from `../../upstream/rax`: a checkout of
its `feat/cortex-m7` branch in an `upstream` directory beside the `emulator` directory holding this
crate. devenv (stable Rust).

## Use

```rust
let config = stm32h7_rax::Config {
    hse_hz: 25_000_000,
    uid: *b"ECSN-DISCO-1",
    external_ram: vec![(0xD000_0000, 32 << 20)], // SDRAM on FMC bank 2
    qspi_flash: vec![0xFF; 128 << 20],
    qspi_dual: true,
    boot_address: 0x0800_0000,
    seed: 1,
};
let mut chip = stm32h7_rax::Stm32h7::new(config, &elf)?;
let event = chip.run_for(1_000_000_000); // picoseconds: 1 ms
```

`Stm32h7::new` loads the loadable segments of an ELF at their physical addresses (the flash, the
RAMs) and resets the core, which boots from the vector table at the boot address. `run_for` runs in
virtual time: one cycle of the core clock, computed from the RCC registers, per instruction; while the
core sleeps (WFI, WFE) time jumps to the next timer event. `periph()` gives the peripherals (GPIO
inputs and outputs, the QSPI flash bytes, the Ethernet frames, the device on the DSI link, `log` and
`trace` for the accesses); `ltdc_frame` composes the LTDC's picture from its frame buffers in memory;
`rtt_read` reads a SEGGER RTT up buffer (defmt).

## Model

| Part | Status |
|---|---|
| Memories | flash (2 MB, written only by loading), ITCM, DTCM, AXI SRAM, SRAM1-4, backup SRAM, system memory with the unique ID and the flash size, external RAMs |
| RCC, PWR | registers kept, ready flags following their enables, PLL source and dividers protected while their PLL runs, the clock tree (SYSCLK, CPU, AHB, APB timer clocks, the LTDC's PLL3 R) computed from them |
| GPIO A-K | modes, outputs (ODR, BSRR), alternate functions; inputs from the board |
| TIM1-8, 12-17 | up-counting from the kernel clock and prescaler, update and compare flags, one interrupt line each (TIM1 and TIM8 without) |
| RNG | always ready, xorshift words |
| FMC | registers kept, SDRAM controller never busy, commands recorded |
| QUADSPI | indirect, memory-mapped and auto-polling modes; a serial NOR flash with the MT25Q commands (single or two chips in dual-flash mode), reporting programs without write enable, bits set back to 1, page wraps |
| ETH | MAC registers, MDIO to the board's PHY, DMA with the normal descriptors (buffer 1), the FCS appended to received frames |
| LTDC | timings, two layers behind their shadow registers (reloaded at once or at the first line after the active area, from the PLL3 R pixel clock), the reload interrupt, the current position; the picture composed from the frame buffers (every pixel format, CLUT, constant and pixel alpha blending over the background) |
| DSI host | wrapper regulator and PLL ready once enabled, the D-PHY in its stop state, generic packets to the board's device on the link (long packets from the payload FIFO, read answers to the payload read FIFO), whether video mode streams the LTDC |
| Others | registers that keep what is written; the first access to each is logged with its PC |

Not modelled: the Cortex-M4 core, caches (identified, maintenance has no effect), the MPU (stored, not
enforced), DMA controllers and DMA2D, the LTDC's line interrupt and color keying, the DSI host's
adapted command mode and link timings, the other peripherals' behaviour.
