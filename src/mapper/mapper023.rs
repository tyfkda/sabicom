use serde::{Deserialize, Serialize};
use std::collections::HashMap;

use crate::{
    consts::{LINES_PER_FRAME, PPU_CLOCK_PER_LINE},
    context::IrqSource,
    rom::Mirroring,
};

const IRQ_ENABLE_AFTER: u8 = 1 << 0;
const IRQ_ENABLE: u8 = 1 << 1;
const IRQ_MODE: u8 = 1 << 2;

#[derive(Serialize, Deserialize)]
pub struct Mapper023 {
    prg_bank_mode: u8,
    prg_bank: [u8; 4],
    chr_bank: [u16; 8],
    irq_control: u8,

    mapping: HashMap<u8, u8>,

    irq_latch: u8,
    irq_counter: u8,
    irq_reload: bool,
    irq_enable: bool,
    ppu_cycle: u64,
    ppu_line: u64,
    // ppu_frame: u64,
    ppu_bus_addr: u16,
    ppu_a12_edge: bool,
}

impl Mapper023 {
    pub fn new(ctx: &mut impl super::Context) -> Self {
        let mut mapping = HashMap::new();
        mapping.insert(0x00, 0);
        mapping.insert(0x04, 2);
        mapping.insert(0x08, 4);
        mapping.insert(0x0c, 6);
        mapping.insert(0x01, 2);
        mapping.insert(0x02, 4);
        mapping.insert(0x03, 6);

        let ret = Self {
            prg_bank_mode: 0,
            prg_bank: [0; 4],
            chr_bank: [0; 8],
            irq_control: 0,
            // irqLatch: 0,
            // irqCounter: 0,
            mapping,

            irq_latch: 0,
            irq_counter: 0,
            irq_reload: false,
            irq_enable: false,
            ppu_cycle: 0,
            ppu_line: 0,
            // ppu_frame: 0,
            ppu_bus_addr: 0,
            ppu_a12_edge: false,
        };

        let prg_pages = ctx.memory_ctrl().prg_pages();
        ctx.map_prg(0, 0);
        ctx.map_prg(1, 1);
        ctx.map_prg(2, prg_pages - 2);
        ctx.map_prg(3, prg_pages - 1);

        ret
    }

    fn set_prg_bank(&mut self, ctx: &mut impl super::Context, bank: u32, value: u8) {
        self.prg_bank[bank as usize] = value;
        ctx.map_prg(bank, value as u32);
    }

    fn set_chr_bank_offset(&mut self, ctx: &mut impl super::Context, bank: u32, value: u16) {
        self.chr_bank[bank as usize] = value;
        ctx.map_chr(bank, value.into());
    }

    fn update_ppu_addr(&mut self, addr: u16) {
        if addr >= 0x2000 {
            return;
        }

        if self.ppu_bus_addr & 0x1000 == 0 && addr & 0x1000 != 0 {
            self.ppu_a12_edge = true;
        }

        self.ppu_bus_addr = addr;
    }
}

impl super::MapperTrait for Mapper023 {
    fn write_prg(&mut self, ctx: &mut impl super::Context, addr: u16, data: u8) {
        if addr & 0x8000 == 0 {
            ctx.write_prg(addr, data);
            return;
        }

        match addr {
            0x8000 ..= 0x8006 => {
                if self.prg_bank_mode == 0 {
                    self.set_prg_bank(ctx, 0, data);
                } else {
                    self.set_prg_bank(ctx, 2, data);
                }
            }
            0x9000 ..= 0x90ff => {
                let reg = self.mapping[&((addr & 0xff) as u8)];
                if reg == 0 || reg == 2 {  // Mirroring Control.
                    let mirroring = match data & 3 {
                        0 => Mirroring::Vertical,
                        1 => Mirroring::Horizontal,
                        2 => Mirroring::OneScreenLow,
                        3 => Mirroring::OneScreenHigh,
                        _ => panic!("Invalid mirroring: {}", data & 3),
                    };
                    ctx.memory_ctrl_mut().set_mirroring(mirroring);
                } else if reg == 4 || reg == 6 {  // PRG Swap Mode control.
                    self.prg_bank_mode = (data >> 1) & 1;
                    let prg_pages = ctx.memory_ctrl().prg_pages() as u8;
                    match self.prg_bank_mode {
                        0 => {
                            self.set_prg_bank(ctx, 2, prg_pages - 2);
                        }
                        1 => {
                            self.set_prg_bank(ctx, 0, prg_pages - 2);
                        }
                        _ => {}
                    }
                }
            }
            0xa000 ..= 0xa006 => {
                self.set_prg_bank(ctx, 1, data);
            }
            0xb000 ..= 0xb0ff => {
                let reg = self.mapping[&((addr & 0xff) as u8)];
                let data = data as u16;
                if reg == 0 {  // CHR Select 0
                    self.set_chr_bank_offset(ctx, 0, (self.chr_bank[0] & !0x0f) | (data & 0x0f));
                } else if reg == 2 {
                    self.set_chr_bank_offset(ctx, 0, (self.chr_bank[0] & !0x1f0) | ((data & 0x1f) << 4));
                } else if reg == 4 {  // CHR Select 1
                    self.set_chr_bank_offset(ctx, 1, (self.chr_bank[1] & !0x0f) | (data & 0x0f));
                } else if reg == 6 {
                    self.set_chr_bank_offset(ctx, 1, (self.chr_bank[1] & !0x1f0) | ((data & 0x1f) << 4));
                }
            }
            0xc000 ..= 0xefff => {
                let reg = self.mapping[&((addr & 0xff) as u8)];
                let ofs: u16;
                let hi: bool;
                (ofs, hi) = if reg == 0 {
                    (0, false)
                } else if reg == 2 {
                    (0, true)
                } else if reg == 4 {
                    (1, false)
                } else if reg == 6 {
                    (1, true)
                } else {
                    return;
                };
                let data = data as u16;
                let bank = ((addr & 0x3000) >> 11) + 2 + ofs;
                let value = if hi {
                  (self.chr_bank[bank as usize] & !0x1f0) | ((data & 0x1f) << 4)
                } else {
                  (self.chr_bank[bank as usize] & !0x0f) | (data & 0x0f)
                };
                self.set_chr_bank_offset(ctx, bank as u32, value);
            }
            0xf000 ..= 0xffff => {
                let reg = self.mapping[&((addr & 0xff) as u8)];
                if reg == 0 {  // IRQ Latch: low 4 bits
                    self.irq_latch = (self.irq_latch & !0x0f) | (data & 0x0f);
                } else if reg == 2 {  // IRQ Latch: high 4 bits
                    self.irq_latch = (self.irq_latch & !0xf0) | ((data & 0x0f) << 4);
                } else if reg == 4 {  // IRQ Control
                    self.irq_control = data;
                    self.irq_enable = (data & IRQ_ENABLE) != 0;
                    if self.irq_enable {
                        self.irq_counter = self.irq_latch;
                    }
                    ctx.set_irq_source(IrqSource::Mapper, false);
                } else if reg == 6 {  // IRQ Acknowledge
                    // Copy to enable
                    let ea = self.irq_control & IRQ_ENABLE_AFTER;
                    self.irq_control = (self.irq_control & !IRQ_ENABLE) | (ea << 1);
                    self.irq_enable = (data & IRQ_ENABLE) != 0;
                    ctx.set_irq_source(IrqSource::Mapper, false);
                }
            }
            _ => {}
        }
    }

    fn read_chr(&mut self, ctx: &mut impl super::Context, addr: u16) -> u8 {
        self.update_ppu_addr(addr);
        ctx.read_chr(addr)
    }

    fn write_chr(&mut self, ctx: &mut impl super::Context, addr: u16, data: u8) {
        self.update_ppu_addr(addr);
        ctx.write_chr(addr, data);
    }

    fn tick(&mut self, ctx: &mut impl super::Context) {
        let mut newline = false;
        self.ppu_cycle += 1;
        if self.ppu_cycle == PPU_CLOCK_PER_LINE {
            self.ppu_cycle = 0;
            self.ppu_line += 1;
            newline = true;
            if self.ppu_line == LINES_PER_FRAME as u64 {
                self.ppu_line = 0;
                // self.ppu_frame += 1;
            }
        }

        if (self.irq_control & IRQ_ENABLE) != 0 {
            if (self.irq_control & IRQ_MODE) != 0 || newline {
                // Clock IRQ Counter.
                if self.irq_counter >= 255 {
                    self.irq_counter = self.irq_latch;
                    ctx.set_irq_source(IrqSource::Mapper, true);
                } else {
                    self.irq_counter += 1;
                }
            }
        }
    }
}
