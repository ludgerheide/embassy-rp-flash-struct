use defmt::{error, info};
use embassy_rp::flash::{ERASE_SIZE, Flash};
use embassy_rp::peripherals::{DMA_CH3, FLASH};

// Taken from example
const ADDR_OFFSET: u32 = 0x100000;
const FLASH_SIZE: usize = 2 * 1024 * 1024;
const BLOCK_COUNT: u32 = 100;

const MAX_VALUE_PER_BLOCK: u32 = 100000;

pub struct FlashCounter<'d> {
    active_block_idx: u32,
    flash: Flash<'d, FLASH, embassy_rp::flash::Async, FLASH_SIZE>,
}

impl<'d> FlashCounter<'d> {
    fn init_flash(
        peripheral_flash: FLASH,
        dma_channel: DMA_CH3,
    ) -> Flash<'d, FLASH, embassy_rp::flash::Async, FLASH_SIZE> {
        embassy_rp::flash::Flash::<'d, FLASH, embassy_rp::flash::Async, FLASH_SIZE>::new(
            peripheral_flash,
            dma_channel,
        )
    }

    pub fn new(peripheral_flash: FLASH, dma_channel: DMA_CH3) -> Self {
        let mut flash = Self::init_flash(peripheral_flash, dma_channel);
        // When we are setting off, every block from the start (at OFFSET) to the block
        // count should be purely zeroes, except the first four bytes
        // If not, we risk corrupting program memory!
        info!("Validating Flash space…");
        let mut active_block_idx = 0;
        for block_index in 0..BLOCK_COUNT {
            let mut buf = [0u8; 4];
            for inner_offset in 0..(ERASE_SIZE as u32) / 4 {
                let addr_to_read = ADDR_OFFSET
                    + (block_index * ERASE_SIZE as u32)
                    + (inner_offset * buf.len() as u32);
                flash.blocking_read(addr_to_read, &mut buf).unwrap();
                let value = u32::from_le_bytes(buf);
                if inner_offset == 0 {
                    if block_index == 0 && value == 0xFFFFFFFF {
                        info!("Initializing flash by writing 0 to the first block!");
                        let to_write = (0 as u32).to_le_bytes();
                        flash.blocking_write(addr_to_read, &to_write).unwrap();
                    } else if value == MAX_VALUE_PER_BLOCK {
                        active_block_idx += 1
                    }
                } else if value != 0xFFFFFFFF {
                    error!("Flash block seems to be in use!");
                    panic!();
                }
            }
        }
        Self {
            flash: flash,
            active_block_idx: active_block_idx,
        }
    }

    pub fn read_counter(&mut self) -> u32 {
        let addr_to_read = ADDR_OFFSET + (self.active_block_idx * ERASE_SIZE as u32);
        let mut buf = [0u8; 4];
        self.flash.blocking_read(addr_to_read, &mut buf).unwrap();
        u32::from_le_bytes(buf) + (self.active_block_idx * MAX_VALUE_PER_BLOCK)
    }

    pub fn increment_counter(&mut self) {
        let addr_to_read = ADDR_OFFSET + (self.active_block_idx * ERASE_SIZE as u32);
        let mut buf = [0u8; 4];
        self.flash.blocking_read(addr_to_read, &mut buf).unwrap();
        let current_value = u32::from_le_bytes(buf);
        info!(
            "Current active block idx {:?}, current value {:?}",
            self.active_block_idx, current_value
        );
        if current_value < MAX_VALUE_PER_BLOCK {
            let addr_to_write = addr_to_read;
            let new_value = current_value + 1;
            let to_write = new_value.to_le_bytes();
            self.flash
                .blocking_erase(addr_to_write, addr_to_write + ERASE_SIZE as u32)
                .unwrap();
            self.flash.blocking_write(addr_to_write, &to_write).unwrap();
            info!(
                "Value set to {:?} effectively: {:?}",
                new_value,
                new_value + (self.active_block_idx * MAX_VALUE_PER_BLOCK)
            );
        } else {
            self.active_block_idx += 1;
            let addr_to_write = ADDR_OFFSET + (self.active_block_idx * ERASE_SIZE as u32);
            let new_value: u32 = 0;
            let to_write = new_value.to_le_bytes();
            self.flash
                .blocking_erase(addr_to_write, addr_to_write + ERASE_SIZE as u32)
                .unwrap();
            self.flash.blocking_write(addr_to_write, &to_write).unwrap();
            info!(
                "Increment active block to {:?} and set value to {:?}",
                self.active_block_idx, new_value
            );
        }
    }
}
