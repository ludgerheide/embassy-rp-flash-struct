#![no_std]

use core::default::Default;
use core::marker::PhantomData;
use core::mem::size_of;

use bincode::{config, Decode, decode_from_slice, Encode, encode_into_slice};
use defmt::{assert, debug, error, info};
use embassy_rp::dma::AnyChannel;
use embassy_rp::flash::{ERASE_SIZE, Flash, PAGE_SIZE};
use embassy_rp::peripherals::FLASH;

/* CHANGE THESE FOR YOUR HARDWARE AND MEMORY LAYOUT */
const TOTAL_FLASH_SIZE: usize = 2 * 1024 * 1024;
const USED_FLASH_SIZE: usize = 1 * 1024 * 1024;

/* DO NOT CHANGE THESE */
// This is where our flash section will start, running up to the end of the flash memory
const ADDR_OFFSET: usize = TOTAL_FLASH_SIZE - USED_FLASH_SIZE;
// This is the amount of blocks we can for into the memory
const BLOCK_COUNT: u32 = USED_FLASH_SIZE as u32 / ERASE_SIZE as u32;
// How many times we write to a single block before moving to the next one
const MAX_WRITES_PER_BLOCK: u32 = 20;
// If the device is already used, we may want to start at a later value, without changing the
// USED_FLASH_SIZE
const START_BLOCK_NO: u32 = 0;
// The maximum value that our flash struct could store, if all teh blocks were fully used
const MAX_FLASH_WRITES: u32 = MAX_WRITES_PER_BLOCK * (BLOCK_COUNT - START_BLOCK_NO);

// This is what gets stored on the flash memory. It contains a serial counter and a custom,
// user-defined structure
#[derive(Encode, Decode)]
pub struct NonVolatileData {
	block_write_count: u32,
	payload: [u8; PAGE_SIZE - size_of::<u32>()],
}

impl NonVolatileData {
	pub fn encode<T: Encode + Decode>(payload: T, block_write_count: u32) -> NonVolatileData {
		let mut payload_buf: [u8; PAGE_SIZE - size_of::<u32>()] =
			[0u8; PAGE_SIZE - size_of::<u32>()];
		encode_into_slice(payload, &mut payload_buf, config::standard()).unwrap();
		let data: NonVolatileData = NonVolatileData {
			block_write_count: block_write_count,
			payload: payload_buf,
		};
		data
	}

	pub fn decode<T: Encode + Decode>(&self) -> (T, u32) {
		let decoded_payload: T = decode_from_slice(&self.payload, config::standard())
			.unwrap()
			.0;
		(decoded_payload, self.block_write_count)
	}
}

pub struct FlashCounter<'d, T: Default + Encode + Decode> {
	active_block_idx: u32,
	block_write_count: u32,
	flash: Flash<'d, FLASH, embassy_rp::flash::Async, TOTAL_FLASH_SIZE>,
	_phantom: PhantomData<T>, // Needed so the generic works
}

impl<'d, T: Default + Encode + Decode> FlashCounter<'d, T> {
	/// Initializes the Flash object we will be operating on
	fn init_flash(
		peripheral_flash: FLASH,
		dma_channel: AnyChannel,
	) -> Flash<'d, FLASH, embassy_rp::flash::Async, TOTAL_FLASH_SIZE> {
		embassy_rp::flash::Flash::<'d, FLASH, embassy_rp::flash::Async, TOTAL_FLASH_SIZE>::new(
			peripheral_flash,
			dma_channel,
		)
	}

	/// Panic if the flash has bytes set beyond the size of what we write there. This probably
	/// indicates we're somehow on the wrong area.
	fn validate_flash_is_empty(
		flash: &mut Flash<'d, FLASH, embassy_rp::flash::Async, TOTAL_FLASH_SIZE>,
	) -> (u32, u32) {
		// When we are setting off, every block from the start (at OFFSET) to the block
		// count should only have data in the first size_of (NonVolatileData<T>) bytes, and be
		// completely 0xFF for the rest of the block.
		info!("Validating Flash space…");

		// A block < ERASE SIZE is definitely required
		// < WRITE size insures it can be written in one operation
		// This should always be true
		assert!(size_of::<NonVolatileData>() <= PAGE_SIZE);

		// Also, payload encodding will fail if the struct to be encoded is too large.
		// That's what we catch here.
		assert!(size_of_val::<T>(&T::default()) < (PAGE_SIZE - size_of::<u32>()));

		// Iterate through each block, either
		// If its `block_write_count` is smaller than MAX_WRITES_PER_BLOCK go to the next block
		// and add MAX_WRITES_PER_BLOCK to the `total_write_count`
		// Or if it's smaller than MAX_WRITES_PER_BLOCK this is our current block. Add the writes
		// So far to `total_write_count`
		for block_index in START_BLOCK_NO..BLOCK_COUNT {
			// Read the payload data
			let mut buf = [0u8; size_of::<NonVolatileData>()];
			let flash_addr = ADDR_OFFSET as u32 + block_index * ERASE_SIZE as u32;
			flash.blocking_read(flash_addr, &mut buf).unwrap();

			// Make sure the rest of the block ix 0xFF
			for i in size_of::<NonVolatileData>()..ERASE_SIZE {
				let mut read: [u8; 1] = [0u8; 1];
				flash
					.blocking_read(flash_addr + i as u32, &mut read)
					.unwrap();
				if read[0] != 0xFF {
					error!("Flash block seems to be in use!");
					panic!();
				}
			}

			// Check if the current block is unitialized
			assert!(MAX_WRITES_PER_BLOCK != 0xFFFFFFFF); // This would break the logic
			let mut initialized = false;
			for i in 0..size_of::<NonVolatileData>() {
				if buf[i] != 0xFF {
					initialized = true;
				}
			}
			if initialized == false {
				debug!("First block is unitialized. Initializing…");
				let nv_data = NonVolatileData::encode(T::default(), 1);
				let mut to_write = [0u8; PAGE_SIZE];
				encode_into_slice(nv_data, &mut to_write, config::standard()).unwrap();
				flash.blocking_write(flash_addr, &to_write).unwrap();
				return (block_index, 0);
			}

			// The block is already initialized
			let nv_data: NonVolatileData = decode_from_slice(&buf, config::standard()).unwrap().0;
			if nv_data.block_write_count < MAX_WRITES_PER_BLOCK {
				debug!(
					"Found active block {:?}, written {:?} times.",
					block_index, nv_data.block_write_count
				);
				return (block_index, nv_data.block_write_count);
			} else {
				// This block is spent, advance to the next one
				debug!("Advancing to block {:?}", block_index)
			}
		}
		panic!("Flash is spent!");
	}

	pub fn new(peripheral_flash: FLASH, dma_channel: AnyChannel) -> Self {
		let mut flash = Self::init_flash(peripheral_flash, dma_channel);

		let (active_block_idx, block_write_count) = Self::validate_flash_is_empty(&mut flash);

		Self {
			flash: flash,
			active_block_idx: active_block_idx,
			block_write_count: block_write_count,
			_phantom: PhantomData::default(),
		}
	}

	/// Get the total amount of times we wrote to our flash
	pub fn total_write_count(&self) -> u32 {
		return self.active_block_idx * MAX_WRITES_PER_BLOCK + self.block_write_count;
	}

	pub fn exhaustion(&self) -> f32 {
		return (self.active_block_idx * MAX_WRITES_PER_BLOCK + self.block_write_count) as f32
			/ MAX_FLASH_WRITES as f32;
	}

	/// Writes the payload to the flash and increments the write counts by one. If the block is
	/// full, we go to the next block.
	pub fn write(&mut self, payload: T) {
		if self.block_write_count >= MAX_WRITES_PER_BLOCK {
			self.block_write_count = 1;
			self.active_block_idx += 1;
			assert!(self.active_block_idx < BLOCK_COUNT);
			debug!("Moving to next block {:?}", self.active_block_idx);
		} else {
			self.block_write_count += 1;
			debug!("New block write coung {:?}", self.block_write_count);
		}

		// Set up the data to be written
		let nv_data = NonVolatileData::encode(payload, self.block_write_count);
		let mut buf = [0u8; size_of::<NonVolatileData>()];
		encode_into_slice(nv_data, &mut buf, config::standard()).unwrap();

		let flash_addr = ADDR_OFFSET as u32 + self.active_block_idx * ERASE_SIZE as u32;
		self.flash
			.blocking_erase(flash_addr, flash_addr + ERASE_SIZE as u32)
			.unwrap();
		self.flash.blocking_write(flash_addr, &buf).unwrap();
	}

	/// Reads the payload from the flash
	pub async fn read(&mut self) -> T {
		debug!("Reading from block {:?}", self.active_block_idx);

		// Read from the flash
		let flash_addr = ADDR_OFFSET as u32 + self.active_block_idx * ERASE_SIZE as u32;
		let mut buf = [0u8; size_of::<NonVolatileData>()];
		self.flash.read(flash_addr, &mut buf).await.unwrap();

		// Decode
		let nv_data: NonVolatileData = decode_from_slice(&buf, config::standard()).unwrap().0;
		debug!(
			"Write count from flash was: {:?}",
			nv_data.block_write_count
		);
		assert!(nv_data.block_write_count == self.block_write_count);
		let payload: T = decode_from_slice(&nv_data.payload, config::standard())
			.unwrap()
			.0;

		return payload;
	}
}
