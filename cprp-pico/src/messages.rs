use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

// Number of ADC boards on each bus
pub const ADC_COUNT_B0: usize = 2;
pub const ADC_COUNT_B1: usize = 1;

pub const BOARD_COUNT: usize = ADC_COUNT_B0 + ADC_COUNT_B1;
pub const CHANNEL_COUNT: usize = BOARD_COUNT * 4;

pub const MAGIC: u16 = 0xAA55;

// Size of the packet in bytes
pub const PACKET_SIZE: usize = 2 + 1 + 4 + CHANNEL_COUNT * 2 as usize;

// Ideally how many packets should we be sending per second?
pub const TARGET_PACKET_RATE: u64 = 1000;

pub static ADC_MESSAGE_CHANNEL: Channel<CriticalSectionRawMutex, ADCMessage, BOARD_COUNT> = Channel::new();

// message from ADC task to main task
#[derive(Debug)]
pub enum ADCMessage {
    // (board id)
    DoneInitializing(u8),
    // (reading, channel, board id)
    Data(u16, u8, u8),
} 