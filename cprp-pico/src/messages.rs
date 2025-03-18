use embassy_sync::channel::Channel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::BOARD_COUNT;

pub static ADC_MESSAGE_CHANNEL: Channel<CriticalSectionRawMutex, ADCMessage, BOARD_COUNT> = Channel::new();

/// message from ADC task to main task
#[derive(Debug)]
pub enum ADCMessage {
    /// (board id)
    DoneInitializing(u8),
    /// (reading, channel, board id)
    Data(u16, u8, u8),
} 