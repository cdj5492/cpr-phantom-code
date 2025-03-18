use crate::data_types::{SensorData, SharedDataBuffer};
use std::error::Error;
use tokio::net::UdpSocket;

pub struct DataReceiver {
    socket: UdpSocket,
    buffer: SharedDataBuffer,
}

impl DataReceiver {
    pub async fn new(addr: &str, buffer: SharedDataBuffer) -> Result<Self, Box<dyn Error>> {
        let socket = UdpSocket::bind(addr).await?;
        Ok(Self { socket, buffer })
    }

    pub async fn start(&self) -> Result<(), Box<dyn Error>> {
        println!("Started UDP data receiver on {}", self.socket.local_addr()?);
        let mut buf = vec![0u8; 1024];

        loop {
            match self.socket.recv(&mut buf).await {
                Ok(n) => {
                    if let Ok(json_str) = String::from_utf8(buf[..n].to_vec()) {
                        println!("Received JSON: {}", json_str); // Add debug print
                        match serde_json::from_str::<SensorData>(&json_str) {
                            Ok(data) => {
                                let mut buffer = self.buffer.write().await;
                                buffer.add_sample(data);
                            }
                            Err(e) => eprintln!("Error parsing JSON: {}", e),
                        }
                    }
                }
                Err(e) => eprintln!("Error receiving data: {}", e),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data_types::DataBuffer;
    use std::sync::Arc;
    use tokio::sync::RwLock;

    #[tokio::test]
    async fn test_data_receiver_creation() {
        let buffer = Arc::new(RwLock::new(DataBuffer::new(1000)));
        let receiver = DataReceiver::new("127.0.0.1:0", buffer).await;
        assert!(receiver.is_ok());
    }

    #[tokio::test]
    async fn test_data_processing() {
        let buffer = Arc::new(RwLock::new(DataBuffer::new(1000)));
        let receiver = DataReceiver::new("127.0.0.1:0", buffer.clone()).await.unwrap();
        
        // Create test data
        let test_data = SensorData {
            timestamp: 1.0,
            adc_values: vec![100, 200, 300],
        };
        
        // Simulate receiving data
        let json_data = serde_json::to_string(&test_data).unwrap();
        let socket = UdpSocket::bind("127.0.0.1:0").await.unwrap();
        socket.send_to(json_data.as_bytes(), receiver.socket.local_addr().unwrap()).await.unwrap();
        
        // Allow some time for processing
        tokio::time::sleep(tokio::time::Duration::from_millis(100)).await;
        
        // Verify data was processed
        let buffer_data = buffer.read().await;
        assert!(!buffer_data.data.is_empty());
        let sample = &buffer_data.data[0];
        assert_eq!(sample.timestamp, 1.0);
        assert_eq!(sample.adc_values, vec![100, 200, 300]);
    }
}

