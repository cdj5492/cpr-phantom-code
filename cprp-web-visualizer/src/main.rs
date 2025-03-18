mod data_receiver;
mod data_types;
mod web_server;

use data_types::DataBuffer;
use std::sync::Arc;
use tokio::sync::RwLock;

const UDP_ADDR: &str = "127.0.0.1:9870";  // Match the forward_to_plotjuggler port
const WEB_ADDR: &str = "127.0.0.1:8080";
const MAX_SAMPLES: usize = 1000;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize shared data buffer
    let buffer = Arc::new(RwLock::new(DataBuffer::new(MAX_SAMPLES)));

    // Create UDP receiver
    let receiver = data_receiver::DataReceiver::new(UDP_ADDR, buffer.clone()).await?;
    
    // Create web server
    let server = web_server::WebServer::new(WEB_ADDR, buffer);

    // Start both services
    println!("Starting CPR Phantom Data Visualizer");
    
    tokio::select! {
        receiver_result = receiver.start() => {
            if let Err(e) = receiver_result {
                eprintln!("UDP receiver error: {}", e);
            }
        }
        server_result = server.start() => {
            if let Err(e) = server_result {
                eprintln!("Web server error: {}", e);
            }
        }
        _ = tokio::signal::ctrl_c() => {
            println!("\nReceived Ctrl+C, shutting down...");
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn test_buffer_initialization() {
        let buffer = Arc::new(RwLock::new(DataBuffer::new(MAX_SAMPLES)));
        let buffer_guard = buffer.read().await;
        assert_eq!(buffer_guard.max_samples, MAX_SAMPLES);
        assert!(buffer_guard.data.is_empty());
    }
}
