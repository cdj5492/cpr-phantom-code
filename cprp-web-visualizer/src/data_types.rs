use serde::{Deserialize, Serialize};
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SensorData {
    #[serde(rename = "t")]
    pub timestamp: f64,
    #[serde(rename = "d")]
    pub adc_values: Vec<u16>,
}

#[derive(Debug, Clone, Default)]
pub struct DataBuffer {
    /// Circular buffer of sensor data with timestamps and values
    pub data: Vec<SensorData>,
    /// Maximum number of samples to keep in the buffer
    pub max_samples: usize,
}

impl DataBuffer {
    pub fn new(max_samples: usize) -> Self {
        Self {
            data: Vec::with_capacity(max_samples),
            max_samples,
        }
    }

    pub fn add_sample(&mut self, sample: SensorData) {
        if self.data.len() >= self.max_samples {
            self.data.remove(0);
        }
        self.data.push(sample);
    }
}

pub type SharedDataBuffer = Arc<RwLock<DataBuffer>>;

#[derive(Debug, Serialize)]
pub struct DataResponse {
    pub timestamps: Vec<f64>,
    pub values: Vec<Vec<u16>>,
}

impl DataResponse {
    pub fn from_buffer(buffer: &DataBuffer) -> Self {
        let mut timestamps = Vec::with_capacity(buffer.data.len());
        let mut values = vec![Vec::with_capacity(buffer.data.len()); 12]; // 12 ADC channels

        for sample in &buffer.data {
            timestamps.push(sample.timestamp);
            for (i, &value) in sample.adc_values.iter().enumerate() {
                if i < values.len() {
                    values[i].push(value);
                }
            }
        }

        Self { timestamps, values }
    }
}

