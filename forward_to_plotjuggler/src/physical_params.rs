use crate::rib::{RibSegment, Rib};

/// array of zero values for each potentiometer
pub const ZERO_POTENTIOMETER_VALUES: [f32; 2] = [
    245.0, // upper sternum
    240.0, // lower sternum
];

/// slope in mm/(adc tick) for each potentiometer
pub const MULTIPLIER_POTENTIOMETER_VALUES: [f32; 2] = [0.111261, 0.111261];

/// array of flat values for each flex sensor
pub const ZERO_FLEX_VALUES: [f32; 25] = [
    1162.0, // TODO: check this one (short one)
    1134.0, // 1
    1168.0, // 2
    1207.0, // 3
    1175.0, // 4
    1215.0, // 5
    1153.0, // 6
    1120.0, // 7
    1205.0, // 8
    1148.0, // 9
    1203.0, // 10
    1190.0, // 11
    1080.0, // 12
    1191.0, // 13
    1175.0, // 14
    1123.0, // 15
    1146.0, // 16
    1205.0, // 17
    1145.0, // 18
    1112.0, // 19
    1175.0, // 20
    970.0,  // 21
    1002.0, // 22
    1015.0, // 23
    1122.0, // 24
];

pub const MULTIPLIER_FLEX_VALUES: [f32; 25] = [
    1.0,            // TODO: check this one (short one)
    0.008333333333, // 1
    0.01032608696,  // 2
    0.006597222222, // 3
    0.0095,         // 4
    0.01010638298,  // 5
    0.008050847458, // 6
    0.007089552239, // 7
    0.005277777778, // 8
    0.005107526882, // 9
    0.008962264151, // 10
    0.00753968254,  // 11
    0.009693877551, // 12
    0.00931372549,  // 13
    0.005337078652, // 14
    0.006506849315, // 15
    0.01130952381,  // 16
    0.0095,         // 17
    0.01696428571,  // 18
    0.008050847458, // 19
    0.008636363636, // 20
    0.01357142857,  // 21
    0.004702970297, // 22
    0.0059375,      // 23
    0.009895833333, // 24
];

/// Each force sensor has a different calibration curve.
/// Store those curves in this array.
pub const FORCE_CURVES: [fn(f32) -> f32; 7] = [
    |x| x * (x * (x * 1.62e-08 + 1.70e-06) + 0.0185) + 0.0974, // 0
    |x| x * (x * (x * 3.09e-08 + -1.78e-05) + 0.0261) + 0.0762, // 1
    |x| x * (x * (x * -1.88e-08 + 4.83e-05) + 0.0090) + 0.0051, // 2
    |x| x * (x * (x * 3.00e-08 + -1.28e-05) + 0.0225) + 0.2349, // 3
    |x| x * (x * (x * -6.63e-08 + 7.81e-05) + 0.0065) + 0.3268, // 4
    |x| x * (x * (x * -5.02e-08 + 9.06e-05) + -0.0034) + 0.2722, // 5
    |x| x * (x * (x * 2.19e-09 + 2.42e-05) + 0.0118) + 0.1831, // 6
];

// TODO: These are all placeholder segments
pub const RIB0_SEGMENTS: [RibSegment; 3] = [
    RibSegment { channel: 0, length: 55.0, error: 0.0 },
    RibSegment { channel: 1, length: 55.0, error: 0.0 },
    RibSegment { channel: 2, length: 55.0, error: 0.0 },
];
pub const RIB1_SEGMENTS: [RibSegment; 4] = [
    RibSegment { channel: 3, length: 55.0, error: 0.0 },
    RibSegment { channel: 4, length: 55.0, error: 0.0 },
    RibSegment { channel: 5, length: 55.0, error: 0.0 },
    RibSegment { channel: 6, length: 55.0, error: 0.0 },
];
pub const RIB2_SEGMENTS: [RibSegment; 4] = [
    RibSegment { channel: 7, length: 55.0, error: 0.0 },
    RibSegment { channel: 8, length: 55.0, error: 0.0 },
    RibSegment { channel: 9, length: 55.0, error: 0.0 },
    RibSegment { channel: 10, length: 55.0, error: 0.0 },
];
pub const RIB3_SEGMENTS: [RibSegment; 5] = [
    RibSegment { channel: 11, length: 55.0, error: 0.0 },
    RibSegment { channel: 12, length: 55.0, error: 0.0 },
    RibSegment { channel: 13, length: 55.0, error: 0.0 },
    RibSegment { channel: 14, length: 55.0, error: 0.0 },
    RibSegment { channel: 15, length: 55.0, error: 0.0 },
];

/// Rib segment lengths. First rib is at the top of the ribcage, last rib is at the bottom.
/// Segments start at the spine and end at the sternum.
pub const RIBS: [Rib; 4] = [
    Rib { segments: &RIB0_SEGMENTS },
    Rib { segments: &RIB1_SEGMENTS },
    Rib { segments: &RIB2_SEGMENTS },
    Rib { segments: &RIB3_SEGMENTS },
];

/// Each channel on each board corresponds to a specific sensor, either a flex or a force sensor.
/// Maps the channel number to the sensor type and ID.
/// WARNING: Does not enforce mutual exclusivity of flex and force sensors.
pub const CHANNEL_SENSOR_ID_MAP: [Sensor; 26] = [
    // built-in channels
    Sensor::Potentiometer(0), // upper sternum
    Sensor::Potentiometer(1), // lower sternum
    // board 0
    // Sensor::Flex(1),
    // Sensor::Flex(2),
    // Sensor::Flex(3),
    // Sensor::Flex(4),
    // board 1
    // Sensor::Flex(5),
    // Sensor::Flex(6),
    // Sensor::Flex(7),
    // Sensor::Flex(8),

    // board 2
    Sensor::Flex(16),
    Sensor::Flex(17),
    Sensor::Flex(15),
    Sensor::Flex(13),
    // board 3
    Sensor::Flex(0), // unused
    Sensor::Force(0),
    Sensor::Force(1),
    Sensor::Force(2),
    // board 4
    Sensor::Force(3),
    Sensor::Force(4),
    Sensor::Force(5),
    Sensor::Force(6),
    // board 5
    Sensor::Flex(8),
    Sensor::Flex(12),
    Sensor::Flex(10),
    Sensor::Flex(16), // Top rib. Double check (stuck under sternum)
    // board 6 (double check this entire rib)
    Sensor::Flex(17),
    Sensor::Flex(18),
    Sensor::Flex(19), // Double check (unclear wiring under tape)
    Sensor::Flex(20), // Double check (stuck under sternum)
    // board 7
    Sensor::Flex(14),
    Sensor::Flex(1),
    Sensor::Flex(2),
    Sensor::Flex(3), // Double check (stuck under sternum)
];

pub enum Sensor {
    /// (ID,)
    Flex(usize),
    /// (ID,)
    Force(usize),
    /// (ID,)
    Potentiometer(usize),
}

impl Sensor {
    pub fn apply_calibration(&self, x: i16) -> f32 {
        match self {
            Sensor::Flex(id) => {
                // return x as f32;
                let x = ZERO_FLEX_VALUES[*id] - x as f32;
                let mult = MULTIPLIER_FLEX_VALUES[*id];
                x * mult
            }
            Sensor::Force(id) => {
                // return x as f32;
                FORCE_CURVES[*id](x as f32)
            }
            Sensor::Potentiometer(id) => {
                (x as f32 - ZERO_POTENTIOMETER_VALUES[*id]) * MULTIPLIER_POTENTIOMETER_VALUES[*id]
            }
        }
    }
}
