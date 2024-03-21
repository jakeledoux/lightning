use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tracing::{event, Level};

use crate::controller::ControlFrame;

const BAUD_RATE: u32 = 38400;
const HANDSHAKE: u8 = 0xFF;
const READY: u8 = 0x9E;

const STEER_GAIN: f32 = 40.0;
const STEER_MIDPOINT: f32 = 90.0;
const THROTTLE_GAIN: f32 = 100.0;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("Car {0}")]
    StatusError(&'static str),
    #[error("Car did not return correct checksum")]
    ChecksumError,
    #[error(transparent)]
    SerialError(#[from] tokio_serial::Error),
    #[error(transparent)]
    IoError(#[from] std::io::Error),
}

#[derive(Debug, Clone, Copy)]
pub enum ThrottleLimit {
    None,
    Limit(f32),
}

impl ThrottleLimit {
    pub fn value(self) -> f32 {
        match self {
            ThrottleLimit::None => 1.0,
            ThrottleLimit::Limit(v) => v,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct CarMessage {
    pub steering: u8,
    pub throttle: i8,
    pub horn: bool,
}

impl CarMessage {
    pub fn with_throttle_limit(mut self, limit: ThrottleLimit) -> Self {
        self.throttle = (self.throttle as f32 * limit.value()).round() as i8;
        self
    }

    pub fn bytes(&self) -> [u8; 3] {
        [self.steering, self.throttle as u8, self.horn as u8]
    }
}

impl From<ControlFrame> for CarMessage {
    fn from(value: ControlFrame) -> Self {
        Self {
            steering: (value.steering * STEER_GAIN + STEER_MIDPOINT).round() as u8,
            throttle: (value.throttle * THROTTLE_GAIN).round() as i8,
            horn: value.horn,
        }
    }
}

pub struct CarConn {
    throttle_limit: ThrottleLimit,
    port: SerialStream,
}

impl CarConn {
    pub async fn connect(tty_path: &str, throttle_limit: ThrottleLimit) -> Result<Self, Error> {
        let mut port = tokio_serial::new(tty_path, BAUD_RATE).open_native_async()?;
        event!(Level::INFO, "connecting to car");
        if port.read_u8().await? != HANDSHAKE {
            return Err(Error::StatusError("failed handshake"));
        }
        event!(Level::INFO, "connection established");
        Ok(Self {
            throttle_limit,
            port,
        })
    }

    pub async fn send(&mut self, frame: ControlFrame) -> Result<(), Error> {
        let message = CarMessage::from(frame);

        // wait for ready signal
        match self.port.read_u8().await {
            Ok(READY) => {} // good
            Ok(_) => {
                return Err(Error::StatusError("mangled ready signal"));
            }
            Err(_) => {
                return Err(Error::StatusError("is not ready"));
            }
        }

        // write controls
        self.port
            .write_all(
                &CarMessage::from(frame)
                    .with_throttle_limit(self.throttle_limit)
                    .bytes(),
            )
            .await?;

        // verify checksum
        let sum = self.port.read_u8().await?;
        let checksum = message
            .steering
            .wrapping_add(message.throttle as u8)
            .wrapping_add(message.horn as u8);

        if sum != checksum {
            return Err(Error::ChecksumError);
        }

        Ok(())
    }
}
