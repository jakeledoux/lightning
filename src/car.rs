use std::{io::Write, time::Duration};

use tokio::time::Instant;
use tokio_serial::{SerialPortBuilderExt, SerialStream};
use tracing::{event, Level};

use crate::controller::ControlFrame;

// TODO: tune this
const MIN_MESSAGE_INTERVAL: Duration = Duration::from_millis(50);

const STEER_GAIN: f32 = 40.0;
const STEER_MIDPOINT: f32 = 90.0;
const THROTTLE_GAIN: f32 = 100.0;

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
    pub steering: u32,
    pub throttle: i32,
    pub beep: bool,
}

impl CarMessage {
    pub fn with_throttle_limit(mut self, limit: ThrottleLimit) -> Self {
        self.throttle = (self.throttle as f32 * limit.value()).round() as i32;
        self
    }

    pub fn bytes(&self) -> Vec<u8> {
        format!(
            "<s{}t{}b{}/>",
            self.steering,
            self.throttle,
            if self.beep { "1" } else { "0" }
        )
        .as_bytes()
        .to_vec()
    }
}

impl From<ControlFrame> for CarMessage {
    fn from(value: ControlFrame) -> Self {
        Self {
            steering: (value.steering * STEER_GAIN + STEER_MIDPOINT).round() as u32,
            throttle: (value.throttle * THROTTLE_GAIN).round() as i32,
            beep: value.beep,
        }
    }
}

pub struct CarConn {
    throttle_limit: ThrottleLimit,
    port: SerialStream,
    last_message: Option<Instant>,
}

impl CarConn {
    pub fn connect(
        tty_path: &str,
        throttle_limit: ThrottleLimit,
    ) -> Result<Self, tokio_serial::Error> {
        let port = tokio_serial::new(tty_path, 9600).open_native_async()?;
        event!(Level::INFO, "connected to car");
        Ok(Self {
            throttle_limit,
            port,
            last_message: None,
        })
    }

    pub async fn send(&mut self, frame: ControlFrame) -> Result<(), tokio_serial::Error> {
        if let Some(last_message) = self.last_message {
            tokio::time::sleep_until(last_message + MIN_MESSAGE_INTERVAL).await;
        }

        self.port.write_all(
            &CarMessage::from(frame)
                .with_throttle_limit(self.throttle_limit)
                .bytes(),
        )?;

        self.last_message = Some(Instant::now());
        Ok(())
    }
}
