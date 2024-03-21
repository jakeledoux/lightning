use std::{
    io::Write,
    path::{Path, PathBuf},
    time::{Duration, SystemTime, UNIX_EPOCH},
};

use anyhow::anyhow;
use nokhwa::{
    pixel_format::RgbFormat,
    utils::{CameraIndex, RequestedFormat},
    Camera,
};
use serde::Deserialize;
use tokio::{fs::File, io::AsyncWriteExt};
use tracing::{event, Level};

use crate::controller::ControlFrame;

pub struct Recorder {
    file: File,
    camera_dir: PathBuf,
    camera: Option<Camera>,
    last_log: Option<SystemTime>,
}

impl Recorder {
    pub async fn open(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let path: &Path = path.as_ref();
        if path.is_file() {
            todo!()
        }
        if !path.is_dir() {
            tokio::fs::create_dir(path).await?;
        }

        let unix_time = unix_time();

        let file = File::create(path.join(format!("{}.csv", unix_time))).await?;

        let camera_dir = path.join(format!("{}", unix_time)).to_owned();
        tokio::fs::create_dir(&camera_dir).await?;

        let camera = if cfg!(target_os = "macos") {
            event!(Level::WARN, "camera capture is not available on macOS");
            None
        } else {
            let mut camera = Camera::new(
                CameraIndex::Index(0),
                RequestedFormat::new::<RgbFormat>(
                    nokhwa::utils::RequestedFormatType::AbsoluteHighestFrameRate,
                ),
            )?;
            camera.open_stream()?;
            Some(camera)
        };

        event!(Level::INFO, "recording ready");

        Ok(Self {
            file,
            camera_dir,
            camera,
            last_log: None,
        })
    }

    async fn log(&mut self, data_frame: DataFrame) -> anyhow::Result<()> {
        let unix_time = unix_time();

        let mut buf = Vec::<u8>::new();
        writeln!(buf, "{:?},{}", unix_time, data_frame.csv_line())?;
        self.file.write_all(&mut buf).await?;

        event!(Level::DEBUG, "recorded data at: {unix_time}");

        if let Some(ref mut camera) = self.camera {
            let frame = camera.frame()?;
            let decoded = frame.decode_image::<RgbFormat>()?;
            // convert from nokhwa to image (for some reason nokwha's version of image refuses to
            // write images
            let decoded =
                image::RgbImage::from_vec(decoded.width(), decoded.height(), decoded.into_vec())
                    .ok_or(anyhow!("failed to decode image"))?;

            // TODO: I wish this was async. capture can possibly be made multithreaded with nokhwa
            // feature: `output-threaded`
            decoded.save(self.camera_dir.join(format!("{}.jpeg", unix_time)))?;
        }

        Ok(())
    }

    pub async fn log_every(
        &mut self,
        data_frame: DataFrame,
        interval: Duration,
    ) -> Option<anyhow::Result<()>> {
        let now = SystemTime::now();
        if let Some(last_log) = self.last_log {
            if now
                .duration_since(last_log)
                .expect("we've invented time travel")
                < interval
            {
                return None;
            }
        }

        let result = self.log(data_frame).await;
        if result.is_ok() {
            self.last_log = Some(now);
        }
        Some(result)
    }
}
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct DataFrame {
    steering: f32,
    throttle: f32,
}

impl DataFrame {
    pub fn new(control_frame: ControlFrame) -> Self {
        Self {
            steering: control_frame.steering,
            throttle: control_frame.throttle,
        }
    }

    pub fn csv_line(&self) -> String {
        format!("{},{}", self.steering, self.throttle)
    }
}

fn unix_time() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("welcome to the 1960s")
        .as_secs()
}
