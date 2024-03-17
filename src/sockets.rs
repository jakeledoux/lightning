use serde::{de::DeserializeOwned, Serialize};
use tokio::{
    io::{self, AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    task::yield_now,
};
use tracing::{event, Level};

pub const MSG_READY: u8 = 0xFF;

#[derive(thiserror::Error, Debug)]
pub enum Error {
    #[error("io error")]
    IoError(#[from] io::Error),
    #[error("deserialization error")]
    DeserializationError(#[from] bincode::Error),
}

pub struct SocketClient {
    stream: TcpStream,
}

impl SocketClient {
    pub async fn new_host(port: u128) -> io::Result<Self> {
        // await TCP connection from car
        event!(Level::INFO, "listening TCP on: {port}");
        let listener = TcpListener::bind(format!("0.0.0.0:{port}")).await?;
        let (stream, _) = listener.accept().await?;
        event!(
            Level::INFO,
            "connection established with: {:?}",
            stream.peer_addr()?
        );

        Ok(Self { stream })
    }

    pub async fn new_client(address: &str) -> io::Result<Self> {
        event!(Level::INFO, "attempting connection to: {address}");
        let stream = TcpStream::connect(address).await?;
        event!(Level::INFO, "connection established");

        Ok(Self { stream })
    }

    pub async fn transmit_message<T: Serialize, F: FnOnce() -> T>(
        &mut self,
        payload_fn: F,
    ) -> Result<(), Error> {
        loop {
            // wait for ready signal
            let ready = self.stream.read_u8().await?;
            if ready != MSG_READY {
                yield_now().await;
                continue;
            };

            // transmit message
            let encoded: Vec<u8> = bincode::serialize(&payload_fn())?;
            self.stream.write_u64(encoded.len() as u64).await?;
            self.stream.write_all(&encoded).await?;
            return Ok(());
        }
    }

    pub async fn request_message<T: DeserializeOwned>(&mut self) -> Result<T, Error> {
        // transmit ready signal
        self.stream.write_u8(MSG_READY).await?;

        // read message from socket
        let message_length = self.stream.read_u64().await?;
        let mut buf = vec![0; message_length as usize];
        self.stream.read_exact(&mut buf).await?;

        let payload: T = bincode::deserialize(&buf)?;
        Ok(payload)
    }
}
