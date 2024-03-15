use std::time::Duration;

use car::ThrottleLimit;
use clap::Parser;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
};
use tracing::{event, level_filters::LevelFilter, Level};
use tracing_subscriber::EnvFilter;

use crate::controller::{ControlFrame, Controller};

pub mod car;
pub mod controller;

#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    #[clap(subcommand)]
    command: Command,
}

#[derive(clap::Subcommand, Debug)]
enum Command {
    /// Relay commands from controller to arduino
    Car {
        #[clap(flatten)]
        car_args: CarArgs,
    },
    /// Connect to gamepad and send commands to car
    Controller {
        #[clap(flatten)]
        gamepad_args: GamePadArgs,
    },
    /// Run both Car and Controller components locally
    Integrated {
        #[clap(flatten)]
        car_args: CarArgs,
        #[clap(flatten)]
        gamepad_args: GamePadArgs,
    },
}

#[derive(clap::Args, Debug)]
struct GamePadArgs {
    /// timeout for connecting to controller
    #[arg(short, long, default_value_t = 10)]
    controller_timeout: u64,
}

#[derive(clap::Args, Debug)]
struct CarArgs {
    // TODO: validate argument in range
    #[arg(long, default_value_t = 0.0)]
    throttle_limit: f32,
    // TTY path to Arduino serial connection
    #[arg(long, default_value = "/dev/cu.usbmodem11121301")]
    tty_path: String,
}

async fn run_car(car_args: CarArgs) -> anyhow::Result<()> {
    // connect to car
    let mut car_conn = car::CarConn::connect(
        &car_args.tty_path,
        ThrottleLimit::Limit(car_args.throttle_limit),
    )?;

    // connect to controller
    let address = "127.0.0.1:25565";
    event!(Level::INFO, "attempting connection to: {address}");
    let mut stream = TcpStream::connect(address).await?;
    event!(Level::INFO, "connection established");

    loop {
        // read frame from socket
        let message_length = stream.read_u64().await?;
        let mut buf = vec![0; message_length as usize];
        stream.read_exact(&mut buf).await?;
        let frame: ControlFrame = bincode::deserialize(&buf)?;

        if car_conn.send(frame).await.is_err() {
            event!(Level::ERROR, "failed to send data to car");
        };
    }
}

async fn run_controller(gamepad_args: GamePadArgs) -> anyhow::Result<()> {
    // connect controller
    let mut controller =
        Controller::find_any_controller(Duration::from_secs(gamepad_args.controller_timeout))
            .await?;

    // await TCP connection from car
    event!(Level::INFO, "listening TCP");
    let listener = TcpListener::bind("0.0.0.0:25565").await?;
    let (mut socket, _) = listener.accept().await?;
    event!(
        Level::INFO,
        "connection established with: {:?}",
        socket.peer_addr()?
    );

    loop {
        tokio::time::sleep(Duration::from_millis(50)).await;
        let frame = controller.poll();
        if frame.start {
            break;
        }
        let encoded: Vec<u8> = bincode::serialize(&frame)?;
        socket.write_u64(encoded.len() as u64).await?;
        socket.write(&encoded).await?;
        // TODO: rate-limit
    }

    Ok(())
}

async fn run_integrated(car_args: CarArgs, gamepad_args: GamePadArgs) -> anyhow::Result<()> {
    // connect controller
    let mut controller =
        Controller::find_any_controller(Duration::from_secs(gamepad_args.controller_timeout))
            .await?;

    // connect to car
    let mut car_conn = car::CarConn::connect(
        &car_args.tty_path,
        ThrottleLimit::Limit(car_args.throttle_limit),
    )?;

    loop {
        let frame = controller.poll();

        if frame.start {
            break;
        }

        if car_conn.send(frame).await.is_err() {
            event!(Level::ERROR, "failed to send data to car");
        };
    }
    Ok(())
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    // attach log subscriber
    tracing_subscriber::fmt()
        // ignore traces from other crates unless they're errors
        .with_env_filter(
            EnvFilter::builder()
                .with_default_directive(LevelFilter::ERROR.into())
                .from_env()?
                .add_directive("lightning=debug".parse()?),
        )
        .init();

    // parse CLI arguments
    let args = Args::parse();

    match args.command {
        Command::Car { car_args } => run_car(car_args).await,
        Command::Controller { gamepad_args } => run_controller(gamepad_args).await,
        Command::Integrated {
            car_args,
            gamepad_args,
        } => run_integrated(car_args, gamepad_args).await,
    }
}
