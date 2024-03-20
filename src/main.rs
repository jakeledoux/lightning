use std::time::Duration;

use car::ThrottleLimit;
use clap::Parser;
use controller::ControlFrame;
use tracing::{event, level_filters::LevelFilter, Level};
use tracing_subscriber::EnvFilter;

use crate::controller::Controller;
use crate::sockets::SocketClient;

pub mod car;
pub mod controller;
pub mod sockets;

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
        address: String,
    },
    /// Connect to gamepad and send commands to car
    Controller {
        #[clap(flatten)]
        gamepad_args: GamePadArgs,
        #[arg(default_value_t = 25565)]
        port: u128,
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
    tty_path: String,
}

async fn run_car(car_args: CarArgs, address: &str) -> anyhow::Result<()> {
    // connect to car
    let mut car_conn = car::CarConn::connect(
        &car_args.tty_path,
        ThrottleLimit::Limit(car_args.throttle_limit),
    )
    .await?;

    // connect to controller
    let mut client = SocketClient::new_client(address).await?;

    loop {
        let frame: ControlFrame = client.request_message().await?;

        if car_conn.send(frame).await.is_err() {
            event!(Level::ERROR, "failed to send data to car");
        };
    }
}

async fn run_controller(gamepad_args: GamePadArgs, port: u128) -> anyhow::Result<()> {
    // connect controller
    let mut controller =
        Controller::find_any_controller(Duration::from_secs(gamepad_args.controller_timeout))
            .await?;

    let mut client = SocketClient::new_host(port).await?;

    loop {
        client.transmit_message(|| controller.poll()).await?;
    }
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
    )
    .await?;

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
        Command::Car { car_args, address } => run_car(car_args, &address).await,
        Command::Controller { gamepad_args, port } => run_controller(gamepad_args, port).await,
        Command::Integrated {
            car_args,
            gamepad_args,
        } => run_integrated(car_args, gamepad_args).await,
    }
}
