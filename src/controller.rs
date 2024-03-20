use std::time::Duration;

use gilrs::{Axis, Button, Event, EventType, Gamepad, GamepadId, Gilrs};
use serde::{Deserialize, Serialize};
use tokio::time::error::Elapsed;
use tracing::{event, Level};

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct ControlFrame {
    pub steering: f32,
    pub throttle: f32,
    pub horn: bool,
    pub start: bool,
}

pub struct Controller {
    gilrs: Gilrs,
    gamepad_id: GamepadId,
}

impl Controller {
    pub async fn find_any_controller(timeout: Duration) -> Result<Self, Elapsed> {
        let mut gilrs = Gilrs::new().unwrap();
        event!(Level::INFO, "Searching for controllers...");
        tokio::time::timeout(timeout, async {
            loop {
                if let Some(Event { id, .. }) = gilrs.next_event() {
                    event!(Level::INFO, "Connected to {}", gilrs.gamepad(id).name());
                    return Self {
                        gilrs,
                        gamepad_id: id,
                    };
                }
                tokio::task::yield_now().await;
            }
        })
        .await
    }

    pub async fn find_controller_by_button(timeout: Duration) -> Result<Self, Elapsed> {
        let mut gilrs = Gilrs::new().unwrap();
        event!(
            Level::INFO,
            "Press the south face button on the controller you wish to use."
        );

        tokio::time::timeout(timeout, async {
            loop {
                while let Some(Event { id, event, .. }) = gilrs.next_event() {
                    if matches!(event, EventType::ButtonPressed(Button::South, ..)) {
                        event!(Level::INFO, "Connected to {}", gilrs.gamepad(id).name());
                        return Self {
                            gilrs,
                            gamepad_id: id,
                        };
                    }
                }
                tokio::task::yield_now().await;
            }
        })
        .await
    }

    pub fn poll(&mut self) -> ControlFrame {
        Self::consume_events(&mut self.gilrs);

        let gamepad = self.gilrs.gamepad(self.gamepad_id);

        ControlFrame {
            steering: Self::read_axis(&gamepad, Axis::LeftStickX),
            throttle: Self::read_button(&gamepad, Button::RightTrigger2)
                - Self::read_button(&gamepad, Button::LeftTrigger2),
            horn: gamepad.is_pressed(Button::South),
            start: gamepad.is_pressed(Button::Start),
        }
    }

    fn consume_events(gilrs: &mut Gilrs) {
        while let Some(_event) = gilrs.next_event() {}
    }

    fn read_axis(gamepad: &Gamepad, axis: Axis) -> f32 {
        gamepad
            .axis_data(axis)
            .map(|axis| axis.value())
            .unwrap_or_default()
    }

    fn read_button(gamepad: &Gamepad, button: Button) -> f32 {
        gamepad
            .button_data(button)
            .map(|button| button.value())
            .unwrap_or_default()
    }
}
