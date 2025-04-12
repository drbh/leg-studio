# Leg Studio


<div align="center">
<img src="https://github.com/user-attachments/assets/962cf9db-5852-44cb-9129-dd6debc8271b" width="1312">
</div>


A visualization and control application for servo motors, with support for defining servo tracks via TOML configuration files.

## Features

- Real-time visualization of servo positions
- Define servo tracks via TOML configuration files
- Create and save looped movement patterns
- Control multiple servos simultaneously
- Interactive UI for configuration and monitoring

## Track Configuration

Tracks can be defined using TOML configuration files in the `tracks` directory. Each file defines one servo track with the following format:

```toml
# Basic track information
servo_id = 10
color = [255, 0, 0]  # RGB color values
history_duration = 10.0
max_points = 1000

# Looped pattern configuration
[pattern]
enabled = true
steps = [
    { target_angle = 180.0, duration = 2.0 },
    { target_angle = 220.0, duration = 2.0 }
]
```

### Track Configuration Fields

- `servo_id`: The unique ID of the servo to control
- `color`: RGB color values for the track in the visualization
- `history_duration`: How many seconds of history to show (default: 10.0)
- `max_points`: Maximum number of data points to store (default: 1000)
- `pattern`: Configuration for the looped movement pattern
  - `enabled`: Whether the pattern is enabled by default
  - `steps`: Array of movement steps, each with:
    - `target_angle`: The target angle for the servo
    - `duration`: How long to hold this position (in seconds)

## Usage

Run the application with:

```bash
cargo run
```

You can specify a custom tracks directory:

```bash
cargo run /path/to/tracks
```

## Managing Track Configurations

The application provides buttons to:

1. **Reload Track Configurations** - Load any changes made to the TOML files
2. **Save Track Configurations** - Save the current track settings to TOML files

This allows you to edit patterns in the UI and save them for future use.

## Compatibility

This repo is intended for use with the [servo-server](https://github.com/drbh/servo-server) repo that provides the backend for controlling servo motors. Ensure that the server is running and accessible before starting the Leg Studio application.
