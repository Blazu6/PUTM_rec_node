# RecNode - Data Recording Node for Formula Student

## Overview

`RecNode` is a ROS2 node developed for the Formula Student team at Poznań University of Technology. It manages the recording of vehicle data using the `ros2 bag record` command, starting and stopping recording based on the car's readiness state.

## How It Works

- **Car Ready (`state = true`)**: Starts recording immediately.
- **Car Not Ready (`state = false`)**: Stops recording after 20 seconds.

The node works in symbiosis with other programs running as systemd services and listens to the `rtd` topic to control the recording.

## Key Features

- Starts and stops data recording automatically based on the car’s readiness state.
- Saves recordings with timestamps in filenames.
- Runs `ros2 bag record` for data logging in the background.
