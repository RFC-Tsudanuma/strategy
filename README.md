# Strategy

A soccer strategy package for humanoid robots participating in the RoboCup Humanoid League. It implements a decision-making system based on HTN (Hierarchical Task Network) planning on ROS 2.

## Overview

This package serves as the strategic decision-making system enabling humanoid robots to play soccer. Using HTN planning with GTPyhop, it derives specific robot actions from high-level goals.

### Key Features

- HTN-based hierarchical task planning
- Role allocation system (Attacker, Defender, Follower, etc.)
- RRT* path planning
- Ball prediction and filtering
- Inter-robot communication (Teammate communication)
- RoboCup GameController support
- Debug visualizer

## Technical Details

### HTN Planning

GTPyhop is used to perform hierarchical decomposition from goals to actions. Tasks are decomposed into subtasks via methods, eventually reaching primitive actions.

### Role System

Each robot is assigned one of the following roles based on the match situation:
- **Attacker**: Moves towards the ball and aims for the goal.
- **Defender**: Defends the own goal.
- **Follower**: Supports teammates.
- **Neutral**: Standby state.

### Coordinate Systems

- **Field Coordinate System**: Center is (0,0), Right is +x, Up is +y (Right-handed system).
- **Robot Coordinate System**: Forward is +x, Left is +y (Right-handed system).

# Environment Setup
`make venv`

# Usage
1. `source .venv/bin/activate`
2. Build this package.
3. `ros2 run strategy main.py`
    - For simulation: `ros2 run strategy main.py --sim-robot-id [robot_id]`
4. To enable neck movements, launch the neck module: https://github.com/RFC-Tsudanuma/neck_movement
5. To receive signals from the GameController, launch the game_controller node: https://github.com/RFC-Tsudanuma/robocup_demo

## Running the Debug Visualizer
1. `source .venv/bin/activate`
2. `ros2 run strategy strategy_state_visualizer.py`
    - For simulation: `ros2 run strategy strategy_state_visualizer.py -- --sim-robot-id [robot_id]`


## License

MIT License - See [LICENSE](LICENSE) for details.
This repository used this https://github.com/Geson-anko/ros2_uv_template template.