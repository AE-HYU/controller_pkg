# Controller Package

Pure pursuit controller for F1TENTH autonomous racing with velocity control and safety monitoring.

## Quick Start

```bash
# Build
colcon build --packages-select controller_pkg
source install/setup.bash

# Real car
ros2 launch controller_pkg controller.launch.py

# Simulation
ros2 launch controller_pkg controller.launch.py sim_mode:=true
```

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `false` | Use simulation topics if true |

## Topics

### Input
- `/planned_waypoints` - Path from lattice planner (`ae_hyu_msgs/WpntArray`)
- `/pf/pose/odom` (real) / `/ego_racecar/odom` (sim) - Vehicle pose

### Output
- `/drive` - Ackermann drive commands (`ackermann_msgs/AckermannDriveStamped`)

## Key Configuration

Edit `config/controller_config.yaml`:

```yaml
# Pure Pursuit
lookahead_distance: 2.5       # Target point distance (m)
lookahead_ratio: 0.4          # Speed-dependent lookahead
min_lookahead: 0.5            # Minimum lookahead (m)
max_lookahead: 3.0            # Maximum lookahead (m)

# Vehicle
wheelbase: 0.35               # Vehicle wheelbase (m)
max_steering_angle: 0.4       # Max steering (rad)
max_steering_rate: 2.0        # Max steering rate (rad/s)

# Speed Control
target_speed: 5.0             # Default speed (m/s)
max_speed: 8.0                # Speed limit (m/s)
use_planner_velocity: true    # Use waypoint speeds
velocity_scale_factor: 1.0    # Speed scaling factor
```

## Control Algorithm

1. **Waypoint Selection**: Find target point based on lookahead distance
2. **Pure Pursuit**: Calculate steering angle to target point
3. **Speed Control**: Use waypoint velocities or constant speed
4. **Safety Limits**: Apply steering and speed constraints
5. **Command Output**: Publish Ackermann drive messages

## Prerequisites

- Lattice planner running (`/planned_waypoints` topic)
- Localization running (odometry)
- Vehicle hardware or simulation ready