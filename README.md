# Controller Package F1TENTH

High-precision vehicle controller for F1TENTH autonomous racing. Executes planned trajectories with velocity control and safety monitoring.

Final component of the F1TENTH autonomous racing system - converts planned paths into drive commands.

## Quick Start

```bash
# Build (from workspace root)
colcon build --packages-select controller_pkg --symlink-install
source install/setup.bash

# For Real Car
ros2 launch controller_pkg controller.launch.py sim_mode:=false

# For Simulation (default)
ros2 launch controller_pkg controller.launch.py sim_mode:=true
```

## Prerequisites

1. **Localization running**: Particle filter for pose estimation
2. **Path planner running**: Lattice planner generating `/planned_waypoints` 
3. **Hardware/Simulation ready**: F1TENTH system or gym bridge

## Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim_mode` | `true` | Use simulation topics if true, real car topics if false |

## Topic Remapping

### Real Car Mode (`sim_mode:=false`)
- **Input Waypoints**: `/planned_waypoints` (from lattice planner)
- **Input Odometry**: `/pf/pose/odom` (from particle filter)
- **Output Drive**: `/drive` (to F1TENTH hardware)

### Simulation Mode (`sim_mode:=true`)
- **Input Waypoints**: `/planned_waypoints` (from lattice planner)
- **Input Odometry**: `/ego_racecar/odom` (from F1TENTH Gym)
- **Output Drive**: `/drive` (to simulation)

## Configuration

Edit `config/controller_config.yaml` to tune:

### Control Parameters
- **Lookahead Distance**: How far ahead to target on path
- **Speed Control**: PID gains for velocity tracking
- **Steering Control**: Pure pursuit or Stanley controller gains
- **Safety Limits**: Maximum steering angle and acceleration

### Performance Tuning
- **Update Rate**: Control loop frequency (typically 20-50 Hz)
- **Path Interpolation**: Smoothing parameters
- **Error Tolerances**: Position and heading thresholds

## Control Algorithm

The controller uses a **hybrid control approach**:

1. **Path Tracking**: Pure pursuit for smooth steering
2. **Velocity Control**: PID controller for speed regulation
3. **Safety Checks**: Real-time collision monitoring
4. **Command Output**: Ackermann drive messages

### Control Flow
```
Planned Waypoints → Waypoint Selection → Lookahead Point → Steering Command
       ↓                                                         ↓
   Velocity Profile → Speed Controller → Throttle/Brake → Ackermann Drive
```

## Safety Monitor (Optional)

The safety monitor provides additional collision protection:

```bash
# Launch with safety monitor (if available)
ros2 launch controller_pkg controller_with_safety.launch.py
```

**Safety Features:**
- **Emergency Braking**: Immediate stop on collision risk
- **Speed Limiting**: Reduce speed in tight spaces
- **Rollback**: Return to last safe state if needed

Configure safety parameters in the controller configuration file.

## Key Topics

**Subscribes:**
- `/planned_waypoints` - Trajectory from lattice planner (`crazy_planner_msgs/WaypointArray`)
- `/odom` - Robot pose and velocity
- `/scan` - Laser data (for safety monitor)

**Publishes:**
- `/drive` - Ackermann drive commands (`AckermannDriveStamped`)
- **Debug Topics**:
  - `/control/lookahead_point` - Target point visualization
  - `/control/path_error` - Tracking error metrics
  - `/control/cmd_vel` - Control commands before conversion

## Integration with System

**Complete Launch Order:**
1. Localization (particle filter)
2. Hardware/Simulation bridge  
3. Path planner (lattice planner)
4. **→ Controller ←**

**System Data Flow:**
```
Sensors → Localization → Planning → Control → Actuators
                           ↑          ↓
                    Reference Path  Drive Commands
```

## Control Performance

- **High Frequency**: 20-50 Hz control updates
- **Low Latency**: <50ms path-to-command delay
- **Smooth Tracking**: Minimal path deviation and oscillation
- **Speed Accuracy**: ±0.1 m/s velocity tracking
- **Safety Critical**: Real-time collision avoidance

## Message Types

Uses custom messages from `crazy_planner_msgs`:
- **Input**: `WaypointArray` - Waypoints with target speeds and track information
- **Output**: `AckermannDriveStamped` - Steering angle and speed commands

## Tuning Guidelines

1. **Start Conservative**: Low speeds, gentle steering
2. **Tune Lookahead**: Longer for stability, shorter for precision  
3. **Adjust PID Gains**: Start with proportional-only control
4. **Test Progressively**: Increase speed as performance improves
5. **Monitor Safety**: Always test with safety monitor enabled

Built for competitive F1TENTH racing with safety-first control architecture.