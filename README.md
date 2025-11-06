# ROS 2 Jazzy × Unity Integration Project

This project provides a ROS 2 Jazzy environment running in Docker on WSL2, designed to communicate with Unity using the [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) bridge.

## Architecture

- **Windows 11**: Host system running Unity with ros2-for-unity
- **WSL2 Ubuntu 24.04**: Docker host
- **Docker Container**: ROS 2 Jazzy desktop-full environment
- **Communication**: DDS (Cyclone DDS) for low-latency bidirectional communication

## Quick Start

### Prerequisites
- Windows 11 with WSL2 enabled
- Docker Desktop with WSL2 integration
- Ubuntu 24.04 WSL distribution

### Setup

1. **Clone/Navigate to workspace:**
   ```bash
   cd ~/ros2_ws
   ```

2. **Start with Docker Compose:**
   ```bash
   docker-compose up -d
   docker exec -it ros2_jazzy bash
   ```

3. **Build the workspace (inside container):**
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ```

### Testing Communication

1. **Start Status Publisher:**
   ```bash
   ros2 run unity_bridge_py status_publisher
   ```

2. **In another terminal, test subscriber:**
   ```bash
   ros2 topic echo /unity/status
   ```

3. **Test Command Publisher:**
   ```bash
   ros2 topic pub /unity/cmd std_msgs/String 'data: "move forward"' --once
   ```

4. **Start Command Subscriber:**
   ```bash
   ros2 run unity_bridge_py cmd_subscriber
   ```

## Topics

- `/unity/status`: Published by ROS 2, subscribed by Unity (status updates)
- `/unity/cmd`: Published by Unity, subscribed by ROS 2 (commands)

## Development with Cursor

1. Open Cursor in WSL: `cursor ~/ros2_ws`
2. Use "Reopen in Container" to work inside the ROS 2 environment
3. The `.devcontainer/devcontainer.json` is configured for seamless development

## Configuration

- **DDS**: Cyclone DDS with multicast enabled
- **Domain ID**: 0 (ensure Unity uses the same)
- **Network**: Host networking for optimal performance

## Unity Integration

When setting up Unity with ros2-for-unity:
1. Ensure `ROS_DOMAIN_ID=0` in Unity environment
2. Use the same DDS implementation (Cyclone DDS recommended)
3. Subscribe to `/unity/status` and publish to `/unity/cmd`

## Package Structure

```
ros2_ws/
├── src/
│   └── unity_bridge_py/          # Python bridge package
│       ├── unity_bridge_py/
│       │   ├── status_publisher.py    # Publishes status to Unity
│       │   └── cmd_subscriber.py      # Receives commands from Unity
│       └── setup.py
├── .devcontainer/
│   └── devcontainer.json         # Cursor dev container config
├── docker-compose.yml            # Docker setup
├── cyclonedds.xml               # DDS configuration
└── README.md
```

## Troubleshooting

1. **No topics visible**: Ensure nodes are running and DDS configuration matches
2. **GUI not working**: Verify WSLg is enabled and X11 forwarding is configured
3. **Unity can't connect**: Check firewall settings and ROS_DOMAIN_ID consistency

## Next Steps

- Add custom message types in `unity_bridge_msgs` package
- Integrate with MoveIt 2 for robot control
- Add sensor simulation (LiDAR, cameras)
- Implement Nav2 integration for navigation
