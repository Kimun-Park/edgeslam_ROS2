# Multi-Robot Edge-SLAM Launch Files

## Usage

### 1. Start the Edge Server
```bash
ros2 launch edge_slam_ros2 server.launch.py
```

### 2. Start Robot Clients
```bash
# Robot 1
export EDGE_SLAM_ROBOT_ID=1
ros2 launch edge_slam_ros2 multi_robot_rgbd.launch.py robot_id:=1 run_type:=client

# Robot 2  
export EDGE_SLAM_ROBOT_ID=2
ros2 launch edge_slam_ros2 multi_robot_rgbd.launch.py robot_id:=2 run_type:=client

# Robot 3
export EDGE_SLAM_ROBOT_ID=3
ros2 launch edge_slam_ros2 multi_robot_rgbd.launch.py robot_id:=3 run_type:=client
```

## Port Allocation

The system automatically assigns ports based on robot ID:
- **Robot 1**: Ports 15003, 15004, 15005 (keyframe, frame, map)
- **Robot 2**: Ports 15006, 15007, 15008
- **Robot 3**: Ports 15009, 15010, 15011
- etc.

## Environment Variables

- `EDGE_SLAM_ROBOT_ID`: Robot identifier (1, 2, 3, ...)
- `EDGE_SLAM_SERVER_IP`: Edge server IP (default: 127.0.0.1)
- `EDGE_SLAM_BASE_PORT`: Base port number (default: 15000)

## Isaac Sim Testing

For Isaac Sim environments, all defaults are pre-configured for localhost testing.
Simply run the commands above without additional configuration.

## Custom Configuration

```bash
# Custom server IP and ports
ros2 launch edge_slam_ros2 server.launch.py server_ip:=192.168.1.100 base_port:=20000

ros2 launch edge_slam_ros2 multi_robot_rgbd.launch.py \
    robot_id:=1 \
    run_type:=client \
    server_ip:=192.168.1.100 \
    base_port:=20000
```