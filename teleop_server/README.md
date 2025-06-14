# ROS2 Humble Phone Teleop Server

A modular, mobile-friendly teleoperation dashboard for ROS2 robots, featuring real-time camera streaming, map visualization, and joystick control—all accessible from your phone or PC browser.

## Features
- **Modular Python backend** using ROS2, websockets, OpenCV, and asyncio
- **Web client**: Responsive, landscape-optimized UI for Android/iOS/PC
- **Real-time camera streaming** (compressed JPEG over WebSocket)
- **Live map visualization** with robot pose overlay
- **Touch joystick** for teleop (geometry_msgs/Twist)
- **No authentication** (for local WiFi use)

## Project Structure
```
teleop_server/
├── ros2_bridge.py          # ROS2 node: subscriptions/publications
├── websocket_server.py     # WebSocket server for client comms
├── video_streamer.py       # Camera feed streaming handler
├── map_handler.py          # Map data processing/transmission
├── teleop_controller.py    # Teleop command processing
├── main.py                 # Application entry point
└── web_client/
    ├── index.html          # Main UI layout
    ├── joystick.js         # Virtual joystick
    ├── map_viewer.js       # Map visualization
    ├── video_player.js     # Camera feed display
    └── styles.css          # Responsive styling
```

## Setup Instructions

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install -y python3-pip ros-humble-desktop
pip3 install rclpy websockets opencv-python numpy cv_bridge
```

### 2. Run the Backend
```bash
cd teleop_server
python3 main.py
```

### 3. Serve the Web Client
```bash
cd teleop_server/web_client
python3 -m http.server 8080
```

### 4. Access the Dashboard
- On your phone/PC browser: `http://<your-pc-ip>:8080`
- (Replace `<your-pc-ip>` with your ROS2 PC's local IP address)

## Communication
- **WebSocket**: `ws://<your-pc-ip>:8765` (auto-connected by frontend)
- **ROS2 Topics**:
  - `/camera/image_raw` (sensor_msgs/Image)
  - `/map` (nav_msgs/OccupancyGrid)
  - `/cmd_vel` (geometry_msgs/Twist)

## Customization
- Adjust video resolution/quality in `video_streamer.py`
- Change joystick sensitivity in `joystick.js`
- Add robot pose publishing in `map_handler.py` for pose overlay

## License
MIT

---

## Contributing
Pull requests welcome! Please open issues for bugs or feature requests.
