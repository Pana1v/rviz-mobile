import asyncio
import rclpy
from ros2_bridge import ROS2Bridge
from websocket_server import WebSocketServer
from video_streamer import VideoStreamer
from map_handler import MapHandler
from teleop_controller import TeleopController
import threading

def main():
    rclpy.init()
    ros2_bridge = ROS2Bridge()
    ws_server = WebSocketServer()
    video_streamer = VideoStreamer(ws_server)
    map_handler = MapHandler(ws_server)
    teleop_controller = TeleopController(ros2_bridge)

    ros2_bridge.set_image_callback(video_streamer.handle_image)
    ros2_bridge.set_map_callback(map_handler.handle_map)
    ws_server.set_teleop_handler(teleop_controller.handle_teleop)

    def ros_spin():
        rclpy.spin(ros2_bridge)

    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    async def main_async():
        ws = await ws_server.start()
        print("WebSocket server started.")
        try:
            while ros_thread.is_alive():
                await asyncio.sleep(0.1)
        except KeyboardInterrupt:
            pass

    asyncio.run(main_async())
    ros2_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
