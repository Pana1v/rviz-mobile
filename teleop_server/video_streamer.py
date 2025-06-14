import cv2
import base64
from cv_bridge import CvBridge

class VideoStreamer:
    def __init__(self, ws_server):
        self.bridge = CvBridge()
        self.ws_server = ws_server
        self.last_sent = 0
        self.min_interval = 1.0 / 15  # 15 FPS target

    def handle_image(self, msg):
        import time
        now = time.time()
        if now - self.last_sent < self.min_interval:
            return  # Drop frame to maintain FPS
        self.last_sent = now
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Resize for bandwidth (e.g., 320x240)
        cv_image = cv2.resize(cv_image, (320, 240), interpolation=cv2.INTER_AREA)
        # Compress JPEG with quality 40 (smaller size, faster transmission)
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]
        _, jpeg = cv2.imencode('.jpg', cv_image, encode_param)
        b64 = base64.b64encode(jpeg.tobytes()).decode('utf-8')
        # Send to all connected clients
        self.ws_server.broadcast({'type': 'video', 'data': b64})
