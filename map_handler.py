import numpy as np

class MapHandler:
    def __init__(self, ws_server):
        self.ws_server = ws_server

    def handle_map(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width)).tolist()
        map_json = {
            'type': 'map',
            'width': width,
            'height': height,
            'resolution': msg.info.resolution,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'theta': 0  # For simplicity
            },
            'data': data
        }
        self.ws_server.broadcast(map_json)
