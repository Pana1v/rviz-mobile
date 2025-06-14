import asyncio
import websockets
import json

class WebSocketServer:
    def __init__(self, host='0.0.0.0', port=8765):
        self.clients = set()
        self.host = host
        self.port = port
        self.teleop_handler = None
        self.loop = None  # Store main event loop

    def set_teleop_handler(self, handler):
        self.teleop_handler = handler

    async def handler(self, websocket):
        self.clients.add(websocket)
        try:
            async for message in websocket:
                data = json.loads(message)
                if data.get('type') == 'teleop' and self.teleop_handler:
                    self.teleop_handler(data)
        finally:
            self.clients.remove(websocket)

    def broadcast(self, data):
        msg = json.dumps(data)
        if self.loop is not None:
            self.loop.call_soon_threadsafe(asyncio.create_task, self._broadcast(msg))
        else:
            # fallback for direct asyncio context
            asyncio.create_task(self._broadcast(msg))

    async def _broadcast(self, msg):
        for ws in list(self.clients):
            try:
                await ws.send(msg)
            except:
                self.clients.remove(ws)

    async def start(self):
        self.loop = asyncio.get_running_loop()
        return await websockets.serve(self.handler, self.host, self.port)
