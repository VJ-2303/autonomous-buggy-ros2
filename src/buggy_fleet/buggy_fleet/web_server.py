"""
Web Server & ROS 2 Bridge: FastAPI application providing REST API, WebSockets,
and OpenStreetMap live tracking for the KCT Autonomous Buggy.
"""

import asyncio
import json
import os
import threading
from typing import Dict, List, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import rclpy
import rclpy.node
from std_msgs.msg import Empty, String
import uvicorn

from .kct_map import KctMap

app = FastAPI(title="KCT Autonomous Buggy Portal")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Shared in-memory telemetry state
latest_telemetry: Dict = {}
connected_clients: Set[WebSocket] = set()
ros_bridge_node: Optional["WebBridgeNode"] = None
kct_map = KctMap()


class WebBridgeNode(rclpy.node.Node):
    """ROS 2 Node running inside FastAPI to bridge topics with WebSockets."""

    def __init__(self):
        super().__init__("web_bridge_node")

        self.sub_telemetry = self.create_subscription(
            String, "/buggy/telemetry", self.on_telemetry, 10
        )
        self.pub_book = self.create_publisher(String, "/buggy/book_request", 10)
        self.pub_reset = self.create_publisher(Empty, "/buggy/reset", 10)

        self.get_logger().info("Web Bridge ROS 2 Node initialized.")

    def on_telemetry(self, msg: String):
        global latest_telemetry
        try:
            latest_telemetry = json.loads(msg.data)
        except Exception:
            pass

    def request_booking(self, pickup: str, drop: str):
        msg = String()
        msg.data = json.dumps({"pickup": pickup, "drop": drop})
        self.pub_book.publish(msg)

    def request_reset(self):
        self.pub_reset.publish(Empty())


@app.get("/api/map")
async def get_map():
    """Return the complete georeferenced KCT campus map as GeoJSON."""
    return JSONResponse(kct_map.to_geojson())


@app.get("/api/status")
async def get_status():
    """Return latest vehicle telemetry and rider queue."""
    return JSONResponse(latest_telemetry)


@app.post("/api/book")
async def book_ride(payload: Dict):
    """Book a car ride between two campus locations."""
    pickup = payload.get("pickup")
    drop = payload.get("drop")

    if not pickup or not drop:
        return JSONResponse({"status": "error", "message": "Pickup and drop are required."}, status_code=400)

    if ros_bridge_node:
        ros_bridge_node.request_booking(str(pickup), str(drop))
        return JSONResponse({"status": "submitted", "pickup": pickup, "drop": drop})
    return JSONResponse({"status": "error", "message": "ROS bridge not connected."}, status_code=503)


@app.post("/api/reset")
async def reset_car():
    """Reset vehicle position and bookings."""
    if ros_bridge_node:
        ros_bridge_node.request_reset()
        return JSONResponse({"status": "reset_requested"})
    return JSONResponse({"status": "error", "message": "ROS bridge not connected."}, status_code=503)


@app.websocket("/ws")
async def websocket_telemetry(websocket: WebSocket):
    """Live WebSocket feed streaming vehicle position, route, and bookings to Leaflet map."""
    await websocket.accept()
    connected_clients.add(websocket)
    try:
        while True:
            if latest_telemetry:
                await websocket.send_json(latest_telemetry)
            await asyncio.sleep(0.1)  # 10 Hz broadcast
    except WebSocketDisconnect:
        connected_clients.remove(websocket)
    except Exception:
        if websocket in connected_clients:
            connected_clients.remove(websocket)


def _find_static_dir() -> Optional[str]:
    candidates = [
        os.path.expanduser("~/Code/autonomous/web/static"),
        os.path.abspath("web/static"),
    ]
    curr = os.path.abspath(__file__)
    for _ in range(7):
        candidates.append(os.path.join(curr, "web", "static"))
        curr = os.path.dirname(curr)
    for c in candidates:
        if os.path.isdir(c) and os.path.exists(os.path.join(c, "index.html")):
            return c
    return None


static_dir = _find_static_dir()
if static_dir:
    app.mount("/static", StaticFiles(directory=static_dir), name="static")

    @app.get("/")
    async def serve_index():
        return FileResponse(os.path.join(static_dir, "index.html"))


def start_ros_spin():
    """Spin ROS 2 node in background thread."""
    global ros_bridge_node
    rclpy.init()
    ros_bridge_node = WebBridgeNode()
    rclpy.spin(ros_bridge_node)


def main():
    """Main entrypoint for web_server."""
    # Start ROS 2 thread
    ros_thread = threading.Thread(target=start_ros_spin, daemon=True)
    ros_thread.start()

    # Start FastAPI server
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")


if __name__ == "__main__":
    main()
