#!/usr/bin/env python3
import argparse
import asyncio
import json
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Response
from fastapi.middleware.cors import CORSMiddleware
import yaml

# Try ROS 2 rclpy (optional)
ROS_AVAILABLE = True
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
except Exception:  # pragma: no cover
    ROS_AVAILABLE = False
    Node = object
    Bool = None
    String = None


@dataclass
class RobotState:
    name: str
    returned: bool = False


@dataclass
class BridgeState:
    robots: Dict[str, RobotState] = field(default_factory=dict)
    qr_detections: List[Dict] = field(default_factory=list)

    def snapshot(self):
        return {
            "robots": [{"name": r.name, "returned": r.returned} for r in self.robots.values()],
            "qrDetections": self.qr_detections[-200:],  # limit
        }


class EventBus:
    """Thread-safe event bus delivering to async consumers."""

    def __init__(self) -> None:
        self.q: Optional[asyncio.Queue] = None

    def attach_loop(self):
        # Create a queue bound to the current running loop
        self.q = asyncio.Queue()

    async def put(self, item: dict):
        if self.q is None:
            # Should not happen once startup completed; drop or buffer
            return
        await self.q.put(item)

    async def get(self):
        if self.q is None:
            # Wait until queue is available
            while self.q is None:
                await asyncio.sleep(0.01)
        return await self.q.get()

    def put_threadsafe(self, loop: asyncio.AbstractEventLoop, item: dict):
        asyncio.run_coroutine_threadsafe(self.put(item), loop)


class MissionBridgeNode(Node):  # type: ignore
    def __init__(self, state: BridgeState, eventbus: EventBus, loop: asyncio.AbstractEventLoop, config: dict):
        super().__init__('mission_timer_bridge')
        self.state = state
        self.eventbus = eventbus
        self.loop = loop

        # Robot returned topics
        self.robot_subs = []
        subs_dbg = []
        for robot in config.get('robots', []):
            name = robot['name']
            topic = robot['returned_topic']
            self.state.robots.setdefault(name, RobotState(name=name))

            def make_cb(robot_name: str):
                def cb(msg: Bool):
                    rs = self.state.robots[robot_name]
                    new_val = bool(msg.data)
                    if rs.returned != new_val:
                        rs.returned = new_val
                        # emit event
                        print(f"[bridge/ros] robot_returned {robot_name} -> {new_val}")
                        self.eventbus.put_threadsafe(self.loop, {
                            "type": "robot_returned",
                            "name": robot_name,
                            "returned": new_val,
                        })
                return cb

            sub = self.create_subscription(Bool, topic, make_cb(name), 10)
            self.robot_subs.append(sub)
            subs_dbg.append((name, topic))

        # QR topic (optional)
        qr_topic = config.get('qr_topic')
        if qr_topic:
            self.create_subscription(String, qr_topic, self.qr_cb, 10)
        # Debug print
        print(f"[bridge/ros] node up. robot_subs={subs_dbg}, qr_topic={qr_topic}")

    def qr_cb(self, msg: String):
        entry = {"value": msg.data, "timestamp": time.strftime('%Y-%m-%dT%H:%M:%S')}
        self.state.qr_detections.append(entry)
        print(f"[bridge/ros] qr {entry['value']} at {entry['timestamp']}")
        self.eventbus.put_threadsafe(self.loop, {"type": "qr", **entry})


def start_ros_thread(state: BridgeState, eventbus: EventBus, loop: asyncio.AbstractEventLoop, config: dict):
    if not ROS_AVAILABLE:
        print('[bridge] ROS not available, skipping rclpy spin (use --mock to simulate).')
        return None

    def run():
        rclpy.init()
        node = MissionBridgeNode(state, eventbus, loop, config)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()

    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


def start_mock_thread(state: BridgeState, eventbus: EventBus, loop: asyncio.AbstractEventLoop):
    def run():
        i = 0
        # init robots if empty
        if not state.robots:
            state.robots["Robot A"] = RobotState(name="Robot A", returned=True)
            state.robots["Robot B"] = RobotState(name="Robot B", returned=False)
        while True:
            time.sleep(5)
            # toggle a robot returned
            for name in list(state.robots.keys()):
                rs = state.robots[name]
                rs.returned = not rs.returned
                eventbus.put_threadsafe(loop, {"type": "robot_returned", "name": name, "returned": rs.returned})
                break
            # add a fake qr
            i += 1
            entry = {"value": f"QR-{i}", "timestamp": time.strftime('%Y-%m-%dT%H:%M:%S')}
            state.qr_detections.append(entry)
            eventbus.put_threadsafe(loop, {"type": "qr", **entry})

    th = threading.Thread(target=run, daemon=True)
    th.start()
    return th


def create_app(config: dict, use_mock: bool = False):
    app = FastAPI()
    app.add_middleware(CORSMiddleware, allow_origins=['*'], allow_methods=['*'], allow_headers=['*'])

    # Private Network Access (PNA) compatibility: add header on all responses
    @app.middleware("http")
    async def add_pna_header(request, call_next):
        resp = await call_next(request)
        # Allow public (Pages) → local (localhost/127.0.0.1) by acknowledging PNA
        resp.headers['Access-Control-Allow-Private-Network'] = 'true'
        return resp

    state = BridgeState()
    eventbus = EventBus()

    # Defer starting ROS/mock threads until ASGI loop is running
    @app.on_event("startup")
    async def on_startup():
        loop = asyncio.get_running_loop()
        eventbus.attach_loop()
        if use_mock or not ROS_AVAILABLE:
            print(f"[bridge] Mode: MOCK (ROS_AVAILABLE={ROS_AVAILABLE}, use_mock={use_mock})")
            start_mock_thread(state, eventbus, loop)
        else:
            print(f"[bridge] Mode: ROS2 (ROS_AVAILABLE={ROS_AVAILABLE})")
            start_ros_thread(state, eventbus, loop, config)

    @app.get('/api/state')
    async def get_state():
        return state.snapshot()

    # Health root for PNA probe (Chrome may request '/')
    @app.get('/')
    async def root():
        return {"ok": True, "service": "mission-timer-bridge"}

    @app.get('/favicon.ico')
    async def favicon():
        return Response(status_code=204)

    @app.websocket('/ws')
    async def ws(ws: WebSocket):
        await ws.accept()
        # send snapshot first
        await ws.send_text(json.dumps({"type": "snapshot", **state.snapshot()}))
        try:
            while True:
                try:
                    evt = await asyncio.wait_for(eventbus.get(), timeout=10.0)
                except asyncio.TimeoutError:
                    # keep alive
                    await ws.send_text(json.dumps({"type": "ping", "t": time.time()}))
                    continue
                await ws.send_text(json.dumps(evt))
        except WebSocketDisconnect:
            return

    return app


def main():
    parser = argparse.ArgumentParser(description='Mission Timer ROS2 Bridge')
    parser.add_argument('--config', type=str, default=None)
    parser.add_argument('--host', type=str, default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--mock', action='store_true', help='Run without ROS2 and generate mock events')
    parser.add_argument('--ssl-certfile', type=str, default=None, help='TLS certificate file for WSS')
    parser.add_argument('--ssl-keyfile', type=str, default=None, help='TLS private key file for WSS')
    args = parser.parse_args()

    config = {}
    if args.config:
        with open(args.config, 'r') as f:
            config = yaml.safe_load(f) or {}

    app = create_app(config, use_mock=args.mock)

    # Lazy import so module import works without uvicorn for tooling/dry-run
    try:
        import uvicorn  # type: ignore
    except Exception as e:
        raise SystemExit(f"uvicorn not available: {e}. Install with 'pip install uvicorn fastapi pyyaml'.")

    uvicorn_kwargs = dict(host=args.host, port=args.port)
    if args.ssl_certfile and args.ssl_keyfile:
        uvicorn_kwargs.update({
            'ssl_certfile': args.ssl_certfile,
            'ssl_keyfile': args.ssl_keyfile,
        })
        scheme = 'wss'
    else:
        scheme = 'ws'

    print(f"[bridge] Starting server at {scheme}://{args.host}:{args.port}/ws")
    uvicorn.run(app, **uvicorn_kwargs)


if __name__ == '__main__':
    main()
