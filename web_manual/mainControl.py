#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import json
import math
import shlex
import time
import uuid
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse
from dataclasses import dataclass
from threading import Event, Lock
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt


MQTT_BROKER = "172.16.0.103"
# MQTT_BROKER = "localhost"
MQTT_PORT = 9001
ROBOT_ID = "9190"
MANUFACTURER = "HikRobot"

USE_WEBSOCKETS = True
WS_PATH: Optional[str] = "/mqtt"

MQTT_USERNAME: Optional[str] = None
MQTT_PASSWORD: Optional[str] = None

MQTT_TOPIC_ORDER = f"VDA/V2.0.0/Robot/{ROBOT_ID}/order"
MQTT_TOPIC_STATE = f"VDA/V2.0.0/Robot/{ROBOT_ID}/state"
MQTT_TOPIC_INSTANTACTIONS = f"VDA/V2.0.0/Robot/{ROBOT_ID}/instantActions"

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_manual")

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
}

SEND_LOCK = threading.Lock()

MQTT_QOS = 0
MQTT_RETAIN = False

STEP = 0.50
MAX_SPEED = 0.5

DEFAULT_MAP_DESCRIPTION = "default"
DEFAULT_MAP_ID = "XX"

ALLOWED_DEVIATION_XY = 0.2
ALLOWED_DEVIATION_THETA = math.pi

ROTATE_SELF = "ROTATE_SELF"
ROTATE_CLOCKWISE = "ROTATE_CLOCKWISE"
ROTATE_COUNTERCLOCKWISE = "ROTATE_COUNTERCLOCKWISE"
ROTATE_DIRECTION_CHOICES = (ROTATE_SELF, ROTATE_CLOCKWISE, ROTATE_COUNTERCLOCKWISE)
ROTATE_DIRECTION_ALIASES = {
    "self": ROTATE_SELF,
    "cw": ROTATE_CLOCKWISE,
    "ccw": ROTATE_COUNTERCLOCKWISE,
}

MODE_ROBOT = "robot"
MODE_MAP = "map"
MODE_CHOICES = (MODE_ROBOT, MODE_MAP)


@dataclass
class Pose:
    x: float
    y: float
    theta: float
    map_id: str
    map_description: str


_pose_lock = Lock()
_latest_pose: Optional[Pose] = None
_pose_ready = Event()

_client: Optional[mqtt.Client] = None
_current_order_id: str = str(uuid.uuid4())
_current_update_id: int = 0


def _utc_timestamp() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())


def reset_order_session():
    global _current_order_id, _current_update_id
    _current_order_id = str(uuid.uuid4())
    _current_update_id = 0


def _enabled_map_id(state: Dict[str, Any]) -> str:
    maps = state.get("maps", [])
    if isinstance(maps, list):
        for m in maps:
            if isinstance(m, dict) and m.get("mapStatus") == "ENABLED" and m.get("mapId"):
                return str(m["mapId"])
    return DEFAULT_MAP_ID


def _pose_from_node_states(state: Dict[str, Any]) -> Optional[Pose]:
    node_states = state.get("nodeStates", [])
    if not isinstance(node_states, list) or not node_states:
        return None

    target_seq = state.get("lastNodeSequenceId")
    chosen = None

    if isinstance(target_seq, int):
        for ns in node_states:
            if isinstance(ns, dict) and ns.get("sequenceId") == target_seq:
                chosen = ns
                break

    if chosen is None:
        chosen = node_states[0] if isinstance(node_states[0], dict) else None

    if not isinstance(chosen, dict):
        return None

    pos = chosen.get("nodePosition", {})
    if not isinstance(pos, dict):
        return None

    try:
        x = float(pos["x"])
        y = float(pos["y"])
        theta = float(pos.get("theta", 0.0))
        map_desc = str(pos.get("mapDescription") or DEFAULT_MAP_DESCRIPTION)
        map_id = str(pos.get("mapId") or "")
        if not map_id:
            map_id = _enabled_map_id(state)
        return Pose(x=x, y=y, theta=theta, map_id=map_id, map_description=map_desc)
    except Exception:
        return None


def _pose_from_agv_position(state: Dict[str, Any]) -> Optional[Pose]:
    ap = state.get("agvPosition", {})
    if not isinstance(ap, dict):
        return None
    if ap.get("positionInitialized") is False:
        return None
    try:
        x = float(ap["x"])
        y = float(ap["y"])
        theta = float(ap.get("theta", 0.0))
        map_id = str(ap.get("mapId") or "")
        if not map_id:
            map_id = _enabled_map_id(state)
        return Pose(x=x, y=y, theta=theta, map_id=map_id, map_description=DEFAULT_MAP_DESCRIPTION)
    except Exception:
        return None


def _try_parse_pose_from_state(state: Dict[str, Any]) -> Optional[Pose]:
    p = _pose_from_agv_position(state)
    if p is not None:
        return p
    return _pose_from_node_states(state)


def _on_connect(
    client: mqtt.Client,
    userdata: Any,
    flags: Dict[str, Any],
    reason_code: Any,
    properties: Any = None,
):
    rc = getattr(reason_code, "value", reason_code)
    if rc == 0:
        client.subscribe(MQTT_TOPIC_STATE, qos=MQTT_QOS)


def _on_message(client: mqtt.Client, userdata: Any, msg: mqtt.MQTTMessage):
    global _latest_pose
    if msg.topic != MQTT_TOPIC_STATE:
        return
    try:
        state = json.loads(msg.payload.decode("utf-8", errors="replace"))
    except Exception:
        return
    pose = _try_parse_pose_from_state(state)
    if pose is None:
        return
    with _pose_lock:
        _latest_pose = pose
        _pose_ready.set()


def _create_mqtt_client(transport: Optional[str] = None) -> mqtt.Client:
    kwargs: Dict[str, Any] = {}
    if transport:
        kwargs["transport"] = transport
    callback_api = getattr(mqtt, "CallbackAPIVersion", None)
    if callback_api is not None:
        kwargs["callback_api_version"] = callback_api.VERSION2
    return mqtt.Client(**kwargs)


def _ensure_client_connected() -> mqtt.Client:
    global _client
    if _client is not None:
        return _client

    if USE_WEBSOCKETS:
        client = _create_mqtt_client(transport="websockets")
        if WS_PATH:
            client.ws_set_options(path=WS_PATH)
    else:
        client = _create_mqtt_client()

    if MQTT_USERNAME is not None:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)

    client.on_connect = _on_connect
    client.on_message = _on_message

    client.connect(MQTT_BROKER, MQTT_PORT, keepalive=60)
    client.loop_start()
    _client = client
    return _client


def disconnect():
    global _client
    if _client is not None:
        _client.loop_stop()
        _client.disconnect()
        _client = None


def _get_latest_pose(timeout_s: float = 10.0) -> Pose:
    _ensure_client_connected()
    if not _pose_ready.wait(timeout=timeout_s):
        raise RuntimeError("No pose received from robot state")
    with _pose_lock:
        if _latest_pose is None:
            raise RuntimeError("Pose is None")
        return _latest_pose


def _get_cached_pose() -> Optional[Pose]:
    with _pose_lock:
        return _latest_pose


def _format_pose(pose: Optional[Pose]) -> str:
    if pose is None:
        return "Pose: n/a"
    return f"Pose: x={pose.x:.3f} y={pose.y:.3f} theta={pose.theta:.3f} mapId={pose.map_id}"


def _normalize_angle_rad(theta: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def _normalize_rotate_direction(value: Optional[str]) -> str:
    if value is None:
        return ROTATE_SELF
    if not isinstance(value, str):
        raise ValueError("rotateDirection must be a string")
    upper = value.upper()
    if upper in ROTATE_DIRECTION_CHOICES:
        return upper
    alias = ROTATE_DIRECTION_ALIASES.get(value.lower())
    if alias:
        return alias
    raise ValueError(f"Invalid rotateDirection: {value}")


def _compute_target_pose_robot(pose: Pose, forward: float, left: float) -> Pose:
    ct = math.cos(pose.theta)
    st = math.sin(pose.theta)
    dx = forward * ct - left * st
    dy = forward * st + left * ct
    return Pose(
        x=pose.x + dx,
        y=pose.y + dy,
        theta=pose.theta,
        map_id=pose.map_id,
        map_description=pose.map_description,
    )


def _build_order_from_poses(
    start: Pose,
    end: Pose,
    edge_orientation: float,
    end_theta: float,
    start_rotate_direction: str = ROTATE_SELF,
) -> Dict[str, Any]:
    global _current_update_id

    start_node_id = str(uuid.uuid4())
    end_node_id = str(uuid.uuid4())
    edge_id = str(uuid.uuid4())

    cp1 = {"weight": 1.0, "x": start.x, "y": start.y}
    cp4 = {"weight": 1.0, "x": end.x, "y": end.y}
    cp2 = {"weight": 1.0, "x": start.x + (end.x - start.x) / 3.0, "y": start.y + (end.y - start.y) / 3.0}
    cp3 = {"weight": 1.0, "x": start.x + 2.0 * (end.x - start.x) / 3.0, "y": start.y + 2.0 * (end.y - start.y) / 3.0}

    start_rotate_direction = _normalize_rotate_direction(start_rotate_direction)
    order = {
        "headerId": 3,
        "manufacturer": MANUFACTURER,
        "version": "2.0.0",
        "serialNumber": ROBOT_ID,
        "timestamp": _utc_timestamp(),
        "zoneSetId": "",
        "orderId": _current_order_id,
        "orderUpdateId": _current_update_id,
        "nodes": [
            {
                "nodeDescription": "",
                "nodeId": start_node_id,
                "sequenceId": 0,
                "released": True,
                "rotateDirection": start_rotate_direction,
                "nodePosition": {
                    "mapDescription": start.map_description,
                    "mapId": start.map_id,
                    "allowedDeviationXy": ALLOWED_DEVIATION_XY,
                    "allowedDeviationTheta": ALLOWED_DEVIATION_THETA,
                    "x": start.x,
                    "y": start.y,
                    "theta": _normalize_angle_rad(start.theta),
                },
            },
            {
                "nodeDescription": "",
                "nodeId": end_node_id,
                "sequenceId": 2,
                "released": True,
                "rotateDirection": ROTATE_SELF,
                "nodePosition": {
                    "mapDescription": end.map_description,
                    "mapId": end.map_id,
                    "allowedDeviationXy": ALLOWED_DEVIATION_XY,
                    "allowedDeviationTheta": ALLOWED_DEVIATION_THETA,
                    "x": end.x,
                    "y": end.y,
                    "theta": _normalize_angle_rad(end_theta),
                },
            },
        ],
        "edges": [
            {
                "edgeDescription": "",
                "edgeId": edge_id,
                "sequenceId": 1,
                "released": True,
                "startNodeId": start_node_id,
                "endNodeId": end_node_id,
                "maxSpeed": MAX_SPEED,
                "orientation": _normalize_angle_rad(edge_orientation),
                "rotationAllowed": True,
                "trajectory": {
                    "controlPoints": [cp1, cp2, cp3, cp4],
                    "degree": 4.0,
                    "knotVector": [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
                },
            }
        ],
    }

    _current_update_id += 1
    return order


def publish_order(order: Dict[str, Any]):
    client = _ensure_client_connected()
    payload = json.dumps(order, ensure_ascii=False)
    info = client.publish(MQTT_TOPIC_ORDER, payload=payload, qos=MQTT_QOS, retain=MQTT_RETAIN)
    info.wait_for_publish()
    print("SENT", order["orderUpdateId"])


def _compute_end_theta(
    start: Pose,
    end_x: float,
    end_y: float,
    reverse: bool,
    end_theta_override_deg: Optional[float],
) -> float:
    dx = end_x - start.x
    dy = end_y - start.y
    vector_theta = math.atan2(dy, dx) if (dx or dy) else start.theta
    if end_theta_override_deg is not None:
        return start.theta + math.radians(end_theta_override_deg)
    if reverse:
        return start.theta
    return vector_theta


def send_move_robot_frame(
    x_right: float,
    y_forward: float,
    reverse: bool = False,
    rotate_direction: str = ROTATE_SELF,
    end_theta_override_deg: Optional[float] = None,
):
    """Robot frame: x=right (+), y=forward (+)."""
    start = _get_latest_pose(timeout_s=15.0)
    # Convert (x_right, y_forward) -> (forward, left)
    forward = y_forward
    left = -x_right
    end = _compute_target_pose_robot(start, forward=forward, left=left)

    end_theta = _compute_end_theta(start, end.x, end.y, reverse, end_theta_override_deg)

    end = Pose(
        x=end.x,
        y=end.y,
        theta=end_theta,
        map_id=start.map_id,
        map_description=start.map_description,
    )
    edge_orientation = math.pi if reverse else 0.0

    order = _build_order_from_poses(
        start,
        end,
        edge_orientation=edge_orientation,
        end_theta=end_theta,
        start_rotate_direction=rotate_direction,
    )
    publish_order(order)


def send_move_map_frame(
    map_x: float,
    map_y: float,
    reverse: bool = False,
    rotate_direction: str = ROTATE_SELF,
    end_theta_override_deg: Optional[float] = None,
):
    """Map frame: absolute target (map_x, map_y)."""
    start = _get_latest_pose(timeout_s=15.0)

    end_theta = _compute_end_theta(start, map_x, map_y, reverse, end_theta_override_deg)

    end = Pose(
        x=map_x,
        y=map_y,
        theta=end_theta,
        map_id=start.map_id,
        map_description=start.map_description,
    )

    dx = end.x - start.x
    dy = end.y - start.y
    vector_theta = math.atan2(dy, dx) if (dx or dy) else start.theta
    edge_orientation = math.pi if reverse else 0.0

    order = _build_order_from_poses(
        start,
        end,
        edge_orientation=edge_orientation,
        end_theta=end_theta,
        start_rotate_direction=rotate_direction,
    )
    publish_order(order)


def manualMove_up(distance: float, reverse: bool = False, rotate_direction: str = ROTATE_SELF, end_theta_override_deg: Optional[float] = None):
    send_move_robot_frame(x_right=0.0, y_forward=distance, reverse=reverse, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)


def manualMove_down(distance: float, reverse: bool = False, rotate_direction: str = ROTATE_SELF, end_theta_override_deg: Optional[float] = None):
    send_move_robot_frame(x_right=0.0, y_forward=-distance, reverse=reverse, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)


def manualMove_left(distance: float, reverse: bool = False, rotate_direction: str = ROTATE_SELF, end_theta_override_deg: Optional[float] = None):
    send_move_robot_frame(x_right=-distance, y_forward=0.0, reverse=reverse, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)


def manualMove_right(distance: float, reverse: bool = False, rotate_direction: str = ROTATE_SELF, end_theta_override_deg: Optional[float] = None):
    send_move_robot_frame(x_right=distance, y_forward=0.0, reverse=reverse, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)


def manualMove_angle(distance: float, angle_deg: float, reverse: bool = False, rotate_direction: str = ROTATE_SELF, end_theta_override_deg: Optional[float] = None):
    # angle: 0=fwd, +right (same UI)
    angle_rad = math.radians(angle_deg)
    x_right = distance * math.sin(angle_rad)
    y_forward = distance * math.cos(angle_rad)
    send_move_robot_frame(x_right=x_right, y_forward=y_forward, reverse=reverse, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)

manualMove_front = manualMove_up
manualMove_back = manualMove_down




def send_rotate_in_place(delta_theta_rad: float, rotate_direction: str):
    """Rotate robot in place by delta_theta_rad (radians)."""
    start = _get_latest_pose(timeout_s=15.0)
    end_theta = _normalize_angle_rad(start.theta + delta_theta_rad)

    end = Pose(
        x=start.x,
        y=start.y,
        theta=end_theta,
        map_id=start.map_id,
        map_description=start.map_description,
    )

    order = _build_order_from_poses(
        start,
        end,
        edge_orientation=end_theta,
        end_theta=end_theta,
        start_rotate_direction=ROTATE_SELF,
    )
    order["nodes"][1]["rotateDirection"] = _normalize_rotate_direction(rotate_direction)
    order["nodes"][1]["nodePosition"]["allowedDeviationTheta"] = ALLOWED_DEVIATION_THETA
    order["edges"][0]["orientation"] = end_theta

    publish_order(order)


def manualRotate_clockwise(angle_deg: float):
    angle_rad = math.radians(angle_deg)
    send_rotate_in_place(delta_theta_rad=-abs(angle_rad), rotate_direction=ROTATE_CLOCKWISE)


def manualRotate_counterclockwise(angle_deg: float):
    angle_rad = math.radians(angle_deg)
    send_rotate_in_place(delta_theta_rad=abs(angle_rad), rotate_direction=ROTATE_COUNTERCLOCKWISE)

def _print_manual_help():
    print("Manual tool commands:")
    print("  w/a/s/d                 move step (robot frame)")
    print("  m <dist> <angle>        move by distance & angle (deg, 0=fwd, +right)")
    print("  goto <mapX> <mapY>      move to absolute map coordinate")
    print("  step <meters>           set default step distance")
    print("  reverse on/off          tail-first mode")
    print("  rotate <self|cw|ccw>    set rotateDirection on start node")
    print("  cw <deg>                rotate in place clockwise")
    print("  ccw <deg>               rotate in place counterclockwise")
    print("  theta <deg|auto>        override end theta (deg, 0=forward)")
    print("  pose                    show latest pose")
    print("  status                  show current config")
    print("  q                       quit")


def _manual_loop():
    step = STEP
    reverse_mode = False
    rotate_direction = ROTATE_SELF
    end_theta_override_deg: Optional[float] = None
    print("Manual tool ready. Type 'help' for commands.")
    try:
        pose = _get_latest_pose(timeout_s=5.0)
        print(_format_pose(pose))
    except Exception as exc:
        print("Pose not available:", exc)

    while True:
        try:
            line = input("manual> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue

        parts = shlex.split(line)
        cmd = parts[0].lower()

        if cmd in ("q", "quit", "exit"):
            break
        if cmd in ("help", "h", "?"):
            _print_manual_help()
            continue
        if cmd in ("status", "st"):
            mode = "tail-first" if reverse_mode else "head-first"
            theta_text = "auto" if end_theta_override_deg is None else f"{end_theta_override_deg:.2f} deg"
            print(f"step={step:.3f}m mode={mode} rotate={rotate_direction} theta={theta_text}")
            continue
        if cmd in ("pose", "p"):
            try:
                pose = _get_latest_pose(timeout_s=2.0)
            except Exception:
                pose = _get_cached_pose()
            print(_format_pose(pose))
            continue
        if cmd == "step":
            if len(parts) == 1:
                print(f"step={step:.3f}m")
                continue
            try:
                value = float(parts[1])
            except ValueError:
                print("Invalid step value")
                continue
            if value <= 0:
                print("Step must be > 0")
                continue
            step = value
            print(f"step={step:.3f}m")
            continue
        if cmd in ("reverse", "tail"):
            if len(parts) == 1:
                reverse_mode = not reverse_mode
            else:
                value = parts[1].lower()
                if value in ("on", "true", "1"):
                    reverse_mode = True
                elif value in ("off", "false", "0"):
                    reverse_mode = False
                else:
                    print("Usage: reverse on/off")
                    continue
            mode = "tail-first" if reverse_mode else "head-first"
            print(f"mode={mode}")
            continue
        if cmd in ("rotate", "rot"):
            if len(parts) == 1:
                print(f"rotate={rotate_direction}")
                continue
            try:
                rotate_direction = _normalize_rotate_direction(parts[1])
            except ValueError as exc:
                print(exc)
                continue
            print(f"rotate={rotate_direction}")
            continue
        if cmd == "theta":
            if len(parts) == 1:
                if end_theta_override_deg is None:
                    print("theta=auto")
                else:
                    print(f"theta={end_theta_override_deg:.2f} deg")
                continue
            if parts[1].lower() in ("auto", "off", "none"):
                end_theta_override_deg = None
                print("theta=auto")
                continue
            try:
                end_theta_override_deg = float(parts[1])
            except ValueError:
                print("Invalid theta value")
                continue
            print(f"theta={end_theta_override_deg:.2f} deg")
            continue

        try:
            if cmd in ("w", "up", "forward"):
                manualMove_up(step, reverse=reverse_mode, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)
            elif cmd in ("s", "down", "back"):
                manualMove_down(step, reverse=reverse_mode, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)
            elif cmd in ("a", "left"):
                manualMove_left(step, reverse=reverse_mode, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)
            elif cmd in ("d", "right"):
                manualMove_right(step, reverse=reverse_mode, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)
            elif cmd in ("m", "move"):
                if len(parts) >= 3:
                    dist = float(parts[1])
                    angle_deg = float(parts[2])
                else:
                    dist = float(input(f"Distance (m) [{step}]: ") or step)
                    angle_deg = float(input("Angle deg (0=fwd, +right): ") or 0.0)
                manualMove_angle(dist, angle_deg, reverse=reverse_mode, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)
            elif cmd in ("goto", "g"):
                if len(parts) < 3:
                    print("Usage: goto <mapX> <mapY>")
                    continue
                map_x = float(parts[1])
                map_y = float(parts[2])
                send_move_map_frame(map_x, map_y, reverse=reverse_mode, rotate_direction=rotate_direction, end_theta_override_deg=end_theta_override_deg)
            else:
                print("Unknown command. Type 'help'.")
        except Exception as exc:
            print("Command failed:", exc)


def _pose_payload(pose: Pose) -> dict:
    return {
        "x": pose.x,
        "y": pose.y,
        "theta": pose.theta,
        "thetaDeg": math.degrees(pose.theta),
        "mapId": pose.map_id,
    }


class ManualWebHandler(BaseHTTPRequestHandler):
    """Very small web UI API + static file server for grid moves."""

    server_version = "ManualWeb/1.1"

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _parse_bool(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return False

    def _send_file(self, file_path: str) -> None:
        ext = os.path.splitext(file_path)[1].lower()
        content_type = CONTENT_TYPES.get(ext, "application/octet-stream")
        with open(file_path, "rb") as handle:
            data = handle.read()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        path = urlparse(self.path).path

        if path == "/api/state":
            pose = _get_cached_pose()
            if pose is None:
                try:
                    pose = _get_latest_pose(timeout_s=0.5)
                except Exception:
                    pose = None
            if pose is None:
                self._send_json(200, {"ok": False, "pose": None})
                return
            self._send_json(200, {"ok": True, "pose": _pose_payload(pose)})
            return

        if path == "/":
            path = "/index.html"

        file_path = os.path.normpath(os.path.join(WEB_DIR, path.lstrip("/")))
        if not file_path.startswith(WEB_DIR):
            self.send_error(403)
            return
        if not os.path.isfile(file_path):
            self.send_error(404)
            return
        self._send_file(file_path)

    def do_POST(self) -> None:
        path = urlparse(self.path).path

        if path == "/api/move":
            length = int(self.headers.get("Content-Length", "0"))
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
            except Exception:
                self._send_json(400, {"ok": False, "error": "Invalid JSON"})
                return

            mode = (payload.get("mode") or MODE_ROBOT).strip().lower()
            if mode not in MODE_CHOICES:
                self._send_json(400, {"ok": False, "error": "Invalid mode (robot|map)"})
                return

            reverse = self._parse_bool(payload.get("reverse"))
            rotate_direction = payload.get("rotateDirection") or ROTATE_SELF
            theta_override = payload.get("thetaOverrideDeg")
            if theta_override is not None:
                try:
                    theta_override = float(theta_override)
                except (TypeError, ValueError):
                    self._send_json(400, {"ok": False, "error": "Invalid thetaOverrideDeg"})
                    return
            try:
                rotate_direction = _normalize_rotate_direction(str(rotate_direction))
            except ValueError as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
                return

            with SEND_LOCK:
                reset_order_session()
                try:
                    pose = _get_latest_pose(timeout_s=5.0)
                except Exception as exc:
                    self._send_json(503, {"ok": False, "error": str(exc)})
                    return

                try:
                    if mode == MODE_MAP:
                        map_x = float(payload.get("mapX"))
                        map_y = float(payload.get("mapY"))
                        start = pose
                        end_theta = _compute_end_theta(start, map_x, map_y, reverse, theta_override)
                        end = Pose(
                            x=map_x,
                            y=map_y,
                            theta=end_theta,
                            map_id=start.map_id,
                            map_description=start.map_description,
                        )
                        edge_orientation = math.pi if reverse else 0.0

                        order = _build_order_from_poses(
                            start,
                            end,
                            edge_orientation,
                            end_theta,
                            start_rotate_direction=rotate_direction,
                        )
                        publish_order(order)
                    else:
                        x = float(payload.get("x"))
                        y = float(payload.get("y"))
                        # robot frame: x=right, y=forward
                        start = pose
                        forward = y
                        left = -x
                        end0 = _compute_target_pose_robot(start, forward=forward, left=left)
                        end_theta = _compute_end_theta(start, end0.x, end0.y, reverse, theta_override)
                        end = Pose(
                            x=end0.x,
                            y=end0.y,
                            theta=end_theta,
                            map_id=start.map_id,
                            map_description=start.map_description,
                        )
                        edge_orientation = math.pi if reverse else 0.0

                        order = _build_order_from_poses(
                            start,
                            end,
                            edge_orientation,
                            end_theta,
                            start_rotate_direction=rotate_direction,
                        )
                        publish_order(order)

                except (TypeError, ValueError):
                    self._send_json(400, {"ok": False, "error": "Invalid numeric payload"})
                    return

            self._send_json(
                200,
                {
                    "ok": True,
                    "orderId": order["orderId"],
                    "orderUpdateId": order["orderUpdateId"],
                    "mode": mode,
                    "targetMap": {"x": end.x, "y": end.y, "theta": end_theta},
                },
            )
            return

        if path == "/api/cancel":
            with SEND_LOCK:
                client = _ensure_client_connected()
                message = {
                    "headerId": int(time.time() * 1000),
                    "timestamp": _utc_timestamp(),
                    "version": "2.0.0",
                    "manufacturer": MANUFACTURER,
                    "serialNumber": ROBOT_ID,
                    "actions": [
                        {
                            "actionType": "cancelOrder",
                            "actionId": str(uuid.uuid4()),
                            "actionDescription": "cancel order",
                            "blockingType": "NONE",
                        }
                    ],
                }
                payload = json.dumps(message, ensure_ascii=False)
                info = client.publish(
                    MQTT_TOPIC_INSTANTACTIONS,
                    payload=payload,
                    qos=MQTT_QOS,
                    retain=MQTT_RETAIN,
                )
                info.wait_for_publish()

            self._send_json(200, {"ok": True})
            return

        self.send_error(404)


def _run_web_ui(host: str, port: int) -> None:
    if not os.path.isdir(WEB_DIR):
        raise RuntimeError(f"Missing web UI directory: {WEB_DIR}")

    reset_order_session()
    _ensure_client_connected()

    server = ThreadingHTTPServer((host, port), ManualWebHandler)
    print(f"Web UI running at http://{host}:{port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        disconnect()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual move tool for VDA5050 robot")
    parser.add_argument("--manual", action="store_true", help="Run interactive manual tool")
    parser.add_argument("--web", action="store_true", help="Run web UI server")
    parser.add_argument("--host", default="127.0.0.1", help="Web UI bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Web UI bind port (default: 8000)")

    # Single move options
    parser.add_argument("--mode", choices=MODE_CHOICES, default=MODE_ROBOT, help="Move mode: robot|map")
    parser.add_argument("--distance", type=float, help="Distance in meters for a single move (robot mode)")
    parser.add_argument(
        "--direction",
        choices=["up", "down", "left", "right"],
        help="Direction for a single move (robot mode)",
    )
    parser.add_argument(
        "--angle",
        type=float,
        help="Direction angle in degrees (0 forward, +right). Overrides --direction (robot mode)",
    )
    parser.add_argument("--map-x", type=float, help="Absolute map X (map mode)")
    parser.add_argument("--map-y", type=float, help="Absolute map Y (map mode)")

    parser.add_argument(
        "--theta",
        type=float,
        help="Override end theta in robot frame (deg, 0=forward).",
    )
    parser.add_argument(
        "--rotate-direction",
        type=_normalize_rotate_direction,
        choices=ROTATE_DIRECTION_CHOICES,
        default=ROTATE_SELF,
        help="rotateDirection for the start node (self/cw/ccw).",
    )
    parser.add_argument(
        "--reverse",
        action="store_true",
        help="Move with tail first (theta reversed, edge pi)",
    )
    return parser.parse_args()


def _run_single_move(args: argparse.Namespace):
    if args.mode == MODE_MAP:
        if args.map_x is None or args.map_y is None:
            raise RuntimeError("Map mode needs --map-x and --map-y")
        send_move_map_frame(
            args.map_x,
            args.map_y,
            reverse=args.reverse,
            rotate_direction=args.rotate_direction,
            end_theta_override_deg=args.theta,
        )
        return

    # robot mode:
    if args.distance is None or (args.direction is None and args.angle is None):
        raise RuntimeError("Robot mode needs --distance and --direction or --angle")

    if args.angle is not None:
        manualMove_angle(
            args.distance,
            args.angle,
            reverse=args.reverse,
            rotate_direction=args.rotate_direction,
            end_theta_override_deg=args.theta,
        )
        return

    if args.direction == "up":
        manualMove_up(args.distance, reverse=args.reverse, rotate_direction=args.rotate_direction, end_theta_override_deg=args.theta)
    elif args.direction == "down":
        manualMove_down(args.distance, reverse=args.reverse, rotate_direction=args.rotate_direction, end_theta_override_deg=args.theta)
    elif args.direction == "left":
        manualMove_left(args.distance, reverse=args.reverse, rotate_direction=args.rotate_direction, end_theta_override_deg=args.theta)
    elif args.direction == "right":
        manualMove_right(args.distance, reverse=args.reverse, rotate_direction=args.rotate_direction, end_theta_override_deg=args.theta)
    else:
        raise RuntimeError("Invalid direction")


def main():
    args = _parse_args()

    if args.web:
        _run_web_ui(args.host, args.port)
        return

    use_manual = args.manual or (
        args.distance is None
        and args.direction is None
        and args.angle is None
        and args.map_x is None
        and args.map_y is None
    )

    reset_order_session()
    _ensure_client_connected()
    try:
        if use_manual:
            _manual_loop()
        else:
            _run_single_move(args)
        time.sleep(0.5)
    finally:
        disconnect()


if __name__ == "__main__":
    main()