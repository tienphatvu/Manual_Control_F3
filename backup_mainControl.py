#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import math
import time
import uuid
from dataclasses import dataclass
from threading import Event, Lock
from typing import Any, Dict, Optional

import paho.mqtt.client as mqtt


MQTT_BROKER = "172.16.0.103"
MQTT_PORT = 9001

USE_WEBSOCKETS = True
WS_PATH: Optional[str] = "/mqtt"

MQTT_USERNAME: Optional[str] = None
MQTT_PASSWORD: Optional[str] = None

MQTT_TOPIC_ORDER = "VDA/V2.0.0/Robot/9190/order"
MQTT_TOPIC_STATE = "VDA/V2.0.0/Robot/9190/state"

MQTT_QOS = 0
MQTT_RETAIN = False

MAX_SPEED = 0.5

DEFAULT_MAP_DESCRIPTION = "default"
DEFAULT_MAP_ID = "XX"

ALLOWED_DEVIATION_XY = 0.05
ALLOWED_DEVIATION_THETA = math.pi
ROTATE_ALLOWED_DEVIATION_THETA = 0.0

ROTATE_SELF = "ROTATE_SELF"
ROTATE_CLOCKWISE = "ROTATE_CLOCKWISE"
ROTATE_COUNTERCLOCKWISE = "ROTATE_COUNTERCLOCKWISE"


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


def _on_connect(client: mqtt.Client, userdata: Any, flags: Dict[str, Any], reason_code: Any, properties: Any):
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


def _ensure_client_connected() -> mqtt.Client:
    global _client
    if _client is not None:
        return _client

    if USE_WEBSOCKETS:
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            transport="websockets",
        )
        if WS_PATH:
            client.ws_set_options(path=WS_PATH)
    else:
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)

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


def _normalize_angle_rad(theta: float) -> float:
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


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
    end_theta_override: Optional[float] = None,
) -> Dict[str, Any]:
    global _current_update_id

    start_node_id = str(uuid.uuid4())
    end_node_id = str(uuid.uuid4())
    edge_id = str(uuid.uuid4())

    dx = end.x - start.x
    dy = end.y - start.y
    if end_theta_override is not None:
        end_theta = end_theta_override
    else:
        end_theta = math.atan2(dy, dx) if (dx != 0.0 or dy != 0.0) else end.theta

    cp1 = {"weight": 1.0, "x": start.x, "y": start.y}
    cp4 = {"weight": 1.0, "x": end.x, "y": end.y}
    cp2 = {"weight": 1.0, "x": start.x + dx / 3.0, "y": start.y + dy / 3.0}
    cp3 = {"weight": 1.0, "x": start.x + 2.0 * dx / 3.0, "y": start.y + 2.0 * dy / 3.0}

    order = {
        "headerId": 3,
        "manufacturer": "HikRobot",
        "version": "2.0.0",
        "serialNumber": "9190",
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
                "rotateDirection": ROTATE_SELF,
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


def _send_robot_distance(forward: float, left: float):
    start = _get_latest_pose(timeout_s=15.0)
    end = _compute_target_pose_robot(start, forward=forward, left=left)
    edge_orientation = math.pi if forward < 0.0 else 0.0
    end_theta_override = start.theta if (forward < 0.0 and left == 0.0) else None
    print("POSE", start.x, start.y, start.theta)
    print("TARGET", end.x, end.y, end.theta)
    publish_order(_build_order_from_poses(start, end, edge_orientation, end_theta_override))


def _send_robot_rotate(delta_theta: float, rotate_direction: str):
    start = _get_latest_pose(timeout_s=15.0)

    end = Pose(
        x=start.x,
        y=start.y,
        theta=_normalize_angle_rad(start.theta + delta_theta),
        map_id=start.map_id,
        map_description=start.map_description,
    )

    order = _build_order_from_poses(start, end, end.theta, end.theta)
    order["nodes"][1]["rotateDirection"] = rotate_direction
    order["nodes"][1]["nodePosition"]["allowedDeviationTheta"] = ROTATE_ALLOWED_DEVIATION_THETA
    order["edges"][0]["orientation"] = end.theta

    publish_order(order)


def manualMove_front(distance: float):
    reset_order_session()
    _send_robot_distance(forward=distance, left=0.0)

def manualMove_back(distance: float):
    reset_order_session()
    _send_robot_distance(forward=-distance, left=0.0)

def manualMove_left(distance: float):
    reset_order_session()
    _send_robot_distance(forward=0.0, left=distance)

def manualMove_right(distance: float):
    reset_order_session()
    _send_robot_distance(forward=0.0, left=-distance)


def manualRotate_clockwise(angle: float):
    reset_order_session()
    if angle >= 4.0:
        angle_rad = angle * math.pi / 180.0
        print(f"Angle_rad: ", angle_rad)
        _send_robot_rotate(delta_theta=-abs(angle_rad), rotate_direction=ROTATE_CLOCKWISE)
    else :
        print("Angle must be greater than or equal to 4 degrees for clockwise rotation.")
def manualRotate_counterclockwise(angle: float):
    reset_order_session()
    if angle >= 4.0:    
        angle_rad = angle * math.pi / 180.0
        print(f"Angle_rad: ", angle_rad)
        _send_robot_rotate(delta_theta=abs(angle_rad), rotate_direction=ROTATE_COUNTERCLOCKWISE)
    else: 
        print("Angle must be greater than or equal to 4 degrees for counterclockwise rotation.")

if __name__ == "__main__":
    try:
        reset_order_session()
        _ensure_client_connected()

        pose = _get_latest_pose(timeout_s=15.0)
        print(
            "CURRENT_POSE",
            f"x={pose.x:.3f}",
            f"y={pose.y:.3f}",
            f"theta={pose.theta:.3f}",
        )

        val = float(input("Input value (m for move / degree for rotate): ").strip())
        cmd = input("Input command (front/back/left/right/cw/ccw): ").strip().lower()

        if cmd == "front":
            manualMove_front(val)
        elif cmd == "back":
            manualMove_back(val)
        elif cmd == "left":
            manualMove_left(val)
        elif cmd == "right":
            manualMove_right(val)
        elif cmd == "cw":
            manualRotate_clockwise(val)
        elif cmd == "ccw":
            manualRotate_counterclockwise(val)
        else:
            print("Command not valid")

        time.sleep(0.5)
    finally:
        disconnect()
