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

ROBOT_ID = "9190"
MANUFACTURER = "hikrobotics"
VERSION = "2.0.0"

MQTT_TOPIC_ORDER = f"VDA/V2.0.0/Robot/{ROBOT_ID}/order"
MQTT_TOPIC_STATE = f"VDA/V2.0.0/Robot/{ROBOT_ID}/state"
MQTT_TOPIC_INSTANTACTIONS = f"VDA/V2.0.0/Robot/{ROBOT_ID}/instantActions"

MQTT_QOS = 0
MQTT_RETAIN = False

DEFAULT_MAP_DESCRIPTION = "default"
DEFAULT_MAP_ID = "XX"

ALLOWED_DEVIATION_XY = 0.7
ALLOWED_DEVIATION_THETA = 6.28

LOG_DIR = "/home/nrc/manual_control/temp/logs"
TEMPLATE_PATH = "/home/nrc/manual_control/temp/template.json"


@dataclass
class Pose:
    x: float
    y: float
    theta: float
    map_id: str
    map_description: str


_pose_lock = Lock()
_latest_state: Optional[Dict[str, Any]] = None
_state_ready = Event()
_client: Optional[mqtt.Client] = None


def _timestamp_utc_plus7() -> str:
    ts = time.time() + 7 * 3600
    return time.strftime("%Y-%m-%dT%H:%M:%S+0700", time.gmtime(ts))


def _normalize_angle_rad(theta: float) -> float:
    while theta > math.pi:
        theta -= 2.0 * math.pi
    while theta < -math.pi:
        theta += 2.0 * math.pi
    return theta


def _enabled_map_id(state: Dict[str, Any]) -> str:
    maps = state.get("maps", [])
    if isinstance(maps, list):
        for m in maps:
            if isinstance(m, dict) and m.get("mapStatus") == "ENABLED" and m.get("mapId"):
                return str(m["mapId"])
    return DEFAULT_MAP_ID


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
        return Pose(
            x=x,
            y=y,
            theta=theta,
            map_id=map_id,
            map_description=DEFAULT_MAP_DESCRIPTION,
        )
    except Exception:
        return None


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
    global _latest_state
    if msg.topic != MQTT_TOPIC_STATE:
        return
    try:
        state = json.loads(msg.payload.decode("utf-8", errors="replace"))
    except Exception:
        return
    with _pose_lock:
        _latest_state = state
        _state_ready.set()


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


def _get_latest_state(timeout_s: float = 10.0) -> Dict[str, Any]:
    _ensure_client_connected()
    if not _state_ready.wait(timeout=timeout_s):
        raise RuntimeError("No robot state received")
    with _pose_lock:
        if _latest_state is None:
            raise RuntimeError("Robot state is None")
        return json.loads(json.dumps(_latest_state))


def _build_order(start: Pose, end: Pose, distance: float) -> Dict[str, Any]:
    order = _load_template()
    orientation = math.pi
    now_ts = _timestamp_utc_plus7()

    nodes = order.get("nodes")
    edges = order.get("edges")
    if not isinstance(nodes, list) or len(nodes) < 2:
        raise RuntimeError("template.json must contain at least 2 nodes")
    if not isinstance(edges, list) or len(edges) < 1:
        raise RuntimeError("template.json must contain at least 1 edge")

    start_node = nodes[0]
    end_node = nodes[1]
    if not isinstance(start_node, dict) or not isinstance(end_node, dict):
        raise RuntimeError("template.json nodes must be objects")

    start_node_id = str(start_node.get("nodeId", "1"))
    end_node_id = str(end_node.get("nodeId", "node3"))

    start_pos = start_node.get("nodePosition", {})
    end_pos = end_node.get("nodePosition", {})
    if not isinstance(start_pos, dict) or not isinstance(end_pos, dict):
        raise RuntimeError("template.json nodePosition must be objects")

    start_pos["mapDescription"] = start.map_description
    start_pos["mapId"] = start.map_id
    start_pos["x"] = start.x
    start_pos["y"] = start.y
    start_pos["theta"] = _normalize_angle_rad(start.theta)

    end_pos["mapDescription"] = start.map_description
    end_pos["mapId"] = start.map_id
    end_pos["x"] = end.x
    end_pos["y"] = end.y
    end_pos["theta"] = _normalize_angle_rad(end.theta)

    for node in (start_node, end_node):
        actions = node.get("actions", [])
        if isinstance(actions, list):
            for action in actions:
                if not isinstance(action, dict):
                    continue
                action_type = str(action.get("actionType", ""))
                if action_type == "detectObject":
                    action["actionId"] = f"detect-{uuid.uuid4().hex[:6]}"
                elif action_type == "pick":
                    action["actionId"] = f"pick-{uuid.uuid4().hex[:6]}"
                else:
                    action["actionId"] = str(uuid.uuid4())

    edge = edges[0]
    if not isinstance(edge, dict):
        raise RuntimeError("template.json edge must be an object")
    edge["startNodeId"] = start_node_id
    edge["endNodeId"] = end_node_id
    edge["orientation"] = orientation

    order["headerId"] = int(time.time() * 1000)
    order["manufacturer"] = MANUFACTURER
    order["serialNumber"] = ROBOT_ID
    order["version"] = VERSION
    order["orderId"] = str(uuid.uuid4())
    order["orderUpdateId"] = 0
    order["timestamp"] = now_ts
    order["timeStamp"] = now_ts

    return order


def _load_template() -> Dict[str, Any]:
    try:
        with open(TEMPLATE_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        raise RuntimeError(f"Failed to load template.json: {exc}")
    if not isinstance(data, dict):
        raise RuntimeError("template.json must be a JSON object")
    return data


def _compute_backward_pose(start: Pose, distance: float) -> Pose:
    dx = -distance * math.cos(start.theta)
    dy = -distance * math.sin(start.theta)
    return Pose(
        x=start.x + dx,
        y=start.y + dy,
        theta=start.theta,
        map_id=start.map_id,
        map_description=start.map_description,
    )


def _publish_order(order: Dict[str, Any]):
    client = _ensure_client_connected()
    payload = json.dumps(order, ensure_ascii=False)
    info = client.publish(MQTT_TOPIC_ORDER, payload=payload, qos=MQTT_QOS, retain=MQTT_RETAIN)
    info.wait_for_publish()


def _send_cancel_order():
    client = _ensure_client_connected()
    message = {
        "headerId": int(time.time() * 1000),
        "timestamp": _timestamp_utc_plus7(),
        "version": VERSION,
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


def _write_log(sent_message: Dict[str, Any], state_snapshot: Dict[str, Any]):
    log_obj = {
        "sent_message": sent_message,
        "robot_state_later": state_snapshot,
    }
    timestamp_safe = _timestamp_utc_plus7().replace(":", "").replace("+", "_")
    order_id = str(sent_message.get("orderId", "order"))
    filename = f"{timestamp_safe}_{order_id}.log"
    log_path = f"{LOG_DIR}/{filename}"
    try:
        import os
        os.makedirs(LOG_DIR, exist_ok=True)
    except Exception:
        pass
    with open(log_path, "w", encoding="utf-8") as f:
        json.dump(log_obj, f, ensure_ascii=False, indent=2)


def main():
    _ensure_client_connected()
    print("Ready. Input distance (m). Press Enter to send. Empty input to quit. 'c' to cancel anytime.")
    while True:
        raw = input("distance> ").strip()
        if raw == "":
            break
        if raw.lower() == "c":
            _send_cancel_order()
            print("Sent cancelOrder.")
            continue
        try:
            distance = float(raw)
        except ValueError:
            print("Invalid distance. Try again.")
            continue

        try:
            state_before = _get_latest_state(timeout_s=10.0)
        except RuntimeError as exc:
            print(f"State error: {exc}")
            continue

        pose = _try_parse_pose_from_state(state_before)
        if pose is None:
            print("Could not parse robot pose from state.")
            continue

        end_pose = _compute_backward_pose(pose, distance)
        order = _build_order(pose, end_pose, distance)
        _publish_order(order)
        print("Sent order. Press Enter to save log, or 'c' to cancel.")
        while True:
            cmd = input("wait> ").strip()
            if cmd == "":
                try:
                    state_after = _get_latest_state(timeout_s=10.0)
                except RuntimeError as exc:
                    print(f"State error after wait: {exc}")
                    break
                _write_log(order, state_after)
                print("Wrote log.")
                break
            if cmd.lower() == "c":
                _send_cancel_order()
                print("Sent cancelOrder.")
                break


if __name__ == "__main__":
    main()
