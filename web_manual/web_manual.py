#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import argparse
import json
import math
import os
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

import mainControl


WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_manual")
MQTT_TOPIC_INSTANTACTIONS = f"VDA/V2.0.0/Robot/{mainControl.ROBOT_ID}/instantActions"

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".svg": "image/svg+xml",
}

SEND_LOCK = threading.Lock()

MODE_ROBOT = "robot"
MODE_MAP = "map"
MODE_CHOICES = (MODE_ROBOT, MODE_MAP)


def _pose_payload(pose: mainControl.Pose) -> dict:
    return {
        "x": pose.x,
        "y": pose.y,
        "theta": pose.theta,
        "thetaDeg": math.degrees(pose.theta),
        "mapId": pose.map_id,
    }


class ManualHandler(BaseHTTPRequestHandler):
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
            pose = mainControl._get_cached_pose()
            if pose is None:
                try:
                    pose = mainControl._get_latest_pose(timeout_s=0.5)
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
            rotate_direction = payload.get("rotateDirection") or mainControl.ROTATE_SELF

            theta_override = payload.get("thetaOverrideDeg")
            if theta_override is not None:
                try:
                    theta_override = float(theta_override)
                except (TypeError, ValueError):
                    self._send_json(400, {"ok": False, "error": "Invalid thetaOverrideDeg"})
                    return

            try:
                rotate_direction = mainControl._normalize_rotate_direction(str(rotate_direction))
            except ValueError as exc:
                self._send_json(400, {"ok": False, "error": str(exc)})
                return

            # Lấy pose
            try:
                pose = mainControl._get_latest_pose(timeout_s=5.0)
            except Exception as exc:
                self._send_json(503, {"ok": False, "error": str(exc)})
                return

            with SEND_LOCK:
                mainControl.reset_order_session()

                try:
                    if mode == MODE_MAP:
                        # absolute map target
                        map_x = float(payload.get("mapX"))
                        map_y = float(payload.get("mapY"))

                        mainControl.send_move_map_frame(
                            map_x=map_x,
                            map_y=map_y,
                            reverse=reverse,
                            rotate_direction=rotate_direction,
                            end_theta_override_deg=theta_override,
                        )

                        # phản hồi target cho UI (ước tính theo đầu vào)
                        target = {"x": map_x, "y": map_y}

                    else:
                        # robot frame vector
                        x = float(payload.get("x"))
                        y = float(payload.get("y"))

                        mainControl.send_move_robot_frame(
                            x_right=x,
                            y_forward=y,
                            reverse=reverse,
                            rotate_direction=rotate_direction,
                            end_theta_override_deg=theta_override,
                        )

                        # target theo kiểu "vector" (UI đang dùng)
                        target = {"x": x, "y": y}

                except (TypeError, ValueError):
                    self._send_json(400, {"ok": False, "error": "Invalid numeric payload"})
                    return
                except Exception as exc:
                    self._send_json(500, {"ok": False, "error": str(exc)})
                    return

            self._send_json(
                200,
                {
                    "ok": True,
                    "mode": mode,
                    "orderId": "see MQTT",  # orderId nằm trong payload MQTT; nếu muốn trả về orderId thật thì cần mainControl trả lại order dict
                    "target": target,
                    "pose": _pose_payload(pose),
                },
            )
            return

        if path == "/api/cancel":
            with SEND_LOCK:
                client = mainControl._ensure_client_connected()
                message = {
                    "headerId": int(time.time() * 1000),
                    "timestamp": mainControl._utc_timestamp(),
                    "version": "2.0.0",
                    "manufacturer": mainControl.MANUFACTURER,
                    "serialNumber": mainControl.ROBOT_ID,
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
                    qos=mainControl.MQTT_QOS,
                    retain=mainControl.MQTT_RETAIN,
                )
                info.wait_for_publish()

            self._send_json(200, {"ok": True})
            return

        self.send_error(404)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Web UI for manual moves (robot frame + map frame)")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8000, help="Bind port (default: 8000)")
    return parser.parse_args()


def main() -> None:
    if not os.path.isdir(WEB_DIR):
        raise RuntimeError(f"Missing web UI directory: {WEB_DIR}")

    args = _parse_args()

    mainControl.reset_order_session()
    mainControl._ensure_client_connected()

    server = ThreadingHTTPServer((args.host, args.port), ManualHandler)
    print(f"Web UI running at http://{args.host}:{args.port}/")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        mainControl.disconnect()


if __name__ == "__main__":
    main()
