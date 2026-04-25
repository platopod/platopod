"""Tests for plato_pod.api_gateway_server — REST and WebSocket endpoints."""

from __future__ import annotations

import json

import pytest
from fastapi.testclient import TestClient

from plato_pod.api_gateway_server import (
    ArenaStateStore,
    ConnectionManager,
    RobotStatusStore,
    create_gateway_app,
)
from plato_pod.control_manager import ControlManager


def _make_app(
    robots: list[dict] | None = None,
    boundary: tuple[tuple[float, float], ...] = (),
    spawn_result: dict | None = None,
    remove_result: dict | None = None,
    reset_result: dict | None = None,
):
    """Create a test gateway app with mock callbacks."""
    robot_store = RobotStatusStore()
    if robots:
        robot_store.update(robots)

    arena_store = ArenaStateStore()
    if boundary:
        arena_store.update(boundary, [], {"boundary": list(boundary)})

    control_mgr = ControlManager()
    conn_mgr = ConnectionManager()

    cmd_vel_calls = []

    def on_cmd_vel(robot_id, linear_x, angular_z):
        cmd_vel_calls.append((robot_id, linear_x, angular_z))

    def on_spawn(x, y, theta, radius):
        return spawn_result or {"success": True, "robot_id": 99, "message": "OK"}

    def on_remove(robot_id):
        return remove_result or {"success": True, "message": "OK"}

    def on_reset(robot_id):
        return reset_result or {"success": True, "message": "OK"}

    def on_list_robots():
        return robot_store.get_all()

    app = create_gateway_app(
        robot_store, arena_store, control_mgr, conn_mgr,
        on_cmd_vel, on_spawn, on_remove, on_reset, on_list_robots,
    )
    return app, robot_store, control_mgr, cmd_vel_calls


# --- REST endpoints ---

class TestListRobots:
    def test_empty_list(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)
        resp = client.get("/robots")
        assert resp.status_code == 200
        assert resp.json() == []

    def test_with_robots(self) -> None:
        robots = [
            {"robot_id": 1, "type": "physical", "x": 0.5, "y": 0.5,
             "theta": 0.0, "radius": 0.028, "status": "active",
             "localization_id": "3", "localization_source": "camera_artag"},
        ]
        app, *_ = _make_app(robots=robots)
        client = TestClient(app)
        resp = client.get("/robots")
        assert resp.status_code == 200
        data = resp.json()
        assert len(data) == 1
        assert data[0]["robot_id"] == 1


class TestSpawnRobot:
    def test_spawn_success(self) -> None:
        app, *_ = _make_app(spawn_result={"success": True, "robot_id": 1, "message": "OK"})
        client = TestClient(app)
        resp = client.post("/robots/spawn", json={"x": 0.5, "y": 0.5, "theta": 0.0})
        assert resp.status_code == 200
        assert resp.json()["success"] is True

    def test_spawn_failure(self) -> None:
        app, *_ = _make_app(
            spawn_result={"success": False, "robot_id": -1, "message": "out_of_bounds"}
        )
        client = TestClient(app)
        resp = client.post("/robots/spawn", json={"x": 5.0, "y": 5.0})
        assert resp.status_code == 400


class TestRemoveRobot:
    def test_remove_success(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)
        resp = client.delete("/robots/1")
        assert resp.status_code == 200

    def test_remove_failure(self) -> None:
        app, *_ = _make_app(remove_result={"success": False, "message": "Cannot remove physical"})
        client = TestClient(app)
        resp = client.delete("/robots/1")
        assert resp.status_code == 400


class TestResetRobot:
    def test_reset_success(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)
        resp = client.post("/robots/1/reset")
        assert resp.status_code == 200


class TestArenaModel:
    def test_no_model(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)
        resp = client.get("/arena/model")
        assert resp.status_code == 503

    def test_with_model(self) -> None:
        boundary = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
        app, *_ = _make_app(boundary=boundary)
        client = TestClient(app)
        resp = client.get("/arena/model")
        assert resp.status_code == 200


class TestHealth:
    def test_health(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)
        resp = client.get("/health")
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "ok"
        assert data["robots_count"] == 0


# --- WebSocket ---

class TestWebSocketCmdVel:
    def test_cmd_vel_valid_robot(self) -> None:
        robots = [
            {"robot_id": 1, "type": "virtual", "x": 0.5, "y": 0.5,
             "theta": 0.0, "radius": 0.028, "status": "active", "localization_id": "", "localization_source": "virtual_sim"},
        ]
        boundary = ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0))
        app, _, _, cmd_vel_calls = _make_app(robots=robots, boundary=boundary)
        client = TestClient(app)

        with client.websocket_connect("/api/control") as ws:
            ws.send_text(json.dumps({
                "type": "cmd_vel",
                "robot_id": 1,
                "linear_x": 0.1,
                "angular_z": 0.0,
            }))
            # The server processes the command — no explicit response for cmd_vel
            # but cmd_vel callback should have been called
        assert len(cmd_vel_calls) >= 1
        assert cmd_vel_calls[0][0] == 1  # robot_id

    def test_cmd_vel_not_found(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)

        with client.websocket_connect("/api/control") as ws:
            ws.send_text(json.dumps({
                "type": "cmd_vel",
                "robot_id": 99,
                "linear_x": 0.1,
                "angular_z": 0.0,
            }))
            resp = json.loads(ws.receive_text())
            assert resp["type"] == "error"
            assert resp["code"] == "not_found"

    def test_invalid_message(self) -> None:
        app, *_ = _make_app()
        client = TestClient(app)

        with client.websocket_connect("/api/control") as ws:
            ws.send_text("not json")
            resp = json.loads(ws.receive_text())
            assert resp["type"] == "error"
            assert resp["code"] == "invalid_command"


class TestWebSocketSubscribe:
    def test_subscribe_ack(self) -> None:
        robots = [
            {"robot_id": 1, "type": "virtual", "x": 0.5, "y": 0.5,
             "theta": 0.0, "radius": 0.028, "status": "active", "localization_id": "", "localization_source": "virtual_sim"},
        ]
        app, *_ = _make_app(robots=robots)
        client = TestClient(app)

        with client.websocket_connect("/api/control") as ws:
            ws.send_text(json.dumps({
                "type": "subscribe",
                "robot_ids": [1, 99],
            }))
            resp = json.loads(ws.receive_text())
            assert resp["type"] == "subscribe_ack"
            assert 1 in resp["subscribed"]
            assert len(resp["failed"]) == 1
            assert resp["failed"][0]["robot_id"] == 99


# --- Stores ---

class TestRobotStatusStore:
    def test_update_and_get(self) -> None:
        store = RobotStatusStore()
        store.update([{"robot_id": 1, "x": 0.5}])
        assert store.get(1) is not None
        assert store.get(1)["x"] == 0.5

    def test_get_all(self) -> None:
        store = RobotStatusStore()
        store.update([{"robot_id": 1}, {"robot_id": 2}])
        assert len(store.get_all()) == 2

    def test_get_nonexistent(self) -> None:
        store = RobotStatusStore()
        assert store.get(99) is None


class TestArenaStateStore:
    def test_update_and_get(self) -> None:
        store = ArenaStateStore()
        boundary = ((0, 0), (1, 0), (1, 1))
        store.update(boundary, [], {"test": True})
        assert store.get_boundary() == boundary
        assert store.get_model_dict() == {"test": True}
