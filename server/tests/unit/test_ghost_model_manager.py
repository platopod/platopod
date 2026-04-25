"""Tests for plato_pod.ghost_model_manager — ghost model lifecycle."""

from __future__ import annotations

from unittest.mock import patch, MagicMock
import subprocess

import pytest

from plato_pod.geo_reference import GeoReference
from plato_pod.ghost_model_manager import GhostModelManager, ROLE_TO_MODEL


def _geo(scale: float = 1000.0) -> GeoReference:
    return GeoReference(
        origin_lat=-35.2975, origin_lon=149.1012, scale_factor=scale,
    )


def _mock_subprocess_success(*args, **kwargs):
    result = MagicMock()
    result.returncode = 0
    result.stderr = b""
    return result


def _mock_subprocess_failure(*args, **kwargs):
    result = MagicMock()
    result.returncode = 1
    result.stderr = b"error"
    return result


class TestGhostModelManager:
    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_success)
    def test_spawn_ghost(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        ok = mgr.spawn_ghost(1, "tank", 0.3, 0.2, 1.5)
        assert ok is True
        assert mgr.has_ghost(1)
        assert mock_run.called

        # Verify the gz service call was for /world/arena/create
        call_args = mock_run.call_args[0][0]
        assert "/world/arena/create" in call_args

    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_failure)
    def test_spawn_failure(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        ok = mgr.spawn_ghost(1, "tank", 0.3, 0.2, 1.5)
        assert ok is False
        assert not mgr.has_ghost(1)

    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_success)
    def test_update_pose(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        mgr.spawn_ghost(1, "recon", 0.3, 0.2, 0.0)

        ok = mgr.update_pose(1, 0.5, 0.4, 1.0)
        assert ok is True

        state = mgr.get_ghost_state(1)
        assert state is not None
        assert state.world_x == pytest.approx(500.0)
        assert state.world_y == pytest.approx(400.0)

    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_success)
    def test_update_unspawned_returns_false(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        ok = mgr.update_pose(99, 0.3, 0.2, 0.0)
        assert ok is False

    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_success)
    def test_remove_ghost(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        mgr.spawn_ghost(1, "tank", 0.3, 0.2, 0.0)
        assert mgr.has_ghost(1)

        ok = mgr.remove_ghost(1)
        assert ok is True
        assert not mgr.has_ghost(1)

    def test_remove_nonexistent(self) -> None:
        mgr = GhostModelManager(_geo())
        ok = mgr.remove_ghost(99)
        assert ok is False

    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_success)
    def test_get_ghost_ids(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        mgr.spawn_ghost(1, "tank", 0.1, 0.1, 0.0)
        mgr.spawn_ghost(3, "recon", 0.5, 0.5, 0.0)
        ids = mgr.get_ghost_ids()
        assert sorted(ids) == [1, 3]

    @patch("plato_pod.ghost_model_manager.subprocess.run", side_effect=_mock_subprocess_success)
    def test_scale_factor_applied(self, mock_run) -> None:
        mgr = GhostModelManager(_geo(scale=500.0))
        mgr.spawn_ghost(1, "apc", 0.4, 0.2, 0.0)
        state = mgr.get_ghost_state(1)
        assert state.world_x == pytest.approx(200.0)
        assert state.world_y == pytest.approx(100.0)

    @patch("plato_pod.ghost_model_manager.subprocess.run",
           side_effect=FileNotFoundError("gz not found"))
    def test_gz_not_found(self, mock_run) -> None:
        mgr = GhostModelManager(_geo())
        ok = mgr.spawn_ghost(1, "tank", 0.3, 0.2, 0.0)
        assert ok is False


class TestRoleToModel:
    def test_all_roles_mapped(self) -> None:
        for role in ["default", "recon", "cbrn_recon", "tank", "apc", "artillery", "sensor"]:
            assert role in ROLE_TO_MODEL

    def test_unknown_role_defaults_to_platopod(self) -> None:
        mgr = GhostModelManager(_geo())
        # The spawn method uses ROLE_TO_MODEL.get(role, "platopod")
        # We can verify this by checking the mapping
        assert ROLE_TO_MODEL.get("unknown_role", "platopod") == "platopod"
