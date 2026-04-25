"""Tests for plato_pod.control_manager — ownership, watchdog, timeout."""

from __future__ import annotations

import pytest

from plato_pod.control_manager import ControlManager


class TestTryAcquire:
    def test_acquire_uncontrolled(self) -> None:
        cm = ControlManager()
        ok, err = cm.try_acquire("client_a", 1)
        assert ok is True
        assert err == ""

    def test_same_client_reacquires(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        ok, err = cm.try_acquire("client_a", 1)
        assert ok is True

    def test_different_client_busy(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        ok, err = cm.try_acquire("client_b", 1)
        assert ok is False
        assert err == "robot_busy"

    def test_different_robots_independent(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        ok, err = cm.try_acquire("client_b", 2)
        assert ok is True


class TestRecordCommand:
    def test_creates_entry(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.5, 100.0)
        assert cm.get_controller(1) == "client_a"

    def test_updates_existing(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        cm.record_command("client_a", 1, 0.2, 0.5, 101.0)
        assert cm.get_controller(1) == "client_a"


class TestRelease:
    def test_release_own_robot(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        assert cm.release("client_a", 1) is True
        assert cm.get_controller(1) is None

    def test_release_other_client_fails(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        assert cm.release("client_b", 1) is False
        assert cm.get_controller(1) == "client_a"

    def test_release_nonexistent(self) -> None:
        cm = ControlManager()
        assert cm.release("client_a", 99) is False


class TestReleaseAll:
    def test_releases_all_for_client(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        cm.record_command("client_a", 2, 0.1, 0.0, 100.0)
        cm.record_command("client_b", 3, 0.1, 0.0, 100.0)
        released = cm.release_all("client_a")
        assert sorted(released) == [1, 2]
        assert cm.get_controller(1) is None
        assert cm.get_controller(3) == "client_b"

    def test_releases_empty(self) -> None:
        cm = ControlManager()
        assert cm.release_all("nobody") == []


class TestControlledRobots:
    def test_returns_controlled(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        cm.record_command("client_a", 3, 0.1, 0.0, 100.0)
        assert sorted(cm.controlled_robots("client_a")) == [1, 3]

    def test_empty_for_unknown(self) -> None:
        cm = ControlManager()
        assert cm.controlled_robots("nobody") == []


class TestWatchdog:
    def test_fires_once_after_timeout(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.5, 100.0)

        # Before timeout
        wd, cr = cm.tick(100.3, watchdog_timeout=0.5)
        assert len(wd) == 0

        # After timeout
        wd, cr = cm.tick(100.6, watchdog_timeout=0.5)
        assert len(wd) == 1
        assert wd[0].robot_id == 1
        assert wd[0].client_id == "client_a"

        # Should not fire again
        wd, cr = cm.tick(101.0, watchdog_timeout=0.5)
        assert len(wd) == 0

    def test_does_not_fire_for_zero_velocity(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.0, 0.0, 100.0)
        wd, _ = cm.tick(101.0, watchdog_timeout=0.5)
        assert len(wd) == 0

    def test_resets_on_new_command(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        wd, _ = cm.tick(100.6, watchdog_timeout=0.5)
        assert len(wd) == 1

        # New command resets watchdog
        cm.record_command("client_a", 1, 0.2, 0.0, 101.0)
        wd, _ = cm.tick(101.3, watchdog_timeout=0.5)
        assert len(wd) == 0  # not yet timed out


class TestControlTimeout:
    def test_releases_after_timeout(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        _, cr = cm.tick(131.0, control_timeout=30.0)
        assert len(cr) == 1
        assert cr[0].robot_id == 1
        assert cm.get_controller(1) is None

    def test_does_not_release_before_timeout(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        _, cr = cm.tick(120.0, control_timeout=30.0)
        assert len(cr) == 0
        assert cm.get_controller(1) == "client_a"

    def test_watchdog_before_control_timeout(self) -> None:
        cm = ControlManager()
        cm.record_command("client_a", 1, 0.1, 0.0, 100.0)
        # Watchdog fires at 0.5s
        wd, cr = cm.tick(100.6, watchdog_timeout=0.5, control_timeout=30.0)
        assert len(wd) == 1
        assert len(cr) == 0
        # Control still held
        assert cm.get_controller(1) == "client_a"
