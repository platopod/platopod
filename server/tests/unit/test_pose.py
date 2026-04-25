"""Tests for plato_pod.pose — unified pose representation."""

from __future__ import annotations

import pytest

from plato_pod.pose import PoseSource, RobotPose


class TestPoseSource:
    def test_camera_artag(self) -> None:
        assert PoseSource.CAMERA_ARTAG.value == "camera_artag"

    def test_gps_imu(self) -> None:
        assert PoseSource.GPS_IMU.value == "gps_imu"

    def test_rf_anchor(self) -> None:
        assert PoseSource.RF_ANCHOR.value == "rf_anchor"

    def test_virtual_sim(self) -> None:
        assert PoseSource.VIRTUAL_SIM.value == "virtual_sim"

    def test_gazebo_sim(self) -> None:
        assert PoseSource.GAZEBO_SIM.value == "gazebo_sim"

    def test_unknown(self) -> None:
        assert PoseSource.UNKNOWN.value == "unknown"


class TestRobotPose:
    def test_fields(self) -> None:
        p = RobotPose(
            robot_id=1, x=0.5, y=0.3, theta=1.0,
            timestamp=1000.0, source=PoseSource.CAMERA_ARTAG,
            confidence=0.95, localization_id="tag_3",
        )
        assert p.robot_id == 1
        assert p.x == pytest.approx(0.5)
        assert p.source == PoseSource.CAMERA_ARTAG
        assert p.localization_id == "tag_3"

    def test_frozen(self) -> None:
        p = RobotPose(
            robot_id=1, x=0.0, y=0.0, theta=0.0,
            timestamp=0.0, source=PoseSource.UNKNOWN,
            confidence=0.0, localization_id="",
        )
        with pytest.raises(AttributeError):
            p.x = 1.0  # type: ignore[misc]

    def test_gps_pose(self) -> None:
        p = RobotPose(
            robot_id=5, x=10.5, y=-3.2, theta=0.785,
            timestamp=1700000000.0, source=PoseSource.GPS_IMU,
            confidence=0.8, localization_id="rover-1",
        )
        assert p.source == PoseSource.GPS_IMU
        assert p.localization_id == "rover-1"

    def test_rf_pose(self) -> None:
        p = RobotPose(
            robot_id=12, x=2.0, y=1.5, theta=3.14,
            timestamp=1700000000.0, source=PoseSource.RF_ANCHOR,
            confidence=0.6, localization_id="uwb-node-12",
        )
        assert p.source == PoseSource.RF_ANCHOR
