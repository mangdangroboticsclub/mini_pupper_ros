# Copyright 2025 Kishan Grewal
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Imports
from mini_pupper_tracking.movement_node import PID
from mini_pupper_tracking.movement_node import MovementNode
import pytest
import sys
import os
from unittest.mock import patch, MagicMock
import rclpy

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_pupper_tracking'))

# Fixtures


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def track_parameters():
    return {
        'yaw.Kp': 5.0,
        'yaw.Kd': 0.1,
        'yaw.decay': 0.5,
        'yaw.clamp': 2.0,
        'yaw.stable_minimum': 0.65,
        'yaw.tracking_enabled': True,
        'pitch.alpha': 0.1,
        'pitch.gain': 1.0,
        'pitch.decay': 0.5,
        'pitch.camera_deadband': 0.020,
        'pitch.tracking_enabled': True
    }


DT = 0.1
KI = 0.0
DEF_CX = 0.5
DEF_TY = 0.4
DEF_A = 0.5

# Test PID Helper


@pytest.mark.parametrize("Kp, Kd, error, expected", [
 (2.0, 0.0, 2.0, 4.0),
 (0.0, 3.0, 2.0, 60.0),
 (2.0, 3.0, 2.0, 64.0)
])
def test_pid_compute(Kp, Kd, error, expected):
    pid = PID(Kp=Kp, Ki=KI, Kd=Kd)
    pid.compute(error=0.0, dt=DT)
    result = pid.compute(error=error, dt=DT)
    assert result == pytest.approx(expected), f"Expected {expected}, got {result}"


# Test Node Logic


@pytest.mark.parametrize("center_x, expected_direction", [  # YAW
    (0.0, "left"),
    (DEF_CX, "center"),
    (1.0, "right")
])
def test_yaw_direction(ros_context, track_parameters, center_x, expected_direction):
    with patch.object(MovementNode, 'get_parameter') as mock_param:
        mock_param.side_effect = lambda name: MagicMock(value=track_parameters.get(name, 1.0))
        node = MovementNode()

        fake_track = MagicMock(center_x=center_x, top_y=DEF_TY, bounding_area=DEF_A)
        node.tracking_callback(MagicMock(tracks=[fake_track]))
        node.yaw_callback()

        if expected_direction == "left":
            assert node.last_yaw_rate > 0.0
        elif expected_direction == "right":
            assert node.last_yaw_rate < 0.0
        elif expected_direction == "center":
            assert node.last_yaw_rate == pytest.approx(0.0)


@pytest.mark.parametrize("top_y, expected_direction", [  # PITCH
    (0.0, "up"),
    (DEF_TY, "center"),
    (1.0, "down")
])
def test_pitch_direction(ros_context, track_parameters, top_y, expected_direction):
    with patch.object(MovementNode, 'get_parameter') as mock_param:
        mock_param.side_effect = lambda name: MagicMock(value=track_parameters.get(name, 1.0))
        node = MovementNode()

        fake_track = MagicMock(center_x=DEF_CX, top_y=top_y, bounding_area=DEF_A)
        node.tracking_callback(MagicMock(tracks=[fake_track]))
        node.pitch_callback()

        if expected_direction == "up":
            assert node.pitch_value - node.current_pitch > 0.0
        elif expected_direction == "down":
            assert node.pitch_value - node.current_pitch < 0.0
        elif expected_direction == "center":
            assert node.pitch_value - node.current_pitch == pytest.approx(0.0)
