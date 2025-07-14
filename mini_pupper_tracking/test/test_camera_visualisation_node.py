# Imports
import pytest
import sys
import os
import unittest.mock
import rclpy
import math
from itertools import product

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'mini_pupper_tracking'))
from mini_pupper_tracking.camera_visualisation_node import CameraVisualisationNode
from unittest.mock import patch, MagicMock

# Fixtures

@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

binary_3bit = list(product([0, 1], repeat=3))

@pytest.mark.parametrize("center_x, top_y, bounding_area", binary_3bit)
def test_person_marker_creation(ros_context, center_x, top_y, bounding_area):
    node = CameraVisualisationNode()

    fake_track = MagicMock(center_x=center_x, top_y=top_y, bounding_area=bounding_area)
    node.people_callback(MagicMock(tracks=[fake_track]))

    assert len(node.people_points) == 1
    point = node.people_points[0]

    if center_x == 0.0:
        expected_angle_x = -node.fov_rad / 2
    elif center_x == 1.0:
        expected_angle_x = node.fov_rad / 2
    else:
        expected_angle_x = 0.0

    if top_y == 0.0:
        expected_angle_y = node.vertical_fov_rad / 2  # Top of image is 0, so this tilts up
    elif top_y == 1.0:
        expected_angle_y = -node.vertical_fov_rad / 2  # Bottom tilts down
    else:
        expected_angle_y = 0.0

    # Depth formula from the node
    depth = min(2.5, 0.3 / max(bounding_area, 0.001) ** 2.5)

    expected_x = depth
    expected_y = depth * math.tan(expected_angle_x)
    expected_z = depth * math.tan(expected_angle_y)

    assert point.x == pytest.approx(expected_x)
    assert point.y == pytest.approx(expected_y)
    assert point.z == pytest.approx(expected_z)


