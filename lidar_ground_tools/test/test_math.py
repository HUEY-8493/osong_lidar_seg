import math

import numpy as np

from lidar_ground_tools.calibrate_lidar_ground_once import (
    rotation_matrix_rpy,
    roll_pitch_from_ground_normal,
)


def test_roll_pitch_world_z_row_matches_normal():
    normal = np.array([-0.8136426463767683, -0.43852730476818463, -0.38167977018668525])
    normal = normal / np.linalg.norm(normal)

    roll, pitch = roll_pitch_from_ground_normal(normal)
    rotation = rotation_matrix_rpy(roll, pitch, math.radians(-25.0))

    assert np.allclose(rotation[2, :], normal, atol=1.0e-9)
