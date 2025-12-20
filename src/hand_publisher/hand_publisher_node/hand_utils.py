import numpy as np
from scipy.spatial.transform import Rotation as R


def is_closed(hand_points: np.ndarray, threshold: float = 0.15):
    return np.linalg.norm(hand_points[8] - hand_points[4]) < threshold


def forward_vector(hand_points: np.ndarray) -> np.ndarray:
    return hand_points[4] - hand_points[1]


def side_vector(hand_points: np.ndarray) -> np.ndarray:
    return hand_points[5] - hand_points[1]


def hand_to_rot(hand_points: np.ndarray) -> R:
    """This will depend on if the hand is the right hand or left hand

    Assuming right hand for now"""
    _forward = forward_vector(hand_points)
    _forward /= np.linalg.norm(_forward)
    _side_skew_vector = side_vector(hand_points)
    _orthogonal_side_vec = _forward - _side_skew_vector * np.dot(
        _forward, _side_skew_vector
    )
    _orthogonal_side_vec /= np.linalg.norm(_orthogonal_side_vec)
    _up = np.cross(_orthogonal_side_vec, _forward)
    _up /= np.linalg.norm(_up)
    # assuming camera is looking upwards
    # TODO: set the camera transform to some position looking from the side
    return R.from_matrix(np.vstack([_up, -_forward, _orthogonal_side_vec]).T)


def hand_to_pose(hand_points: np.ndarray) -> tuple[R, np.ndarray]:
    return hand_to_rot(hand_points), np.mean(hand_points, axis=0)
