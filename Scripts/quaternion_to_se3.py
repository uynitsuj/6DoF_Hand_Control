import numpy as np


def normalize_quaternion(quat):
    norm = np.linalg.norm(quat)
    return tuple(q / norm for q in quat)


def quaternion_to_se3(quat):
    w, x, y, z = quat
    Nq = w*w + x*x + y*y + z*z
    if Nq < np.finfo(float).eps:
        raise ValueError("Input quaternion has zero length.")

    s = 2.0 / Nq
    xs, ys, zs = x * s, y * s, z * s
    wx, wy, wz = w * xs, w * ys, w * zs
    xx, xy, xz = x * xs, x * ys, x * zs
    yy, yz, zz = y * ys, y * zs, z * zs

    se3 = np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy, 0],
        [xy + wz, 1.0 - (xx + zz), yz - wx, 0],
        [xz - wy, yz + wx, 1.0 - (xx + yy), 0],
        [0, 0, 0, 1]
    ])

    return se3


# Example usage:

quat = (1, 0, 0, 0)  # Identity quaternion
normalized_quat = normalize_quaternion(quat)
se3 = quaternion_to_se3(normalized_quat)
print(se3)
