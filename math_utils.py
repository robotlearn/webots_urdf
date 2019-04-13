"""Import modules."""

import numpy as np


def vectorNorm(data, axis=None, out=None):
    """Calculate norm of a vector."""
    data = np.array(data, dtype=np.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return np.sqrt(np.dot(data, data))
        data *= data
        out = np.atleast_1d(np.sum(data, axis=axis))
        np.sqrt(out, out)
        return out
    else:
        data *= data
        np.sum(data, axis=axis, out=out)
        np.sqrt(out, out)


def rotationFromQuaternion(q):
    """Convert quaternion to euler-axis-angle (vrml).

    Args:
        q (np.array of 4 float): quaternion (w,x,y,z)

    Returns:
        np.array of 4 float: euler axis-angle representation (x, y, z, angle)
    """
    q = np.array(q)
    v = np.array([0.0, 0.0, 0.0, 0.0])  # axis (x,y,z) + angle
    v[3] = 2.0 * np.arccos(q[0])  # angle
    if v[3] < 0.0001:
        # if v[3] close to zero then direction of axis not important
        v[:3] = [0.0, 1.0, 0.0]
    else:
        # normalise axes
        n = np.sqrt(np.sum(q[1:]**2))
        v[:3] = q[1:] / n
    return v


def convertRPYtoQuaternions(rpy, cylinder=False):
    """Convert RPY to quaternions.

    Args:
        rpy (np.array of 3 float): roll-pitch-yaw angle.
        cylinder (bool): If True, it is a cylinder.
    """
    if cylinder:
        rpy[0] += np.pi/2.
    cy, sy = np.cos(rpy[2]/2.), np.sin(rpy[2]/2.)
    cp, sp = np.cos(rpy[1]/2.), np.sin(rpy[1]/2.)
    cr, sr = np.cos(rpy[0]/2.), np.sin(rpy[0]/2.)

    q = np.array([0, 0, 0, 0])
    q[0] = cy * cp * cr + sy * sp * sr  # w
    q[1] = cy * cp * sr - sy * sp * cr  # x
    q[2] = sy * cp * sr + cy * sp * cr  # y
    q[3] = sy * cp * cr - cy * sp * sr  # z
    return q


def convertRPYtoEulerAxis(rpy, cylinder=False):
    """Convert RPY angles to Euler angles.

    Args:
        rpy (np.array of 3 float): roll-pitch-yaw angle.
        cylinder (bool): If True, it is a cylinder.
    """
    return rotationFromQuaternion(convertRPYtoQuaternions(rpy, cylinder))


def multiplyMatrix(mat1, mat2):
    """Multiply two matrices."""
    mat1 = np.array(mat1).reshape(3, 3)
    mat2 = np.array(mat2).reshape(3, 3)
    matrix = mat1.dot(mat2).reshape(-1)
    return matrix


def matrixFromRotation(rotation):
    """Get the 3x3 matrix associated to this VRML rotation.

    Args:
        rotation (np.array of 4 float): Euler axis (x,y,z) - angle representation.
    """
    x, y, z, a = rotation
    c, s = np.cos(a), np.sin(a)
    t1 = 1.0 - c
    t2 = x * z * t1
    t3 = x * y * t1
    t4 = y * z * t1
    matrix = []
    matrix.append(rotation[0] * rotation[0] * t1 + c)
    matrix.append(t3 - rotation[2] * s)
    matrix.append(t2 + rotation[1] * s)
    matrix.append(t3 + rotation[2] * s)
    matrix.append(rotation[1] * rotation[1] * t1 + c)
    matrix.append(t4 - rotation[0] * s)
    matrix.append(t2 - rotation[1] * s)
    matrix.append(t4 + rotation[0] * s)
    matrix.append(rotation[2] * rotation[2] * t1 + c)
    return matrix


def rotationFromMatrix(matrix):
    """Get the VRML rotation associated to this 3x3 matrix.

    Args:
        matrix (np.array[3,3]): rotation matrix.
    """
    rotation = []
    cosAngle = 0.5 * (matrix[0] + matrix[4] + matrix[8] - 1.0)
    absCosAngle = abs(cosAngle)
    if absCosAngle > 1.0:
        if (absCosAngle - 1.0) > 0.0000001:
            return [1.0, 0.0, 0.0, 0.0]
        if cosAngle < 0.0:
            cosAngle = -1.0
        else:
            cosAngle = 1.0

    rotation.append(matrix[7] - matrix[5])
    rotation.append(matrix[2] - matrix[6])
    rotation.append(matrix[3] - matrix[1])
    rotation.append(np.arccos(cosAngle))
    return rotation


def rotateVector(vector, rotation):
    """Rotate the vector by the VRML rotation."""
    # multiply matrix by vector
    matrix = matrixFromRotation(rotation)
    v = []
    v.append(vector[0] * matrix[0] + vector[1] * matrix[3] + vector[2] * matrix[6])
    v.append(vector[0] * matrix[1] + vector[1] * matrix[4] + vector[2] * matrix[7])
    v.append(vector[0] * matrix[2] + vector[1] * matrix[5] + vector[2] * matrix[8])
    return v
