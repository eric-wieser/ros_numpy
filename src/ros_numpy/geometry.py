from .registry import converts_from_numpy, converts_to_numpy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Point, Pose
from . import numpify

import numpy as np

# transformation helpers
_EPS = np.finfo(float).eps * 4.0


def translation_matrix(direction):
	"""Create 4x4 array to translate by the direction.

	Parameters
	----------
	direction : array-like
		Translation x, y, z vector.

	Returns
	-------
	array
		4x4 transformation matrix with translation

	Notes
	-----
	Adapted from:
	https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py

	Examples
	--------
	>>> v = np.random.random(3) - 0.5
	>>> np.allclose(v, translation_matrix(v)[:3, 3])
	True
	"""
	M = np.identity(4)
	M[:3, 3] = direction[:3]
	return M


def translation_from_matrix(matrix):
	"""Get translation vector from transformation matrix.

	Parameters
	----------
	matrix : 2D array-like
		4x4 transformation matrix

	Returns
	-------
	array
		Translation direction vector

	Notes
	-----
	Adapted from:
	https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py

	Examples
	--------
	>>> v0 = np.random.random(3) - 0.5
	>>> v1 = translation_from_matrix(translation_matrix(v0))
	>>> np.allclose(v0, v1)
	True
	"""
	return np.array(matrix, copy=False)[:3, 3].copy()


def quaternion_matrix(quaternion):
	"""Return homogeneous rotation matrix from quaternion.

	Parameters
	----------
	quaternion : 1D array-like
		Rotation as a quaternion (w, x, y, z)

	Returns
	-------
	array
		4x4 transformation matrix with rotation

	Notes
	-----
	Adapted from:
	https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py
	"""
	q = np.array(quaternion[:4], dtype=np.float64, copy=True)
	nq = np.dot(q, q)
	if nq < _EPS:
		return np.identity(4, dtype=np.float64)
	q *= np.sqrt(2.0 / nq)
	q = np.outer(q, q)
	return np.array((
		(1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
		(    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
		(    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
		(                0.0,                 0.0,                 0.0, 1.0)
		), dtype=np.float64)


def quaternion_from_matrix(matrix):
	"""Return quaternion from rotation matrix.

	Parameters
	----------
	matrix : 2D array-like
		4x4 transformation matrix

	Returns
	-------
	array-like
		Rotation as a quaternion (w, x, y, z)

	Notes
	-----
	Adapted from:
 	https://github.com/ros/geometry/blob/noetic-devel/tf/src/tf/transformations.py
 	"""
	q = np.empty((4, ), dtype=np.float64)
	M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
	t = np.trace(M)
	if t > M[3, 3]:
		q[3] = t
		q[2] = M[1, 0] - M[0, 1]
		q[1] = M[0, 2] - M[2, 0]
		q[0] = M[2, 1] - M[1, 2]
	else:
		i, j, k = 0, 1, 2
		if M[1, 1] > M[0, 0]:
			i, j, k = 1, 2, 0
		if M[2, 2] > M[i, i]:
			i, j, k = 2, 0, 1
		t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
		q[i] = t
		q[j] = M[i, j] + M[j, i]
		q[k] = M[k, i] + M[i, k]
		q[3] = M[k, j] - M[j, k]
	q *= 0.5 / np.sqrt(t * M[3, 3])
	return q

# basic types

@converts_to_numpy(Vector3)
def vector3_to_numpy(msg, hom=False):
	if hom:
		return np.array([msg.x, msg.y, msg.z, 0])
	else:
		return np.array([msg.x, msg.y, msg.z])

@converts_from_numpy(Vector3)
def numpy_to_vector3(arr):
	if arr.shape[-1] == 4:
		assert np.all(arr[...,-1] == 0)
		arr = arr[...,:-1]

	if len(arr.shape) == 1:
		return Vector3(*arr)
	else:
		return np.apply_along_axis(lambda v: Vector3(*v), axis=-1, arr=arr)

@converts_to_numpy(Point)
def point_to_numpy(msg, hom=False):
	if hom:
		return np.array([msg.x, msg.y, msg.z, 1])
	else:
		return np.array([msg.x, msg.y, msg.z])

@converts_from_numpy(Point)
def numpy_to_point(arr):
	if arr.shape[-1] == 4:
		arr = arr[...,:-1] / arr[...,-1]

	if len(arr.shape) == 1:
		return Point(*arr)
	else:
		return np.apply_along_axis(lambda v: Point(*v), axis=-1, arr=arr)

@converts_to_numpy(Quaternion)
def quat_to_numpy(msg):
	return np.array([msg.x, msg.y, msg.z, msg.w])

@converts_from_numpy(Quaternion)
def numpy_to_quat(arr):
	assert arr.shape[-1] == 4

	if len(arr.shape) == 1:
		return Quaternion(*arr)
	else:
		return np.apply_along_axis(lambda v: Quaternion(*v), axis=-1, arr=arr)


# compound types
# all of these take ...x4x4 homogeneous matrices

@converts_to_numpy(Transform)
def transform_to_numpy(msg):
	return np.dot(
		translation_matrix(numpify(msg.translation)),
		quaternion_matrix(numpify(msg.rotation))
	)

@converts_from_numpy(Transform)
def numpy_to_transform(arr):
	shape, rest = arr.shape[:-2], arr.shape[-2:]
	assert rest == (4,4)

	if len(shape) == 0:
		trans = translation_from_matrix(arr)
		quat = quaternion_from_matrix(arr)

		return Transform(
			translation=Vector3(*trans),
			rotation=Quaternion(*quat)
		)
	else:
		res = np.empty(shape, dtype=np.object_)
		for idx in np.ndindex(shape):
			res[idx] = Transform(
				translation=Vector3(*translation_from_matrix(arr[idx])),
				rotation=Quaternion(*quaternion_from_matrix(arr[idx]))
			)

@converts_to_numpy(Pose)
def pose_to_numpy(msg):
	return np.dot(
		translation_matrix(numpify(msg.position)),
		quaternion_matrix(numpify(msg.orientation))
	)

@converts_from_numpy(Pose)
def numpy_to_pose(arr):
	shape, rest = arr.shape[:-2], arr.shape[-2:]
	assert rest == (4,4)

	if len(shape) == 0:
		trans = translation_from_matrix(arr)
		quat = quaternion_from_matrix(arr)

		return Pose(
			position=Vector3(*trans),
			orientation=Quaternion(*quat)
		)
	else:
		res = np.empty(shape, dtype=np.object_)
		for idx in np.ndindex(shape):
			res[idx] = Pose(
				position=Vector3(*translation_from_matrix(arr[idx])),
				orientation=Quaternion(*quaternion_from_matrix(arr[idx]))
			)
