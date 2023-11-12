from .registry import converts_from_numpy, converts_to_numpy
from geometry_msgs.msg import Transform, Vector3, Quaternion, Point, Pose
from geometry_msgs.msg import TransformStamped, Vector3Stamped, QuaternionStamped, PointStamped, PoseStamped
from . import numpify

import numpy as np

# basic types

@converts_to_numpy(Vector3)
def vector3_to_numpy(msg, hom=False):
	if hom:
		return np.array([msg.x, msg.y, msg.z, 0])
	else:
		return np.array([msg.x, msg.y, msg.z])

@converts_to_numpy(Vector3Stamped)
def vector3stamped_to_numpy(msg, hom=False):
	return vector3_to_numpy(msg.vector, hom)

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

@converts_to_numpy(PointStamped)
def pointstamped_to_numpy(msg, hom=False):
	return point_to_numpy(msg.point, hom)

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

@converts_to_numpy(QuaternionStamped)
def quatstamped_to_numpy(msg):
	return quat_to_numpy(msg.quaternion)

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
	from tf import transformations

	return np.dot(
        transformations.translation_matrix(numpify(msg.translation)),
        transformations.quaternion_matrix(numpify(msg.rotation))
    )

@converts_to_numpy(TransformStamped)
def transformstamped_to_numpy(msg):
	return transform_to_numpy(msg.transform)

@converts_from_numpy(Transform)
def numpy_to_transform(arr):
	from tf import transformations

	shape, rest = arr.shape[:-2], arr.shape[-2:]
	assert rest == (4,4)

	if len(shape) == 0:
		trans = transformations.translation_from_matrix(arr)
		quat = transformations.quaternion_from_matrix(arr)

		return Transform(
			translation=Vector3(*trans),
			rotation=Quaternion(*quat)
		)
	else:
		res = np.empty(shape, dtype=np.object_)
		for idx in np.ndindex(shape):
			res[idx] = Transform(
				translation=Vector3(*transformations.translation_from_matrix(arr[idx])),
				rotation=Quaternion(*transformations.quaternion_from_matrix(arr[idx]))
			)

@converts_to_numpy(Pose)
def pose_to_numpy(msg):
	from tf import transformations

	return np.dot(
        transformations.translation_matrix(numpify(msg.position)),
        transformations.quaternion_matrix(numpify(msg.orientation))
    )

@converts_to_numpy(PoseStamped)
def posestamped_to_numpy(msg):
	return pose_to_numpy(msg.pose)

@converts_from_numpy(Pose)
def numpy_to_pose(arr):
	from tf import transformations

	shape, rest = arr.shape[:-2], arr.shape[-2:]
	assert rest == (4,4)

	if len(shape) == 0:
		trans = transformations.translation_from_matrix(arr)
		quat = transformations.quaternion_from_matrix(arr)

		return Pose(
			position=Vector3(*trans),
			orientation=Quaternion(*quat)
		)
	else:
		res = np.empty(shape, dtype=np.object_)
		for idx in np.ndindex(shape):
			res[idx] = Pose(
				position=Vector3(*transformations.translation_from_matrix(arr[idx])),
				orientation=Quaternion(*transformations.quaternion_from_matrix(arr[idx]))
			)

