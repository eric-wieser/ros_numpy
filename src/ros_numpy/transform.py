from .registry import converts_from_numpy, converts_to_numpy
from geometry_msgs.msg import Transform, Vector3, Quaternion

import numpy as np
from numpy.lib.stride_tricks import as_strided

@converts_to_numpy(Transform)
def transform_to_numpy(msg):
	from tf import transformations

	return np.dot(
        transformations.translation_matrix([msg.translation.x, msg.translation.y, msg.translation.z]),
        transformations.quaternion_matrix([msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w])
    )


@converts_from_numpy(Transform)
def numpy_to_transform(arr):
	from tf import transformations

	assert arr.shape == (4,4)

	trans = transformations.translation_from_matrix(arr)
	quat = transformations.quaternion_from_matrix(arr)

	t = Transform()
	t.translation = Vector3(*trans)
	t.rotation = Quaternion(*quat)
	return t
