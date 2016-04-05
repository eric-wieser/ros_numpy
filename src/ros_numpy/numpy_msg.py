import rospy.numpy_msg

"""
The building numpy_msg has a bug - see https://github.com/ros/ros_comm/pull/743

This patches it
"""

_numpy_msg = rospy.numpy_msg.numpy_msg
_cached = {}
def numpy_msg(cls):
	if cls not in _cached:
		res = _numpy_msg(cls)
		_cached[cls] = res
	else:
		res = _cached[cls]

	return res

# patch the original for good measure
rospy.numpy_msg.numpy_msg = numpy_msg
