import unittest
import numpy as np
import ros_numpy
from rosbag import Bag
from ros_numpy import msgify
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3, Point, Quaternion, Transform, Pose


class TestImagesFromRosbag(unittest.TestCase):
	def test_image_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/image']):
			arr = ros_numpy.numpify(bagmsg)
			msg = Image()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_pointcloud2_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/poincloud']):
			arr = ros_numpy.numpify(msg)
			msg = PointCloud2()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_occupancy_grid_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/occupancygrid']):
			arr = ros_numpy.numpify(bagmsg)
			msg = OccupancyGrid()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_vector3_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/vector3']):
			arr = ros_numpy.numpify(bagmsg)
			msg = Vector3()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_point_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/point']):
			arr = ros_numpy.numpify(bagmsg)
			msg = Point()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_quaternion_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/quaternion']):
			arr = ros_numpy.numpify(bagmsg)
			msg = Quaternion()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_transform_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/transform']):
			arr = ros_numpy.numpify(bagmsg)
			msg = Transform()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_pose_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/pose']):
			arr = ros_numpy.numpify(bagmsg)
			msg = Pose()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)


if __name__ == '__main__':
	unittest.main()
