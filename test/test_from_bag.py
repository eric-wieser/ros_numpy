import unittest
import numpy as np
import ros_numpy
from rosbag import Bag
from ros_numpy import msgify
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
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

	def test_compressed_image_from_bag(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/compressedimage']):
			arr = ros_numpy.numpify(bagmsg)
			msg = CompressedImage()
			for slot in msg.__slots__:
				# Go thru all the slots, data, header...
				setattr(msg, slot, getattr(bagmsg, slot))
			arr2 = ros_numpy.numpify(msg)
			np.testing.assert_equal(arr, arr2)

	def test_pose_from_bag_with_different_md5(self):
		bag = Bag('test.bag')
		# There is only one message
		for topic, bagmsg, time in bag.read_messages(topics=['/pose']):
			# use a different md5 than the installed one, for reference
			# ros-indigo-geometry-msgs 1.11.9-0trusty-20160627-170848-0700
			# Pose msg has md5sum e45d45a5a1ce597b249e23fb30fc871f
			# so we use one made up from:
			from .different_md5_Pose import Pose as different_md5_Pose
			msg = different_md5_Pose()
			with self.assertRaises(ValueError):
				arr = ros_numpy.numpify(msg)



if __name__ == '__main__':
	unittest.main()
