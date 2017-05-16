#!/usr/bin/env python
# Create a rosbag with the supported types
import rosbag
from ros_numpy import msgify, numpy_msg
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3, Point, Quaternion, Transform, Pose
import zlib, struct


def makeArray(npoints):
	points_arr = np.zeros((npoints,), dtype=[
		('x', np.float32),
		('y', np.float32),
		('z', np.float32),
		('r', np.uint8),
		('g', np.uint8),
		('b', np.uint8)])
	points_arr['x'] = np.random.random((npoints,))
	points_arr['y'] = np.random.random((npoints,))
	points_arr['z'] = np.random.random((npoints,))
	points_arr['r'] = np.floor(np.random.random((npoints,)) * 255)
	points_arr['g'] = 0
	points_arr['b'] = 255

	return points_arr

# From http://stackoverflow.com/questions/902761/saving-a-numpy-array-as-an-image
def write_png(buf, width, height):
	""" buf: must be bytes or a bytearray in Python3.x,
		a regular string in Python2.x.
	"""

	# reverse the vertical line order and add null bytes at the start
	width_byte_4 = width * 4
	raw_data = b''.join(b'\x00' + buf[span:span + width_byte_4]
						for span in range((height - 1) * width_byte_4, -1, - width_byte_4))

	def png_pack(png_tag, data):
		chunk_head = png_tag + data
		return (struct.pack("!I", len(data)) +
				chunk_head +
				struct.pack("!I", 0xFFFFFFFF & zlib.crc32(chunk_head)))

	return b''.join([
		b'\x89PNG\r\n\x1a\n',
		png_pack(b'IHDR', struct.pack("!2I5B", width, height, 8, 6, 0, 0, 0)),
		png_pack(b'IDAT', zlib.compress(raw_data, 9)),
		png_pack(b'IEND', b'')])

def get_png_numpy_array(array):
	if any([len(row) != len(array[0]) for row in array]):
		raise ValueError, "Array should have elements of equal size"

		# First row becomes top row of image.
	flat = []
	map(flat.extend, reversed(array))
	# Big-endian, unsigned 32-byte integer.
	buf = b''.join([struct.pack('>I', ((0xffFFff & i32) << 8) | (i32 >> 24))
					for i32 in flat])  # Rotate from ARGB to RGBA.

	data = write_png(buf, len(array[0]), len(array))
	return data

bag = rosbag.Bag('test.bag', 'w')

try:
	arr = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.uint8)
	img = msgify(Image, arr, encoding='rgb8')
	bag.write('/image', img)

	points_arr = makeArray(100)
	cloud_msg = msgify(numpy_msg(PointCloud2), points_arr)
	bag.write('/pointcloud', cloud_msg)

	data = -np.ones((30, 30), np.int8)
	data[10:20, 10:20] = 100
	occgrid = msgify(OccupancyGrid, data)
	bag.write('/occupancygrid', occgrid)

	v = Vector3(1.0, 2.0, 3.0)
	bag.write('/vector3', v)

	p = Point(0.0, 0.1, 0.3)
	bag.write('/point', p)

	q = Quaternion(0.0, 0.0, 0.0, 1.0)
	bag.write('/quaternion', q)

	t = Transform(v, q)
	bag.write('/transform', t)

	ps = Pose(p, q)
	bag.write('/pose', ps)

	ci = CompressedImage()
	ci.format = 'png'
	ci.data = get_png_numpy_array([[0xffFF0000, 0xffFFFF00],
								   [0xff00aa77, 0xff333333]])
	bag.write('/compressedimage', ci)

finally:
	bag.close()
