#!/usr/bin/env python
# Create a rosbag with the supported types
import rosbag
from ros_numpy import msgify, numpy_msg
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Vector3, Point, Quaternion, Transform, Pose


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
	points_arr['r'] = np.floor(np.random.random((npoints,))*255)
	points_arr['g'] = 0
	points_arr['b'] = 255

	return points_arr


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
	
finally:
	bag.close()
