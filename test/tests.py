import unittest
import numpy as np

from ros_numpy import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

class TestPointClouds(unittest.TestCase):
    def makeArray(self, npoints):
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

    def test_convert_dtype(self):
        msg = PointCloud2(
            fields=[
                PointField(name='x', offset=0, count=1, datatype=PointField.FLOAT32),
                PointField(name='y', offset=4, count=1, datatype=PointField.FLOAT32)
            ],
            point_step=8
        )
        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32)
        ])
        conv_fields = point_cloud2.dtype_to_fields(dtype)
        self.assertSequenceEqual(msg.fields, conv_fields, 'dtype->Pointfield Failed with simple values')

        conv_dtype = point_cloud2.pointcloud2_to_dtype(msg)
        self.assertSequenceEqual(dtype, conv_dtype, 'dtype->Pointfield Failed with simple values')

    def test_convert_dtype_inner(self):
        msg = PointCloud2(
            fields=[
                PointField(name='x', offset=0, count=1, datatype=PointField.FLOAT32),
                PointField(name='y', offset=4, count=1, datatype=PointField.FLOAT32),
                PointField(name='vectors', offset=8, count=3, datatype=PointField.FLOAT32)
            ],
            point_step=20
        )

        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('vectors', np.float32, (3,))
        ])

        conv_fields = point_cloud2.dtype_to_fields(dtype)
        self.assertSequenceEqual(msg.fields, conv_fields, 'dtype->Pointfield with inner dimensions')

        conv_dtype = point_cloud2.pointcloud2_to_dtype(msg)
        self.assertEqual(dtype, conv_dtype, 'Pointfield->dtype with inner dimensions')


    def test_roundtrip(self):

        points_arr = self.makeArray(100)
        cloud_msg = point_cloud2.array_to_pointcloud2(points_arr, merge_rgb=True)
        new_points_arr = point_cloud2.pointcloud2_to_array(cloud_msg, split_rgb=True)

        np.testing.assert_equal(points_arr, new_points_arr)


if __name__ == '__main__':
    unittest.main()
    # import rostest
    # rostest.rosrun('ros_numpy', 'test_point_cloud', TestPointClouds)