import unittest
import numpy as np

class TestPointClouds(unittest.TestCase):
    def test_roundtrip(self):
        from ros_numpy import point_cloud2
        import numpy as np    

        npoints = 100

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

        cloud_msg = point_cloud2.array_to_pointcloud2(points_arr, merge_rgb=True)
        new_points_arr = point_cloud2.pointcloud2_to_array(cloud_msg, split_rgb=True)

        np.testing.assert_equal(points_arr, new_points_arr)


if __name__ == '__main__':
    unittest.main()
    # import rostest
    # rostest.rosrun('ros_numpy', 'test_point_cloud', TestPointClouds)