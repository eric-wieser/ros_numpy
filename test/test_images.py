import unittest
import numpy as np
import ros_numpy
from sensor_msgs.msg import PointCloud2, PointField, Image

class TestImages(unittest.TestCase):
    def test_roundtrip_rgb8(self):
        arr = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.uint8)
        msg = ros_numpy.msgify(Image, arr, encoding='rgb8')
        arr2 = ros_numpy.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_roundtrip_mono(self):
        arr = np.random.randint(0, 256, size=(240, 360)).astype(np.uint8)
        msg = ros_numpy.msgify(Image, arr, encoding='mono8')
        arr2 = ros_numpy.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_roundtrip_big_endian(self):
        arr = np.random.randint(0, 256, size=(240, 360)).astype('>u2')
        msg = ros_numpy.msgify(Image, arr, encoding='mono16')
        self.assertEqual(msg.is_bigendian, True)
        arr2 = ros_numpy.numpify(msg)

        np.testing.assert_equal(arr, arr2)

    def test_roundtrip_little_endian(self):
        arr = np.random.randint(0, 256, size=(240, 360)).astype('<u2')
        msg = ros_numpy.msgify(Image, arr, encoding='mono16')
        self.assertEqual(msg.is_bigendian, False)
        arr2 = ros_numpy.numpify(msg)

        np.testing.assert_equal(arr, arr2)


    def test_bad_encodings(self):
        mono_arr = np.random.randint(0, 256, size=(240, 360)).astype(np.uint8)
        mono_arrf = np.random.randint(0, 256, size=(240, 360)).astype(np.float32)
        rgb_arr = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.uint8)
        rgb_arrf = np.random.randint(0, 256, size=(240, 360, 3)).astype(np.float32)

        with self.assertRaises(TypeError):
            msg = ros_numpy.msgify(Image, rgb_arr, encoding='mono8')
        with self.assertRaises(TypeError):
            msg = ros_numpy.msgify(Image, mono_arrf, encoding='mono8')

        with self.assertRaises(TypeError):
            msg = ros_numpy.msgify(Image, rgb_arrf, encoding='rgb8')
        with self.assertRaises(TypeError):
            msg = ros_numpy.msgify(Image, mono_arr, encoding='rgb8')


if __name__ == '__main__':
    unittest.main()
