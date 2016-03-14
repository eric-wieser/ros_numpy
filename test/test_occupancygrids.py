import unittest
import numpy as np
import ros_numpy
from nav_msgs.msg import OccupancyGrid

class TestOccupancyGrids(unittest.TestCase):
    def test_masking(self):
        data = -np.ones((30, 30), np.int8)
        data[10:20, 10:20] = 100

        msg = ros_numpy.msgify(OccupancyGrid, data)

        data_out = ros_numpy.numpify(msg)

        self.assertIs(data_out[5, 5], np.ma.masked)
        np.testing.assert_equal(data_out[10:20, 10:20], 100)

if __name__ == '__main__':
    unittest.main()
