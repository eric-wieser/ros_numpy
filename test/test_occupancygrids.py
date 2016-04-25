import unittest
import numpy as np
import ros_numpy
from nav_msgs.msg import OccupancyGrid, MapMetaData

from io import BytesIO

class TestOccupancyGrids(unittest.TestCase):
    def test_masking(self):
        data = -np.ones((30, 30), np.int8)
        data[10:20, 10:20] = 100

        msg = ros_numpy.msgify(OccupancyGrid, data)

        data_out = ros_numpy.numpify(msg)

        self.assertIs(data_out[5, 5], np.ma.masked)
        np.testing.assert_equal(data_out[10:20, 10:20], 100)

    def test_serialization(self):
        msg = OccupancyGrid(
            info=MapMetaData(
                width=3,
                height=3
            ),
            data = [0, 0, 0, 0, -1, 0, 0, 0, 0]
        )

        data = ros_numpy.numpify(msg)
        self.assertIs(data[1,1], np.ma.masked)
        msg2 = ros_numpy.msgify(OccupancyGrid, data)

        self.assertEqual(msg.info, msg2.info)

        io1 = BytesIO()
        io2 = BytesIO()
        msg.serialize(io1)
        msg2.serialize(io2)

        self.assertEqual(io1.getvalue(), io2.getvalue(), "Message serialization survives round-trip")

if __name__ == '__main__':
    unittest.main()
