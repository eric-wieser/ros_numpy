import unittest
import numpy as np
import ros_numpy

from tf import transformations

from geometry_msgs.msg import Vector3, Quaternion, Transform

class TestTransform(unittest.TestCase):
    def test_aomething(self):
        t = Transform(
            translation=Vector3(1, 2, 3),
            rotation=Quaternion(*transformations.quaternion_from_euler(np.pi, 0, 0))
        )

        t_mat = ros_numpy.numpify(t)

        np.testing.assert_allclose(t_mat.dot([0, 0, 1, 1]), [1.0, 2.0, 2.0, 1.0])

        msg = ros_numpy.msgify(Transform, t_mat)

        self.assertEqual(msg.translation, t.translation)
        np.testing.assert_allclose(msg.rotation.x, t.rotation.x)
        np.testing.assert_allclose(msg.rotation.y, t.rotation.y)
        np.testing.assert_allclose(msg.rotation.z, t.rotation.z)
        np.testing.assert_allclose(msg.rotation.w, t.rotation.w)

if __name__ == '__main__':
    unittest.main()
