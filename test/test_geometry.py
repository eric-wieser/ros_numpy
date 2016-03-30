import unittest
import numpy as np
import ros_numpy

from tf import transformations

from geometry_msgs.msg import Vector3, Quaternion, Transform, Point, Pose

class TestGeometry(unittest.TestCase):
    def test_point(self):
        p = Point(1, 2, 3)

        p_arr = ros_numpy.numpify(p)
        np.testing.assert_array_equal(p_arr, [1, 2, 3])

        p_arrh = ros_numpy.numpify(p, hom=True)
        np.testing.assert_array_equal(p_arrh, [1, 2, 3, 1])

        self.assertEqual(p, ros_numpy.msgify(Point, p_arr))
        self.assertEqual(p, ros_numpy.msgify(Point, p_arrh))
        self.assertEqual(p, ros_numpy.msgify(Point, p_arrh * 2))

    def test_vector3(self):
        v = Vector3(1, 2, 3)

        v_arr = ros_numpy.numpify(v)
        np.testing.assert_array_equal(v_arr, [1, 2, 3])

        v_arrh = ros_numpy.numpify(v, hom=True)
        np.testing.assert_array_equal(v_arrh, [1, 2, 3, 0])

        self.assertEqual(v, ros_numpy.msgify(Vector3, v_arr))
        self.assertEqual(v, ros_numpy.msgify(Vector3, v_arrh))

        with self.assertRaises(AssertionError):
            ros_numpy.msgify(Vector3, np.array([0, 0, 0, 1]))

    def test_transform(self):
        t = Transform(
            translation=Vector3(1, 2, 3),
            rotation=Quaternion(*transformations.quaternion_from_euler(np.pi, 0, 0))
        )

        t_mat = ros_numpy.numpify(t)

        np.testing.assert_allclose(t_mat.dot([0, 0, 1, 1]), [1.0, 2.0, 2.0, 1.0])

        msg = ros_numpy.msgify(Transform, t_mat)

        np.testing.assert_allclose(msg.translation.x, t.translation.x)
        np.testing.assert_allclose(msg.translation.y, t.translation.y)
        np.testing.assert_allclose(msg.translation.z, t.translation.z)
        np.testing.assert_allclose(msg.rotation.x, t.rotation.x)
        np.testing.assert_allclose(msg.rotation.y, t.rotation.y)
        np.testing.assert_allclose(msg.rotation.z, t.rotation.z)
        np.testing.assert_allclose(msg.rotation.w, t.rotation.w)

    def test_pose(self):
        t = Pose(
            position=Point(1.0, 2.0, 3.0),
            orientation=Quaternion(*transformations.quaternion_from_euler(np.pi, 0, 0))
        )

        t_mat = ros_numpy.numpify(t)

        np.testing.assert_allclose(t_mat.dot([0, 0, 1, 1]), [1.0, 2.0, 2.0, 1.0])

        msg = ros_numpy.msgify(Pose, t_mat)

        np.testing.assert_allclose(msg.position.x, t.position.x)
        np.testing.assert_allclose(msg.position.y, t.position.y)
        np.testing.assert_allclose(msg.position.z, t.position.z)
        np.testing.assert_allclose(msg.orientation.x, t.orientation.x)
        np.testing.assert_allclose(msg.orientation.y, t.orientation.y)
        np.testing.assert_allclose(msg.orientation.z, t.orientation.z)
        np.testing.assert_allclose(msg.orientation.w, t.orientation.w)

if __name__ == '__main__':
    unittest.main()
