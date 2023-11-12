# This Python file uses the following encoding: utf-8
"""Taken from
ros-indigo-geometry-msgs 1.11.9-0trusty-20160627-170848-0700
and modified to have a made up _md5sum for
testing purposes"""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg

class Pose(genpy.Message):
  _md5sum = "13371337133713371337133713371337"
  _type = "geometry_msgs/Pose"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
"""
  __slots__ = ['position','orientation']
  _slot_types = ['geometry_msgs/Point','geometry_msgs/Quaternion']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       position,orientation

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Pose, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
    else:
      self.position = geometry_msgs.msg.Point()
      self.orientation = geometry_msgs.msg.Quaternion()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.position.x, _x.position.y, _x.position.z, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.position.x, _x.position.y, _x.position.z, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_7d.pack(_x.position.x, _x.position.y, _x.position.z, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.position is None:
        self.position = geometry_msgs.msg.Point()
      if self.orientation is None:
        self.orientation = geometry_msgs.msg.Quaternion()
      end = 0
      _x = self
      start = end
      end += 56
      (_x.position.x, _x.position.y, _x.position.z, _x.orientation.x, _x.orientation.y, _x.orientation.z, _x.orientation.w,) = _struct_7d.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_7d = struct.Struct("<7d")
