# ros_numpy
Tools for converting ROS messages to and from numpy arrays. Contains two functions:

* `arr = numpify(msg, ...)` - try to get a numpy object from a message
* `msg = msgify(MessageType, arr, ...)` - try and convert a numpy object to a message

Currently supports:

* `sensor_msgs.msg.PointCloud2` &harr; structured `np.array`:
   
   ```python
   data = np.zeros(100, dtype=[
     ('x', np.float32),
     ('y', np.float32),
     ('vectors', np.float32, (3,))
   ])
   data['x'] = np.arange(100)
   data['y'] = data['x']*2
   data['z'] = np.arange(100)[:,None]
   
   msg = ros_numpy.msgify(PointCloud2, data)
   ```
   
   ```
   data = ros_numpy.numpify(msg)
   ```

* `sensor_msgs.msg.Image` &harr; 2/3-D `np.array`, similar to the function of `cv_bridge`, but without the dependency on `cv2`

Support for more types can be added with:

```python
@ros_numpy.converts_to_numpy(SomeMessageClass)
def convert(my_msg):
    return np.array(...)

@ros_numpy.converts_from_numpy(SomeMessageClass)
def convert(my_array):
    return SomeMessageClass(...)
```

Any extra args or kwargs to `numpify` or `msgify` will be forwarded to your conversion function
