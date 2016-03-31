import itertools
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout
from std_msgs.msg import (
	Float32MultiArray,
	Float64MultiArray,
	Int8MultiArray,
	Int16MultiArray,
	Int32MultiArray,
	Int64MultiArray,
	UInt8MultiArray,
	UInt16MultiArray,
	UInt32MultiArray,
	UInt64MultiArray
)

dtypes = {
	Float32MultiArray: np.float32,
	Float64MultiArray: np.float64,
	Int8MultiArray:    np.int8,
	Int16MultiArray:   np.int16,
	Int32MultiArray:   np.int32,
	Int64MultiArray:   np.int64,
	UInt8MultiArray:   np.int8,
	UInt16MultiArray:  np.int16,
	UInt32MultiArray:  np.int32,
	UInt64MultiArray:  np.int64
}

def _add_converters(msg_class, np_type):
	dtype = np.dtype(np_type)

	@converts_to_numpy(msg_class)
	def _convert(msg):
		layout = msg.layout
		shape = tuple(d.size for d in layout.dims)
		stride = tuple(d.stride for d in layout.dims[1:]) + (1,)
		stride = (dtype.itemsize * s for s in stride)

		res = np.asarray(msg.data, dtype=dtype, shape=shape)
		res.stride = stride
		return res

	@converts_from_numpy(msg_class)
	def _convert(arr, labels=None):
		if not labels:
			labels = itertools.count()
		if arr.strides and arr.strides[-1] != 1:
			raise ValueError("Last stride must be 1 - see https://github.com/ros/std_msgs/issues/8")


		strides = (np.prod(arr.shape),)
		for s in arr.strides[1:]:
			if s % dtype.itemsize != 0:
				raise ValueError('Strides cannot jump across word boundaries')

			strides = strides + (s,)

		msg = msg_class(
			layout=MultiArrayLayout(
				data_offset=0,
				dims=[
					MultiArrayDimension(
						size=size,
						stride=stride,
						label=label
					)
					for size, stride, label in zip(arr.shape, strides, labels)
				]
			),
			data=
		)



for msg_class, np_type in dtypes.items():
	_add_converters(msg_class, np_type)