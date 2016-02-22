import functools
import genpy
import collections
from . import numpy_msg

_to_numpy = {}
_from_numpy = {}

def converts_to_numpy(msgtype, plural=False):
	assert issubclass(msgtype, genpy.Message)
	def decorator(f):
		_to_numpy[msgtype, plural] = f
		_to_numpy[numpy_msg(msgtype), plural] = f
		return f
	return decorator

def converts_from_numpy(msgtype, plural=False):
	assert issubclass(msgtype, genpy.Message)
	def decorator(f):
		_from_numpy[msgtype, plural] = f
		_from_numpy[numpy_msg(msgtype), plural] = f
		return f
	return decorator

def numpify(msg, *args, **kwargs):
	if msg is None:
		return

	conv = _to_numpy.get((msg.__class__, False))
	if not conv and isinstance(msg, collections.Sequence):
		if not msg:
			raise ValueError("Cannot determine the type of an empty Collection")
		conv = _to_numpy.get((msg[0].__class__, True))


	if not conv:
		raise ValueError("Unable to convert message {} - only supports {}".format(
			msg.__class__.__name__,
			', '.join(cls.__name__ + ("[]" if pl else '') for cls, pl in _to_numpy.keys())
		))

	return conv(msg, *args, **kwargs)

def msgify(msg_type, numpy_obj, *args, **kwargs):
	conv = _from_numpy.get((msg_type, kwargs.pop('plural', False)))
	if not conv:
		raise ValueError("Unable to build message {} - only supports {}".format(
			msg_type.__name__,
			', '.join(cls.__name__ + ("[]" if pl else '') for cls, pl in _to_numpy.keys())
		))
	return conv(numpy_obj, *args, **kwargs)
