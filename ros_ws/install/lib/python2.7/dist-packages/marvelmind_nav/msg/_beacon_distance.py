# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from marvelmind_nav/beacon_distance.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class beacon_distance(genpy.Message):
  _md5sum = "0961792211a42c14a3b38a49e24931f3"
  _type = "marvelmind_nav/beacon_distance"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """uint8 address_hedge
uint8 address_beacon
float64 distance_m
"""
  __slots__ = ['address_hedge','address_beacon','distance_m']
  _slot_types = ['uint8','uint8','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       address_hedge,address_beacon,distance_m

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(beacon_distance, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.address_hedge is None:
        self.address_hedge = 0
      if self.address_beacon is None:
        self.address_beacon = 0
      if self.distance_m is None:
        self.distance_m = 0.
    else:
      self.address_hedge = 0
      self.address_beacon = 0
      self.distance_m = 0.

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
      buff.write(_get_struct_2Bd().pack(_x.address_hedge, _x.address_beacon, _x.distance_m))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.address_hedge, _x.address_beacon, _x.distance_m,) = _get_struct_2Bd().unpack(str[start:end])
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
      buff.write(_get_struct_2Bd().pack(_x.address_hedge, _x.address_beacon, _x.distance_m))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 10
      (_x.address_hedge, _x.address_beacon, _x.distance_m,) = _get_struct_2Bd().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2Bd = None
def _get_struct_2Bd():
    global _struct_2Bd
    if _struct_2Bd is None:
        _struct_2Bd = struct.Struct("<2Bd")
    return _struct_2Bd
