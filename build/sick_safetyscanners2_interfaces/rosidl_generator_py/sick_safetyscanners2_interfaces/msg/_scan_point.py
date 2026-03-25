# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/ScanPoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ScanPoint(type):
    """Metaclass of message 'ScanPoint'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('sick_safetyscanners2_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'sick_safetyscanners2_interfaces.msg.ScanPoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__scan_point
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__scan_point
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__scan_point
            cls._TYPE_SUPPORT = module.type_support_msg__msg__scan_point
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__scan_point

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ScanPoint(metaclass=Metaclass_ScanPoint):
    """Message class 'ScanPoint'."""

    __slots__ = [
        '_angle',
        '_distance',
        '_reflectivity',
        '_valid',
        '_infinite',
        '_glare',
        '_reflector',
        '_contamination',
        '_contamination_warning',
    ]

    _fields_and_field_types = {
        'angle': 'float',
        'distance': 'uint16',
        'reflectivity': 'uint8',
        'valid': 'boolean',
        'infinite': 'boolean',
        'glare': 'boolean',
        'reflector': 'boolean',
        'contamination': 'boolean',
        'contamination_warning': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.angle = kwargs.get('angle', float())
        self.distance = kwargs.get('distance', int())
        self.reflectivity = kwargs.get('reflectivity', int())
        self.valid = kwargs.get('valid', bool())
        self.infinite = kwargs.get('infinite', bool())
        self.glare = kwargs.get('glare', bool())
        self.reflector = kwargs.get('reflector', bool())
        self.contamination = kwargs.get('contamination', bool())
        self.contamination_warning = kwargs.get('contamination_warning', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.angle != other.angle:
            return False
        if self.distance != other.distance:
            return False
        if self.reflectivity != other.reflectivity:
            return False
        if self.valid != other.valid:
            return False
        if self.infinite != other.infinite:
            return False
        if self.glare != other.glare:
            return False
        if self.reflector != other.reflector:
            return False
        if self.contamination != other.contamination:
            return False
        if self.contamination_warning != other.contamination_warning:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def angle(self):
        """Message field 'angle'."""
        return self._angle

    @angle.setter
    def angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angle = value

    @builtins.property
    def distance(self):
        """Message field 'distance'."""
        return self._distance

    @distance.setter
    def distance(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'distance' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'distance' field must be an unsigned integer in [0, 65535]"
        self._distance = value

    @builtins.property
    def reflectivity(self):
        """Message field 'reflectivity'."""
        return self._reflectivity

    @reflectivity.setter
    def reflectivity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reflectivity' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'reflectivity' field must be an unsigned integer in [0, 255]"
        self._reflectivity = value

    @builtins.property
    def valid(self):
        """Message field 'valid'."""
        return self._valid

    @valid.setter
    def valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid' field must be of type 'bool'"
        self._valid = value

    @builtins.property
    def infinite(self):
        """Message field 'infinite'."""
        return self._infinite

    @infinite.setter
    def infinite(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'infinite' field must be of type 'bool'"
        self._infinite = value

    @builtins.property
    def glare(self):
        """Message field 'glare'."""
        return self._glare

    @glare.setter
    def glare(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'glare' field must be of type 'bool'"
        self._glare = value

    @builtins.property
    def reflector(self):
        """Message field 'reflector'."""
        return self._reflector

    @reflector.setter
    def reflector(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reflector' field must be of type 'bool'"
        self._reflector = value

    @builtins.property
    def contamination(self):
        """Message field 'contamination'."""
        return self._contamination

    @contamination.setter
    def contamination(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'contamination' field must be of type 'bool'"
        self._contamination = value

    @builtins.property
    def contamination_warning(self):
        """Message field 'contamination_warning'."""
        return self._contamination_warning

    @contamination_warning.setter
    def contamination_warning(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'contamination_warning' field must be of type 'bool'"
        self._contamination_warning = value
