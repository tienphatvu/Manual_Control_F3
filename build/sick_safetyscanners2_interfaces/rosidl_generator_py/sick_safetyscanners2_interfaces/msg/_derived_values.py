# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/DerivedValues.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DerivedValues(type):
    """Metaclass of message 'DerivedValues'."""

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
                'sick_safetyscanners2_interfaces.msg.DerivedValues')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__derived_values
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__derived_values
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__derived_values
            cls._TYPE_SUPPORT = module.type_support_msg__msg__derived_values
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__derived_values

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DerivedValues(metaclass=Metaclass_DerivedValues):
    """Message class 'DerivedValues'."""

    __slots__ = [
        '_multiplication_factor',
        '_number_of_beams',
        '_scan_time',
        '_start_angle',
        '_angular_beam_resolution',
        '_interbeam_period',
    ]

    _fields_and_field_types = {
        'multiplication_factor': 'uint16',
        'number_of_beams': 'uint16',
        'scan_time': 'uint16',
        'start_angle': 'float',
        'angular_beam_resolution': 'float',
        'interbeam_period': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.multiplication_factor = kwargs.get('multiplication_factor', int())
        self.number_of_beams = kwargs.get('number_of_beams', int())
        self.scan_time = kwargs.get('scan_time', int())
        self.start_angle = kwargs.get('start_angle', float())
        self.angular_beam_resolution = kwargs.get('angular_beam_resolution', float())
        self.interbeam_period = kwargs.get('interbeam_period', int())

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
        if self.multiplication_factor != other.multiplication_factor:
            return False
        if self.number_of_beams != other.number_of_beams:
            return False
        if self.scan_time != other.scan_time:
            return False
        if self.start_angle != other.start_angle:
            return False
        if self.angular_beam_resolution != other.angular_beam_resolution:
            return False
        if self.interbeam_period != other.interbeam_period:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def multiplication_factor(self):
        """Message field 'multiplication_factor'."""
        return self._multiplication_factor

    @multiplication_factor.setter
    def multiplication_factor(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'multiplication_factor' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'multiplication_factor' field must be an unsigned integer in [0, 65535]"
        self._multiplication_factor = value

    @builtins.property
    def number_of_beams(self):
        """Message field 'number_of_beams'."""
        return self._number_of_beams

    @number_of_beams.setter
    def number_of_beams(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'number_of_beams' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'number_of_beams' field must be an unsigned integer in [0, 65535]"
        self._number_of_beams = value

    @builtins.property
    def scan_time(self):
        """Message field 'scan_time'."""
        return self._scan_time

    @scan_time.setter
    def scan_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'scan_time' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'scan_time' field must be an unsigned integer in [0, 65535]"
        self._scan_time = value

    @builtins.property
    def start_angle(self):
        """Message field 'start_angle'."""
        return self._start_angle

    @start_angle.setter
    def start_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'start_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'start_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._start_angle = value

    @builtins.property
    def angular_beam_resolution(self):
        """Message field 'angular_beam_resolution'."""
        return self._angular_beam_resolution

    @angular_beam_resolution.setter
    def angular_beam_resolution(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_beam_resolution' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_beam_resolution' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_beam_resolution = value

    @builtins.property
    def interbeam_period(self):
        """Message field 'interbeam_period'."""
        return self._interbeam_period

    @interbeam_period.setter
    def interbeam_period(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'interbeam_period' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'interbeam_period' field must be an unsigned integer in [0, 4294967295]"
        self._interbeam_period = value
