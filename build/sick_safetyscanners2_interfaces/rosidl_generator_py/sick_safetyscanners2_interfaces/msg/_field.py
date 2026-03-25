# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/Field.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'ranges'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Field(type):
    """Metaclass of message 'Field'."""

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
                'sick_safetyscanners2_interfaces.msg.Field')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__field
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__field
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__field
            cls._TYPE_SUPPORT = module.type_support_msg__msg__field
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__field

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Field(metaclass=Metaclass_Field):
    """Message class 'Field'."""

    __slots__ = [
        '_ranges',
        '_start_angle',
        '_angular_resolution',
        '_protective_field',
    ]

    _fields_and_field_types = {
        'ranges': 'sequence<float>',
        'start_angle': 'float',
        'angular_resolution': 'float',
        'protective_field': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ranges = array.array('f', kwargs.get('ranges', []))
        self.start_angle = kwargs.get('start_angle', float())
        self.angular_resolution = kwargs.get('angular_resolution', float())
        self.protective_field = kwargs.get('protective_field', bool())

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
        if self.ranges != other.ranges:
            return False
        if self.start_angle != other.start_angle:
            return False
        if self.angular_resolution != other.angular_resolution:
            return False
        if self.protective_field != other.protective_field:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ranges(self):
        """Message field 'ranges'."""
        return self._ranges

    @ranges.setter
    def ranges(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'ranges' array.array() must have the type code of 'f'"
            self._ranges = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'ranges' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._ranges = array.array('f', value)

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
    def angular_resolution(self):
        """Message field 'angular_resolution'."""
        return self._angular_resolution

    @angular_resolution.setter
    def angular_resolution(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'angular_resolution' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'angular_resolution' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._angular_resolution = value

    @builtins.property
    def protective_field(self):
        """Message field 'protective_field'."""
        return self._protective_field

    @protective_field.setter
    def protective_field(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'protective_field' field must be of type 'bool'"
        self._protective_field = value
