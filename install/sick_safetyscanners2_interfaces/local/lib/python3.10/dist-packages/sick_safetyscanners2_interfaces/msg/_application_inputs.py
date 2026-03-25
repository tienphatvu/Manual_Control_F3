# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/ApplicationInputs.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'monitoring_case_number_inputs'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ApplicationInputs(type):
    """Metaclass of message 'ApplicationInputs'."""

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
                'sick_safetyscanners2_interfaces.msg.ApplicationInputs')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__application_inputs
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__application_inputs
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__application_inputs
            cls._TYPE_SUPPORT = module.type_support_msg__msg__application_inputs
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__application_inputs

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ApplicationInputs(metaclass=Metaclass_ApplicationInputs):
    """Message class 'ApplicationInputs'."""

    __slots__ = [
        '_unsafe_inputs_input_sources',
        '_unsafe_inputs_flags',
        '_monitoring_case_number_inputs',
        '_monitoring_case_number_inputs_flags',
        '_linear_velocity_inputs_velocity_0',
        '_linear_velocity_inputs_velocity_0_valid',
        '_linear_velocity_inputs_velocity_0_transmitted_safely',
        '_linear_velocity_inputs_velocity_1',
        '_linear_velocity_inputs_velocity_1_valid',
        '_linear_velocity_inputs_velocity_1_transmitted_safely',
        '_sleep_mode_input',
    ]

    _fields_and_field_types = {
        'unsafe_inputs_input_sources': 'sequence<boolean>',
        'unsafe_inputs_flags': 'sequence<boolean>',
        'monitoring_case_number_inputs': 'sequence<uint16>',
        'monitoring_case_number_inputs_flags': 'sequence<boolean>',
        'linear_velocity_inputs_velocity_0': 'int16',
        'linear_velocity_inputs_velocity_0_valid': 'boolean',
        'linear_velocity_inputs_velocity_0_transmitted_safely': 'boolean',
        'linear_velocity_inputs_velocity_1': 'int16',
        'linear_velocity_inputs_velocity_1_valid': 'boolean',
        'linear_velocity_inputs_velocity_1_transmitted_safely': 'boolean',
        'sleep_mode_input': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.unsafe_inputs_input_sources = kwargs.get('unsafe_inputs_input_sources', [])
        self.unsafe_inputs_flags = kwargs.get('unsafe_inputs_flags', [])
        self.monitoring_case_number_inputs = array.array('H', kwargs.get('monitoring_case_number_inputs', []))
        self.monitoring_case_number_inputs_flags = kwargs.get('monitoring_case_number_inputs_flags', [])
        self.linear_velocity_inputs_velocity_0 = kwargs.get('linear_velocity_inputs_velocity_0', int())
        self.linear_velocity_inputs_velocity_0_valid = kwargs.get('linear_velocity_inputs_velocity_0_valid', bool())
        self.linear_velocity_inputs_velocity_0_transmitted_safely = kwargs.get('linear_velocity_inputs_velocity_0_transmitted_safely', bool())
        self.linear_velocity_inputs_velocity_1 = kwargs.get('linear_velocity_inputs_velocity_1', int())
        self.linear_velocity_inputs_velocity_1_valid = kwargs.get('linear_velocity_inputs_velocity_1_valid', bool())
        self.linear_velocity_inputs_velocity_1_transmitted_safely = kwargs.get('linear_velocity_inputs_velocity_1_transmitted_safely', bool())
        self.sleep_mode_input = kwargs.get('sleep_mode_input', int())

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
        if self.unsafe_inputs_input_sources != other.unsafe_inputs_input_sources:
            return False
        if self.unsafe_inputs_flags != other.unsafe_inputs_flags:
            return False
        if self.monitoring_case_number_inputs != other.monitoring_case_number_inputs:
            return False
        if self.monitoring_case_number_inputs_flags != other.monitoring_case_number_inputs_flags:
            return False
        if self.linear_velocity_inputs_velocity_0 != other.linear_velocity_inputs_velocity_0:
            return False
        if self.linear_velocity_inputs_velocity_0_valid != other.linear_velocity_inputs_velocity_0_valid:
            return False
        if self.linear_velocity_inputs_velocity_0_transmitted_safely != other.linear_velocity_inputs_velocity_0_transmitted_safely:
            return False
        if self.linear_velocity_inputs_velocity_1 != other.linear_velocity_inputs_velocity_1:
            return False
        if self.linear_velocity_inputs_velocity_1_valid != other.linear_velocity_inputs_velocity_1_valid:
            return False
        if self.linear_velocity_inputs_velocity_1_transmitted_safely != other.linear_velocity_inputs_velocity_1_transmitted_safely:
            return False
        if self.sleep_mode_input != other.sleep_mode_input:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def unsafe_inputs_input_sources(self):
        """Message field 'unsafe_inputs_input_sources'."""
        return self._unsafe_inputs_input_sources

    @unsafe_inputs_input_sources.setter
    def unsafe_inputs_input_sources(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'unsafe_inputs_input_sources' field must be a set or sequence and each value of type 'bool'"
        self._unsafe_inputs_input_sources = value

    @builtins.property
    def unsafe_inputs_flags(self):
        """Message field 'unsafe_inputs_flags'."""
        return self._unsafe_inputs_flags

    @unsafe_inputs_flags.setter
    def unsafe_inputs_flags(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'unsafe_inputs_flags' field must be a set or sequence and each value of type 'bool'"
        self._unsafe_inputs_flags = value

    @builtins.property
    def monitoring_case_number_inputs(self):
        """Message field 'monitoring_case_number_inputs'."""
        return self._monitoring_case_number_inputs

    @monitoring_case_number_inputs.setter
    def monitoring_case_number_inputs(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'monitoring_case_number_inputs' array.array() must have the type code of 'H'"
            self._monitoring_case_number_inputs = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 65536 for val in value)), \
                "The 'monitoring_case_number_inputs' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._monitoring_case_number_inputs = array.array('H', value)

    @builtins.property
    def monitoring_case_number_inputs_flags(self):
        """Message field 'monitoring_case_number_inputs_flags'."""
        return self._monitoring_case_number_inputs_flags

    @monitoring_case_number_inputs_flags.setter
    def monitoring_case_number_inputs_flags(self, value):
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
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'monitoring_case_number_inputs_flags' field must be a set or sequence and each value of type 'bool'"
        self._monitoring_case_number_inputs_flags = value

    @builtins.property
    def linear_velocity_inputs_velocity_0(self):
        """Message field 'linear_velocity_inputs_velocity_0'."""
        return self._linear_velocity_inputs_velocity_0

    @linear_velocity_inputs_velocity_0.setter
    def linear_velocity_inputs_velocity_0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'linear_velocity_inputs_velocity_0' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'linear_velocity_inputs_velocity_0' field must be an integer in [-32768, 32767]"
        self._linear_velocity_inputs_velocity_0 = value

    @builtins.property
    def linear_velocity_inputs_velocity_0_valid(self):
        """Message field 'linear_velocity_inputs_velocity_0_valid'."""
        return self._linear_velocity_inputs_velocity_0_valid

    @linear_velocity_inputs_velocity_0_valid.setter
    def linear_velocity_inputs_velocity_0_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_inputs_velocity_0_valid' field must be of type 'bool'"
        self._linear_velocity_inputs_velocity_0_valid = value

    @builtins.property
    def linear_velocity_inputs_velocity_0_transmitted_safely(self):
        """Message field 'linear_velocity_inputs_velocity_0_transmitted_safely'."""
        return self._linear_velocity_inputs_velocity_0_transmitted_safely

    @linear_velocity_inputs_velocity_0_transmitted_safely.setter
    def linear_velocity_inputs_velocity_0_transmitted_safely(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_inputs_velocity_0_transmitted_safely' field must be of type 'bool'"
        self._linear_velocity_inputs_velocity_0_transmitted_safely = value

    @builtins.property
    def linear_velocity_inputs_velocity_1(self):
        """Message field 'linear_velocity_inputs_velocity_1'."""
        return self._linear_velocity_inputs_velocity_1

    @linear_velocity_inputs_velocity_1.setter
    def linear_velocity_inputs_velocity_1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'linear_velocity_inputs_velocity_1' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'linear_velocity_inputs_velocity_1' field must be an integer in [-32768, 32767]"
        self._linear_velocity_inputs_velocity_1 = value

    @builtins.property
    def linear_velocity_inputs_velocity_1_valid(self):
        """Message field 'linear_velocity_inputs_velocity_1_valid'."""
        return self._linear_velocity_inputs_velocity_1_valid

    @linear_velocity_inputs_velocity_1_valid.setter
    def linear_velocity_inputs_velocity_1_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_inputs_velocity_1_valid' field must be of type 'bool'"
        self._linear_velocity_inputs_velocity_1_valid = value

    @builtins.property
    def linear_velocity_inputs_velocity_1_transmitted_safely(self):
        """Message field 'linear_velocity_inputs_velocity_1_transmitted_safely'."""
        return self._linear_velocity_inputs_velocity_1_transmitted_safely

    @linear_velocity_inputs_velocity_1_transmitted_safely.setter
    def linear_velocity_inputs_velocity_1_transmitted_safely(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_inputs_velocity_1_transmitted_safely' field must be of type 'bool'"
        self._linear_velocity_inputs_velocity_1_transmitted_safely = value

    @builtins.property
    def sleep_mode_input(self):
        """Message field 'sleep_mode_input'."""
        return self._sleep_mode_input

    @sleep_mode_input.setter
    def sleep_mode_input(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sleep_mode_input' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sleep_mode_input' field must be an unsigned integer in [0, 255]"
        self._sleep_mode_input = value
