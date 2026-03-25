# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/ApplicationOutputs.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'monitoring_case_number_outputs'
# Member 'resulting_velocity'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ApplicationOutputs(type):
    """Metaclass of message 'ApplicationOutputs'."""

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
                'sick_safetyscanners2_interfaces.msg.ApplicationOutputs')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__application_outputs
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__application_outputs
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__application_outputs
            cls._TYPE_SUPPORT = module.type_support_msg__msg__application_outputs
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__application_outputs

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ApplicationOutputs(metaclass=Metaclass_ApplicationOutputs):
    """Message class 'ApplicationOutputs'."""

    __slots__ = [
        '_evaluation_path_outputs_eval_out',
        '_evaluation_path_outputs_is_safe',
        '_evaluation_path_outputs_is_valid',
        '_monitoring_case_number_outputs',
        '_monitoring_case_number_outputs_flags',
        '_sleep_mode_output',
        '_sleep_mode_output_valid',
        '_error_flag_contamination_warning',
        '_error_flag_contamination_error',
        '_error_flag_manipulation_error',
        '_error_flag_glare',
        '_error_flag_reference_contour_intruded',
        '_error_flag_critical_error',
        '_error_flags_are_valid',
        '_linear_velocity_outputs_velocity_0',
        '_linear_velocity_outputs_velocity_0_valid',
        '_linear_velocity_outputs_velocity_0_transmitted_safely',
        '_linear_velocity_outputs_velocity_1',
        '_linear_velocity_outputs_velocity_1_valid',
        '_linear_velocity_outputs_velocity_1_transmitted_safely',
        '_resulting_velocity',
        '_resulting_velocity_flags',
    ]

    _fields_and_field_types = {
        'evaluation_path_outputs_eval_out': 'sequence<boolean>',
        'evaluation_path_outputs_is_safe': 'sequence<boolean>',
        'evaluation_path_outputs_is_valid': 'sequence<boolean>',
        'monitoring_case_number_outputs': 'sequence<uint16>',
        'monitoring_case_number_outputs_flags': 'sequence<boolean>',
        'sleep_mode_output': 'uint8',
        'sleep_mode_output_valid': 'boolean',
        'error_flag_contamination_warning': 'boolean',
        'error_flag_contamination_error': 'boolean',
        'error_flag_manipulation_error': 'boolean',
        'error_flag_glare': 'boolean',
        'error_flag_reference_contour_intruded': 'boolean',
        'error_flag_critical_error': 'boolean',
        'error_flags_are_valid': 'boolean',
        'linear_velocity_outputs_velocity_0': 'int16',
        'linear_velocity_outputs_velocity_0_valid': 'boolean',
        'linear_velocity_outputs_velocity_0_transmitted_safely': 'boolean',
        'linear_velocity_outputs_velocity_1': 'int16',
        'linear_velocity_outputs_velocity_1_valid': 'boolean',
        'linear_velocity_outputs_velocity_1_transmitted_safely': 'boolean',
        'resulting_velocity': 'sequence<int16>',
        'resulting_velocity_flags': 'sequence<boolean>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int16')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.evaluation_path_outputs_eval_out = kwargs.get('evaluation_path_outputs_eval_out', [])
        self.evaluation_path_outputs_is_safe = kwargs.get('evaluation_path_outputs_is_safe', [])
        self.evaluation_path_outputs_is_valid = kwargs.get('evaluation_path_outputs_is_valid', [])
        self.monitoring_case_number_outputs = array.array('H', kwargs.get('monitoring_case_number_outputs', []))
        self.monitoring_case_number_outputs_flags = kwargs.get('monitoring_case_number_outputs_flags', [])
        self.sleep_mode_output = kwargs.get('sleep_mode_output', int())
        self.sleep_mode_output_valid = kwargs.get('sleep_mode_output_valid', bool())
        self.error_flag_contamination_warning = kwargs.get('error_flag_contamination_warning', bool())
        self.error_flag_contamination_error = kwargs.get('error_flag_contamination_error', bool())
        self.error_flag_manipulation_error = kwargs.get('error_flag_manipulation_error', bool())
        self.error_flag_glare = kwargs.get('error_flag_glare', bool())
        self.error_flag_reference_contour_intruded = kwargs.get('error_flag_reference_contour_intruded', bool())
        self.error_flag_critical_error = kwargs.get('error_flag_critical_error', bool())
        self.error_flags_are_valid = kwargs.get('error_flags_are_valid', bool())
        self.linear_velocity_outputs_velocity_0 = kwargs.get('linear_velocity_outputs_velocity_0', int())
        self.linear_velocity_outputs_velocity_0_valid = kwargs.get('linear_velocity_outputs_velocity_0_valid', bool())
        self.linear_velocity_outputs_velocity_0_transmitted_safely = kwargs.get('linear_velocity_outputs_velocity_0_transmitted_safely', bool())
        self.linear_velocity_outputs_velocity_1 = kwargs.get('linear_velocity_outputs_velocity_1', int())
        self.linear_velocity_outputs_velocity_1_valid = kwargs.get('linear_velocity_outputs_velocity_1_valid', bool())
        self.linear_velocity_outputs_velocity_1_transmitted_safely = kwargs.get('linear_velocity_outputs_velocity_1_transmitted_safely', bool())
        self.resulting_velocity = array.array('h', kwargs.get('resulting_velocity', []))
        self.resulting_velocity_flags = kwargs.get('resulting_velocity_flags', [])

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
        if self.evaluation_path_outputs_eval_out != other.evaluation_path_outputs_eval_out:
            return False
        if self.evaluation_path_outputs_is_safe != other.evaluation_path_outputs_is_safe:
            return False
        if self.evaluation_path_outputs_is_valid != other.evaluation_path_outputs_is_valid:
            return False
        if self.monitoring_case_number_outputs != other.monitoring_case_number_outputs:
            return False
        if self.monitoring_case_number_outputs_flags != other.monitoring_case_number_outputs_flags:
            return False
        if self.sleep_mode_output != other.sleep_mode_output:
            return False
        if self.sleep_mode_output_valid != other.sleep_mode_output_valid:
            return False
        if self.error_flag_contamination_warning != other.error_flag_contamination_warning:
            return False
        if self.error_flag_contamination_error != other.error_flag_contamination_error:
            return False
        if self.error_flag_manipulation_error != other.error_flag_manipulation_error:
            return False
        if self.error_flag_glare != other.error_flag_glare:
            return False
        if self.error_flag_reference_contour_intruded != other.error_flag_reference_contour_intruded:
            return False
        if self.error_flag_critical_error != other.error_flag_critical_error:
            return False
        if self.error_flags_are_valid != other.error_flags_are_valid:
            return False
        if self.linear_velocity_outputs_velocity_0 != other.linear_velocity_outputs_velocity_0:
            return False
        if self.linear_velocity_outputs_velocity_0_valid != other.linear_velocity_outputs_velocity_0_valid:
            return False
        if self.linear_velocity_outputs_velocity_0_transmitted_safely != other.linear_velocity_outputs_velocity_0_transmitted_safely:
            return False
        if self.linear_velocity_outputs_velocity_1 != other.linear_velocity_outputs_velocity_1:
            return False
        if self.linear_velocity_outputs_velocity_1_valid != other.linear_velocity_outputs_velocity_1_valid:
            return False
        if self.linear_velocity_outputs_velocity_1_transmitted_safely != other.linear_velocity_outputs_velocity_1_transmitted_safely:
            return False
        if self.resulting_velocity != other.resulting_velocity:
            return False
        if self.resulting_velocity_flags != other.resulting_velocity_flags:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def evaluation_path_outputs_eval_out(self):
        """Message field 'evaluation_path_outputs_eval_out'."""
        return self._evaluation_path_outputs_eval_out

    @evaluation_path_outputs_eval_out.setter
    def evaluation_path_outputs_eval_out(self, value):
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
                "The 'evaluation_path_outputs_eval_out' field must be a set or sequence and each value of type 'bool'"
        self._evaluation_path_outputs_eval_out = value

    @builtins.property
    def evaluation_path_outputs_is_safe(self):
        """Message field 'evaluation_path_outputs_is_safe'."""
        return self._evaluation_path_outputs_is_safe

    @evaluation_path_outputs_is_safe.setter
    def evaluation_path_outputs_is_safe(self, value):
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
                "The 'evaluation_path_outputs_is_safe' field must be a set or sequence and each value of type 'bool'"
        self._evaluation_path_outputs_is_safe = value

    @builtins.property
    def evaluation_path_outputs_is_valid(self):
        """Message field 'evaluation_path_outputs_is_valid'."""
        return self._evaluation_path_outputs_is_valid

    @evaluation_path_outputs_is_valid.setter
    def evaluation_path_outputs_is_valid(self, value):
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
                "The 'evaluation_path_outputs_is_valid' field must be a set or sequence and each value of type 'bool'"
        self._evaluation_path_outputs_is_valid = value

    @builtins.property
    def monitoring_case_number_outputs(self):
        """Message field 'monitoring_case_number_outputs'."""
        return self._monitoring_case_number_outputs

    @monitoring_case_number_outputs.setter
    def monitoring_case_number_outputs(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'H', \
                "The 'monitoring_case_number_outputs' array.array() must have the type code of 'H'"
            self._monitoring_case_number_outputs = value
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
                "The 'monitoring_case_number_outputs' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 65535]"
        self._monitoring_case_number_outputs = array.array('H', value)

    @builtins.property
    def monitoring_case_number_outputs_flags(self):
        """Message field 'monitoring_case_number_outputs_flags'."""
        return self._monitoring_case_number_outputs_flags

    @monitoring_case_number_outputs_flags.setter
    def monitoring_case_number_outputs_flags(self, value):
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
                "The 'monitoring_case_number_outputs_flags' field must be a set or sequence and each value of type 'bool'"
        self._monitoring_case_number_outputs_flags = value

    @builtins.property
    def sleep_mode_output(self):
        """Message field 'sleep_mode_output'."""
        return self._sleep_mode_output

    @sleep_mode_output.setter
    def sleep_mode_output(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sleep_mode_output' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sleep_mode_output' field must be an unsigned integer in [0, 255]"
        self._sleep_mode_output = value

    @builtins.property
    def sleep_mode_output_valid(self):
        """Message field 'sleep_mode_output_valid'."""
        return self._sleep_mode_output_valid

    @sleep_mode_output_valid.setter
    def sleep_mode_output_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sleep_mode_output_valid' field must be of type 'bool'"
        self._sleep_mode_output_valid = value

    @builtins.property
    def error_flag_contamination_warning(self):
        """Message field 'error_flag_contamination_warning'."""
        return self._error_flag_contamination_warning

    @error_flag_contamination_warning.setter
    def error_flag_contamination_warning(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flag_contamination_warning' field must be of type 'bool'"
        self._error_flag_contamination_warning = value

    @builtins.property
    def error_flag_contamination_error(self):
        """Message field 'error_flag_contamination_error'."""
        return self._error_flag_contamination_error

    @error_flag_contamination_error.setter
    def error_flag_contamination_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flag_contamination_error' field must be of type 'bool'"
        self._error_flag_contamination_error = value

    @builtins.property
    def error_flag_manipulation_error(self):
        """Message field 'error_flag_manipulation_error'."""
        return self._error_flag_manipulation_error

    @error_flag_manipulation_error.setter
    def error_flag_manipulation_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flag_manipulation_error' field must be of type 'bool'"
        self._error_flag_manipulation_error = value

    @builtins.property
    def error_flag_glare(self):
        """Message field 'error_flag_glare'."""
        return self._error_flag_glare

    @error_flag_glare.setter
    def error_flag_glare(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flag_glare' field must be of type 'bool'"
        self._error_flag_glare = value

    @builtins.property
    def error_flag_reference_contour_intruded(self):
        """Message field 'error_flag_reference_contour_intruded'."""
        return self._error_flag_reference_contour_intruded

    @error_flag_reference_contour_intruded.setter
    def error_flag_reference_contour_intruded(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flag_reference_contour_intruded' field must be of type 'bool'"
        self._error_flag_reference_contour_intruded = value

    @builtins.property
    def error_flag_critical_error(self):
        """Message field 'error_flag_critical_error'."""
        return self._error_flag_critical_error

    @error_flag_critical_error.setter
    def error_flag_critical_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flag_critical_error' field must be of type 'bool'"
        self._error_flag_critical_error = value

    @builtins.property
    def error_flags_are_valid(self):
        """Message field 'error_flags_are_valid'."""
        return self._error_flags_are_valid

    @error_flags_are_valid.setter
    def error_flags_are_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'error_flags_are_valid' field must be of type 'bool'"
        self._error_flags_are_valid = value

    @builtins.property
    def linear_velocity_outputs_velocity_0(self):
        """Message field 'linear_velocity_outputs_velocity_0'."""
        return self._linear_velocity_outputs_velocity_0

    @linear_velocity_outputs_velocity_0.setter
    def linear_velocity_outputs_velocity_0(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'linear_velocity_outputs_velocity_0' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'linear_velocity_outputs_velocity_0' field must be an integer in [-32768, 32767]"
        self._linear_velocity_outputs_velocity_0 = value

    @builtins.property
    def linear_velocity_outputs_velocity_0_valid(self):
        """Message field 'linear_velocity_outputs_velocity_0_valid'."""
        return self._linear_velocity_outputs_velocity_0_valid

    @linear_velocity_outputs_velocity_0_valid.setter
    def linear_velocity_outputs_velocity_0_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_outputs_velocity_0_valid' field must be of type 'bool'"
        self._linear_velocity_outputs_velocity_0_valid = value

    @builtins.property
    def linear_velocity_outputs_velocity_0_transmitted_safely(self):
        """Message field 'linear_velocity_outputs_velocity_0_transmitted_safely'."""
        return self._linear_velocity_outputs_velocity_0_transmitted_safely

    @linear_velocity_outputs_velocity_0_transmitted_safely.setter
    def linear_velocity_outputs_velocity_0_transmitted_safely(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_outputs_velocity_0_transmitted_safely' field must be of type 'bool'"
        self._linear_velocity_outputs_velocity_0_transmitted_safely = value

    @builtins.property
    def linear_velocity_outputs_velocity_1(self):
        """Message field 'linear_velocity_outputs_velocity_1'."""
        return self._linear_velocity_outputs_velocity_1

    @linear_velocity_outputs_velocity_1.setter
    def linear_velocity_outputs_velocity_1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'linear_velocity_outputs_velocity_1' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'linear_velocity_outputs_velocity_1' field must be an integer in [-32768, 32767]"
        self._linear_velocity_outputs_velocity_1 = value

    @builtins.property
    def linear_velocity_outputs_velocity_1_valid(self):
        """Message field 'linear_velocity_outputs_velocity_1_valid'."""
        return self._linear_velocity_outputs_velocity_1_valid

    @linear_velocity_outputs_velocity_1_valid.setter
    def linear_velocity_outputs_velocity_1_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_outputs_velocity_1_valid' field must be of type 'bool'"
        self._linear_velocity_outputs_velocity_1_valid = value

    @builtins.property
    def linear_velocity_outputs_velocity_1_transmitted_safely(self):
        """Message field 'linear_velocity_outputs_velocity_1_transmitted_safely'."""
        return self._linear_velocity_outputs_velocity_1_transmitted_safely

    @linear_velocity_outputs_velocity_1_transmitted_safely.setter
    def linear_velocity_outputs_velocity_1_transmitted_safely(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'linear_velocity_outputs_velocity_1_transmitted_safely' field must be of type 'bool'"
        self._linear_velocity_outputs_velocity_1_transmitted_safely = value

    @builtins.property
    def resulting_velocity(self):
        """Message field 'resulting_velocity'."""
        return self._resulting_velocity

    @resulting_velocity.setter
    def resulting_velocity(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'h', \
                "The 'resulting_velocity' array.array() must have the type code of 'h'"
            self._resulting_velocity = value
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
                 all(val >= -32768 and val < 32768 for val in value)), \
                "The 'resulting_velocity' field must be a set or sequence and each value of type 'int' and each integer in [-32768, 32767]"
        self._resulting_velocity = array.array('h', value)

    @builtins.property
    def resulting_velocity_flags(self):
        """Message field 'resulting_velocity_flags'."""
        return self._resulting_velocity_flags

    @resulting_velocity_flags.setter
    def resulting_velocity_flags(self, value):
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
                "The 'resulting_velocity_flags' field must be a set or sequence and each value of type 'bool'"
        self._resulting_velocity_flags = value
