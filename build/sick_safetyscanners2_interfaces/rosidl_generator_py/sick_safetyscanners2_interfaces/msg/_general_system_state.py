# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/GeneralSystemState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GeneralSystemState(type):
    """Metaclass of message 'GeneralSystemState'."""

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
                'sick_safetyscanners2_interfaces.msg.GeneralSystemState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__general_system_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__general_system_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__general_system_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__general_system_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__general_system_state

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GeneralSystemState(metaclass=Metaclass_GeneralSystemState):
    """Message class 'GeneralSystemState'."""

    __slots__ = [
        '_run_mode_active',
        '_standby_mode_active',
        '_contamination_warning',
        '_contamination_error',
        '_reference_contour_status',
        '_manipulation_status',
        '_safe_cut_off_path',
        '_non_safe_cut_off_path',
        '_reset_required_cut_off_path',
        '_current_monitoring_case_no_table_1',
        '_current_monitoring_case_no_table_2',
        '_current_monitoring_case_no_table_3',
        '_current_monitoring_case_no_table_4',
        '_application_error',
        '_device_error',
    ]

    _fields_and_field_types = {
        'run_mode_active': 'boolean',
        'standby_mode_active': 'boolean',
        'contamination_warning': 'boolean',
        'contamination_error': 'boolean',
        'reference_contour_status': 'boolean',
        'manipulation_status': 'boolean',
        'safe_cut_off_path': 'sequence<boolean>',
        'non_safe_cut_off_path': 'sequence<boolean>',
        'reset_required_cut_off_path': 'sequence<boolean>',
        'current_monitoring_case_no_table_1': 'uint8',
        'current_monitoring_case_no_table_2': 'uint8',
        'current_monitoring_case_no_table_3': 'uint8',
        'current_monitoring_case_no_table_4': 'uint8',
        'application_error': 'boolean',
        'device_error': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.run_mode_active = kwargs.get('run_mode_active', bool())
        self.standby_mode_active = kwargs.get('standby_mode_active', bool())
        self.contamination_warning = kwargs.get('contamination_warning', bool())
        self.contamination_error = kwargs.get('contamination_error', bool())
        self.reference_contour_status = kwargs.get('reference_contour_status', bool())
        self.manipulation_status = kwargs.get('manipulation_status', bool())
        self.safe_cut_off_path = kwargs.get('safe_cut_off_path', [])
        self.non_safe_cut_off_path = kwargs.get('non_safe_cut_off_path', [])
        self.reset_required_cut_off_path = kwargs.get('reset_required_cut_off_path', [])
        self.current_monitoring_case_no_table_1 = kwargs.get('current_monitoring_case_no_table_1', int())
        self.current_monitoring_case_no_table_2 = kwargs.get('current_monitoring_case_no_table_2', int())
        self.current_monitoring_case_no_table_3 = kwargs.get('current_monitoring_case_no_table_3', int())
        self.current_monitoring_case_no_table_4 = kwargs.get('current_monitoring_case_no_table_4', int())
        self.application_error = kwargs.get('application_error', bool())
        self.device_error = kwargs.get('device_error', bool())

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
        if self.run_mode_active != other.run_mode_active:
            return False
        if self.standby_mode_active != other.standby_mode_active:
            return False
        if self.contamination_warning != other.contamination_warning:
            return False
        if self.contamination_error != other.contamination_error:
            return False
        if self.reference_contour_status != other.reference_contour_status:
            return False
        if self.manipulation_status != other.manipulation_status:
            return False
        if self.safe_cut_off_path != other.safe_cut_off_path:
            return False
        if self.non_safe_cut_off_path != other.non_safe_cut_off_path:
            return False
        if self.reset_required_cut_off_path != other.reset_required_cut_off_path:
            return False
        if self.current_monitoring_case_no_table_1 != other.current_monitoring_case_no_table_1:
            return False
        if self.current_monitoring_case_no_table_2 != other.current_monitoring_case_no_table_2:
            return False
        if self.current_monitoring_case_no_table_3 != other.current_monitoring_case_no_table_3:
            return False
        if self.current_monitoring_case_no_table_4 != other.current_monitoring_case_no_table_4:
            return False
        if self.application_error != other.application_error:
            return False
        if self.device_error != other.device_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def run_mode_active(self):
        """Message field 'run_mode_active'."""
        return self._run_mode_active

    @run_mode_active.setter
    def run_mode_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'run_mode_active' field must be of type 'bool'"
        self._run_mode_active = value

    @builtins.property
    def standby_mode_active(self):
        """Message field 'standby_mode_active'."""
        return self._standby_mode_active

    @standby_mode_active.setter
    def standby_mode_active(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'standby_mode_active' field must be of type 'bool'"
        self._standby_mode_active = value

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

    @builtins.property
    def contamination_error(self):
        """Message field 'contamination_error'."""
        return self._contamination_error

    @contamination_error.setter
    def contamination_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'contamination_error' field must be of type 'bool'"
        self._contamination_error = value

    @builtins.property
    def reference_contour_status(self):
        """Message field 'reference_contour_status'."""
        return self._reference_contour_status

    @reference_contour_status.setter
    def reference_contour_status(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reference_contour_status' field must be of type 'bool'"
        self._reference_contour_status = value

    @builtins.property
    def manipulation_status(self):
        """Message field 'manipulation_status'."""
        return self._manipulation_status

    @manipulation_status.setter
    def manipulation_status(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'manipulation_status' field must be of type 'bool'"
        self._manipulation_status = value

    @builtins.property
    def safe_cut_off_path(self):
        """Message field 'safe_cut_off_path'."""
        return self._safe_cut_off_path

    @safe_cut_off_path.setter
    def safe_cut_off_path(self, value):
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
                "The 'safe_cut_off_path' field must be a set or sequence and each value of type 'bool'"
        self._safe_cut_off_path = value

    @builtins.property
    def non_safe_cut_off_path(self):
        """Message field 'non_safe_cut_off_path'."""
        return self._non_safe_cut_off_path

    @non_safe_cut_off_path.setter
    def non_safe_cut_off_path(self, value):
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
                "The 'non_safe_cut_off_path' field must be a set or sequence and each value of type 'bool'"
        self._non_safe_cut_off_path = value

    @builtins.property
    def reset_required_cut_off_path(self):
        """Message field 'reset_required_cut_off_path'."""
        return self._reset_required_cut_off_path

    @reset_required_cut_off_path.setter
    def reset_required_cut_off_path(self, value):
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
                "The 'reset_required_cut_off_path' field must be a set or sequence and each value of type 'bool'"
        self._reset_required_cut_off_path = value

    @builtins.property
    def current_monitoring_case_no_table_1(self):
        """Message field 'current_monitoring_case_no_table_1'."""
        return self._current_monitoring_case_no_table_1

    @current_monitoring_case_no_table_1.setter
    def current_monitoring_case_no_table_1(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_monitoring_case_no_table_1' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_monitoring_case_no_table_1' field must be an unsigned integer in [0, 255]"
        self._current_monitoring_case_no_table_1 = value

    @builtins.property
    def current_monitoring_case_no_table_2(self):
        """Message field 'current_monitoring_case_no_table_2'."""
        return self._current_monitoring_case_no_table_2

    @current_monitoring_case_no_table_2.setter
    def current_monitoring_case_no_table_2(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_monitoring_case_no_table_2' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_monitoring_case_no_table_2' field must be an unsigned integer in [0, 255]"
        self._current_monitoring_case_no_table_2 = value

    @builtins.property
    def current_monitoring_case_no_table_3(self):
        """Message field 'current_monitoring_case_no_table_3'."""
        return self._current_monitoring_case_no_table_3

    @current_monitoring_case_no_table_3.setter
    def current_monitoring_case_no_table_3(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_monitoring_case_no_table_3' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_monitoring_case_no_table_3' field must be an unsigned integer in [0, 255]"
        self._current_monitoring_case_no_table_3 = value

    @builtins.property
    def current_monitoring_case_no_table_4(self):
        """Message field 'current_monitoring_case_no_table_4'."""
        return self._current_monitoring_case_no_table_4

    @current_monitoring_case_no_table_4.setter
    def current_monitoring_case_no_table_4(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_monitoring_case_no_table_4' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'current_monitoring_case_no_table_4' field must be an unsigned integer in [0, 255]"
        self._current_monitoring_case_no_table_4 = value

    @builtins.property
    def application_error(self):
        """Message field 'application_error'."""
        return self._application_error

    @application_error.setter
    def application_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'application_error' field must be of type 'bool'"
        self._application_error = value

    @builtins.property
    def device_error(self):
        """Message field 'device_error'."""
        return self._device_error

    @device_error.setter
    def device_error(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'device_error' field must be of type 'bool'"
        self._device_error = value
