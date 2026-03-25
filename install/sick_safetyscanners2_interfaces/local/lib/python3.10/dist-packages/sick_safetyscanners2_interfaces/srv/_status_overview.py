# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:srv/StatusOverview.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StatusOverview_Request(type):
    """Metaclass of message 'StatusOverview_Request'."""

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
                'sick_safetyscanners2_interfaces.srv.StatusOverview_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__status_overview__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__status_overview__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__status_overview__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__status_overview__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__status_overview__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StatusOverview_Request(metaclass=Metaclass_StatusOverview_Request):
    """Message class 'StatusOverview_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_StatusOverview_Response(type):
    """Metaclass of message 'StatusOverview_Response'."""

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
                'sick_safetyscanners2_interfaces.srv.StatusOverview_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__status_overview__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__status_overview__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__status_overview__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__status_overview__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__status_overview__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StatusOverview_Response(metaclass=Metaclass_StatusOverview_Response):
    """Message class 'StatusOverview_Response'."""

    __slots__ = [
        '_version_c_version',
        '_version_major_version_number',
        '_version_minor_version_number',
        '_version_release_number',
        '_device_state',
        '_config_state',
        '_application_state',
        '_current_time_power_on_count',
        '_current_time',
        '_current_time_time',
        '_current_time_date',
        '_error_info_code',
        '_error_info_time',
        '_error_info_time_time',
        '_error_info_time_date',
    ]

    _fields_and_field_types = {
        'version_c_version': 'string',
        'version_major_version_number': 'uint8',
        'version_minor_version_number': 'uint8',
        'version_release_number': 'uint8',
        'device_state': 'uint8',
        'config_state': 'uint8',
        'application_state': 'uint8',
        'current_time_power_on_count': 'uint32',
        'current_time': 'string',
        'current_time_time': 'uint32',
        'current_time_date': 'uint16',
        'error_info_code': 'uint32',
        'error_info_time': 'string',
        'error_info_time_time': 'uint32',
        'error_info_time_date': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.version_c_version = kwargs.get('version_c_version', str())
        self.version_major_version_number = kwargs.get('version_major_version_number', int())
        self.version_minor_version_number = kwargs.get('version_minor_version_number', int())
        self.version_release_number = kwargs.get('version_release_number', int())
        self.device_state = kwargs.get('device_state', int())
        self.config_state = kwargs.get('config_state', int())
        self.application_state = kwargs.get('application_state', int())
        self.current_time_power_on_count = kwargs.get('current_time_power_on_count', int())
        self.current_time = kwargs.get('current_time', str())
        self.current_time_time = kwargs.get('current_time_time', int())
        self.current_time_date = kwargs.get('current_time_date', int())
        self.error_info_code = kwargs.get('error_info_code', int())
        self.error_info_time = kwargs.get('error_info_time', str())
        self.error_info_time_time = kwargs.get('error_info_time_time', int())
        self.error_info_time_date = kwargs.get('error_info_time_date', int())

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
        if self.version_c_version != other.version_c_version:
            return False
        if self.version_major_version_number != other.version_major_version_number:
            return False
        if self.version_minor_version_number != other.version_minor_version_number:
            return False
        if self.version_release_number != other.version_release_number:
            return False
        if self.device_state != other.device_state:
            return False
        if self.config_state != other.config_state:
            return False
        if self.application_state != other.application_state:
            return False
        if self.current_time_power_on_count != other.current_time_power_on_count:
            return False
        if self.current_time != other.current_time:
            return False
        if self.current_time_time != other.current_time_time:
            return False
        if self.current_time_date != other.current_time_date:
            return False
        if self.error_info_code != other.error_info_code:
            return False
        if self.error_info_time != other.error_info_time:
            return False
        if self.error_info_time_time != other.error_info_time_time:
            return False
        if self.error_info_time_date != other.error_info_time_date:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def version_c_version(self):
        """Message field 'version_c_version'."""
        return self._version_c_version

    @version_c_version.setter
    def version_c_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'version_c_version' field must be of type 'str'"
        self._version_c_version = value

    @builtins.property
    def version_major_version_number(self):
        """Message field 'version_major_version_number'."""
        return self._version_major_version_number

    @version_major_version_number.setter
    def version_major_version_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_major_version_number' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_major_version_number' field must be an unsigned integer in [0, 255]"
        self._version_major_version_number = value

    @builtins.property
    def version_minor_version_number(self):
        """Message field 'version_minor_version_number'."""
        return self._version_minor_version_number

    @version_minor_version_number.setter
    def version_minor_version_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_minor_version_number' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_minor_version_number' field must be an unsigned integer in [0, 255]"
        self._version_minor_version_number = value

    @builtins.property
    def version_release_number(self):
        """Message field 'version_release_number'."""
        return self._version_release_number

    @version_release_number.setter
    def version_release_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_release_number' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_release_number' field must be an unsigned integer in [0, 255]"
        self._version_release_number = value

    @builtins.property
    def device_state(self):
        """Message field 'device_state'."""
        return self._device_state

    @device_state.setter
    def device_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'device_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'device_state' field must be an unsigned integer in [0, 255]"
        self._device_state = value

    @builtins.property
    def config_state(self):
        """Message field 'config_state'."""
        return self._config_state

    @config_state.setter
    def config_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'config_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'config_state' field must be an unsigned integer in [0, 255]"
        self._config_state = value

    @builtins.property
    def application_state(self):
        """Message field 'application_state'."""
        return self._application_state

    @application_state.setter
    def application_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'application_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'application_state' field must be an unsigned integer in [0, 255]"
        self._application_state = value

    @builtins.property
    def current_time_power_on_count(self):
        """Message field 'current_time_power_on_count'."""
        return self._current_time_power_on_count

    @current_time_power_on_count.setter
    def current_time_power_on_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_time_power_on_count' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'current_time_power_on_count' field must be an unsigned integer in [0, 4294967295]"
        self._current_time_power_on_count = value

    @builtins.property
    def current_time(self):
        """Message field 'current_time'."""
        return self._current_time

    @current_time.setter
    def current_time(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'current_time' field must be of type 'str'"
        self._current_time = value

    @builtins.property
    def current_time_time(self):
        """Message field 'current_time_time'."""
        return self._current_time_time

    @current_time_time.setter
    def current_time_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_time_time' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'current_time_time' field must be an unsigned integer in [0, 4294967295]"
        self._current_time_time = value

    @builtins.property
    def current_time_date(self):
        """Message field 'current_time_date'."""
        return self._current_time_date

    @current_time_date.setter
    def current_time_date(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'current_time_date' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'current_time_date' field must be an unsigned integer in [0, 65535]"
        self._current_time_date = value

    @builtins.property
    def error_info_code(self):
        """Message field 'error_info_code'."""
        return self._error_info_code

    @error_info_code.setter
    def error_info_code(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_info_code' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'error_info_code' field must be an unsigned integer in [0, 4294967295]"
        self._error_info_code = value

    @builtins.property
    def error_info_time(self):
        """Message field 'error_info_time'."""
        return self._error_info_time

    @error_info_time.setter
    def error_info_time(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_info_time' field must be of type 'str'"
        self._error_info_time = value

    @builtins.property
    def error_info_time_time(self):
        """Message field 'error_info_time_time'."""
        return self._error_info_time_time

    @error_info_time_time.setter
    def error_info_time_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_info_time_time' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'error_info_time_time' field must be an unsigned integer in [0, 4294967295]"
        self._error_info_time_time = value

    @builtins.property
    def error_info_time_date(self):
        """Message field 'error_info_time_date'."""
        return self._error_info_time_date

    @error_info_time_date.setter
    def error_info_time_date(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'error_info_time_date' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'error_info_time_date' field must be an unsigned integer in [0, 65535]"
        self._error_info_time_date = value


class Metaclass_StatusOverview(type):
    """Metaclass of service 'StatusOverview'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('sick_safetyscanners2_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'sick_safetyscanners2_interfaces.srv.StatusOverview')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__status_overview

            from sick_safetyscanners2_interfaces.srv import _status_overview
            if _status_overview.Metaclass_StatusOverview_Request._TYPE_SUPPORT is None:
                _status_overview.Metaclass_StatusOverview_Request.__import_type_support__()
            if _status_overview.Metaclass_StatusOverview_Response._TYPE_SUPPORT is None:
                _status_overview.Metaclass_StatusOverview_Response.__import_type_support__()


class StatusOverview(metaclass=Metaclass_StatusOverview):
    from sick_safetyscanners2_interfaces.srv._status_overview import StatusOverview_Request as Request
    from sick_safetyscanners2_interfaces.srv._status_overview import StatusOverview_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
