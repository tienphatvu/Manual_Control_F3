# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/DataHeader.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DataHeader(type):
    """Metaclass of message 'DataHeader'."""

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
                'sick_safetyscanners2_interfaces.msg.DataHeader')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__data_header
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__data_header
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__data_header
            cls._TYPE_SUPPORT = module.type_support_msg__msg__data_header
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__data_header

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DataHeader(metaclass=Metaclass_DataHeader):
    """Message class 'DataHeader'."""

    __slots__ = [
        '_version_version',
        '_version_major_version',
        '_version_minor_version',
        '_version_release',
        '_serial_number_of_device',
        '_serial_number_of_channel_plug',
        '_channel_number',
        '_sequence_number',
        '_scan_number',
        '_timestamp_date',
        '_timestamp_time',
    ]

    _fields_and_field_types = {
        'version_version': 'uint8',
        'version_major_version': 'uint8',
        'version_minor_version': 'uint8',
        'version_release': 'uint8',
        'serial_number_of_device': 'uint32',
        'serial_number_of_channel_plug': 'uint32',
        'channel_number': 'uint8',
        'sequence_number': 'uint32',
        'scan_number': 'uint32',
        'timestamp_date': 'uint16',
        'timestamp_time': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.version_version = kwargs.get('version_version', int())
        self.version_major_version = kwargs.get('version_major_version', int())
        self.version_minor_version = kwargs.get('version_minor_version', int())
        self.version_release = kwargs.get('version_release', int())
        self.serial_number_of_device = kwargs.get('serial_number_of_device', int())
        self.serial_number_of_channel_plug = kwargs.get('serial_number_of_channel_plug', int())
        self.channel_number = kwargs.get('channel_number', int())
        self.sequence_number = kwargs.get('sequence_number', int())
        self.scan_number = kwargs.get('scan_number', int())
        self.timestamp_date = kwargs.get('timestamp_date', int())
        self.timestamp_time = kwargs.get('timestamp_time', int())

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
        if self.version_version != other.version_version:
            return False
        if self.version_major_version != other.version_major_version:
            return False
        if self.version_minor_version != other.version_minor_version:
            return False
        if self.version_release != other.version_release:
            return False
        if self.serial_number_of_device != other.serial_number_of_device:
            return False
        if self.serial_number_of_channel_plug != other.serial_number_of_channel_plug:
            return False
        if self.channel_number != other.channel_number:
            return False
        if self.sequence_number != other.sequence_number:
            return False
        if self.scan_number != other.scan_number:
            return False
        if self.timestamp_date != other.timestamp_date:
            return False
        if self.timestamp_time != other.timestamp_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def version_version(self):
        """Message field 'version_version'."""
        return self._version_version

    @version_version.setter
    def version_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_version' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_version' field must be an unsigned integer in [0, 255]"
        self._version_version = value

    @builtins.property
    def version_major_version(self):
        """Message field 'version_major_version'."""
        return self._version_major_version

    @version_major_version.setter
    def version_major_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_major_version' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_major_version' field must be an unsigned integer in [0, 255]"
        self._version_major_version = value

    @builtins.property
    def version_minor_version(self):
        """Message field 'version_minor_version'."""
        return self._version_minor_version

    @version_minor_version.setter
    def version_minor_version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_minor_version' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_minor_version' field must be an unsigned integer in [0, 255]"
        self._version_minor_version = value

    @builtins.property
    def version_release(self):
        """Message field 'version_release'."""
        return self._version_release

    @version_release.setter
    def version_release(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version_release' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version_release' field must be an unsigned integer in [0, 255]"
        self._version_release = value

    @builtins.property
    def serial_number_of_device(self):
        """Message field 'serial_number_of_device'."""
        return self._serial_number_of_device

    @serial_number_of_device.setter
    def serial_number_of_device(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'serial_number_of_device' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'serial_number_of_device' field must be an unsigned integer in [0, 4294967295]"
        self._serial_number_of_device = value

    @builtins.property
    def serial_number_of_channel_plug(self):
        """Message field 'serial_number_of_channel_plug'."""
        return self._serial_number_of_channel_plug

    @serial_number_of_channel_plug.setter
    def serial_number_of_channel_plug(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'serial_number_of_channel_plug' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'serial_number_of_channel_plug' field must be an unsigned integer in [0, 4294967295]"
        self._serial_number_of_channel_plug = value

    @builtins.property
    def channel_number(self):
        """Message field 'channel_number'."""
        return self._channel_number

    @channel_number.setter
    def channel_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'channel_number' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'channel_number' field must be an unsigned integer in [0, 255]"
        self._channel_number = value

    @builtins.property
    def sequence_number(self):
        """Message field 'sequence_number'."""
        return self._sequence_number

    @sequence_number.setter
    def sequence_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sequence_number' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'sequence_number' field must be an unsigned integer in [0, 4294967295]"
        self._sequence_number = value

    @builtins.property
    def scan_number(self):
        """Message field 'scan_number'."""
        return self._scan_number

    @scan_number.setter
    def scan_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'scan_number' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'scan_number' field must be an unsigned integer in [0, 4294967295]"
        self._scan_number = value

    @builtins.property
    def timestamp_date(self):
        """Message field 'timestamp_date'."""
        return self._timestamp_date

    @timestamp_date.setter
    def timestamp_date(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp_date' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'timestamp_date' field must be an unsigned integer in [0, 65535]"
        self._timestamp_date = value

    @builtins.property
    def timestamp_time(self):
        """Message field 'timestamp_time'."""
        return self._timestamp_time

    @timestamp_time.setter
    def timestamp_time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'timestamp_time' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'timestamp_time' field must be an unsigned integer in [0, 4294967295]"
        self._timestamp_time = value
