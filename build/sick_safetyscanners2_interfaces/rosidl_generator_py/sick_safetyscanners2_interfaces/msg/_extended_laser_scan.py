# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/ExtendedLaserScan.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExtendedLaserScan(type):
    """Metaclass of message 'ExtendedLaserScan'."""

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
                'sick_safetyscanners2_interfaces.msg.ExtendedLaserScan')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__extended_laser_scan
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__extended_laser_scan
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__extended_laser_scan
            cls._TYPE_SUPPORT = module.type_support_msg__msg__extended_laser_scan
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__extended_laser_scan

            from sensor_msgs.msg import LaserScan
            if LaserScan.__class__._TYPE_SUPPORT is None:
                LaserScan.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExtendedLaserScan(metaclass=Metaclass_ExtendedLaserScan):
    """Message class 'ExtendedLaserScan'."""

    __slots__ = [
        '_laser_scan',
        '_reflektor_status',
        '_reflektor_median',
        '_intrusion',
    ]

    _fields_and_field_types = {
        'laser_scan': 'sensor_msgs/LaserScan',
        'reflektor_status': 'sequence<boolean>',
        'reflektor_median': 'sequence<boolean>',
        'intrusion': 'sequence<boolean>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'LaserScan'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('boolean')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import LaserScan
        self.laser_scan = kwargs.get('laser_scan', LaserScan())
        self.reflektor_status = kwargs.get('reflektor_status', [])
        self.reflektor_median = kwargs.get('reflektor_median', [])
        self.intrusion = kwargs.get('intrusion', [])

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
        if self.laser_scan != other.laser_scan:
            return False
        if self.reflektor_status != other.reflektor_status:
            return False
        if self.reflektor_median != other.reflektor_median:
            return False
        if self.intrusion != other.intrusion:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def laser_scan(self):
        """Message field 'laser_scan'."""
        return self._laser_scan

    @laser_scan.setter
    def laser_scan(self, value):
        if __debug__:
            from sensor_msgs.msg import LaserScan
            assert \
                isinstance(value, LaserScan), \
                "The 'laser_scan' field must be a sub message of type 'LaserScan'"
        self._laser_scan = value

    @builtins.property
    def reflektor_status(self):
        """Message field 'reflektor_status'."""
        return self._reflektor_status

    @reflektor_status.setter
    def reflektor_status(self, value):
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
                "The 'reflektor_status' field must be a set or sequence and each value of type 'bool'"
        self._reflektor_status = value

    @builtins.property
    def reflektor_median(self):
        """Message field 'reflektor_median'."""
        return self._reflektor_median

    @reflektor_median.setter
    def reflektor_median(self, value):
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
                "The 'reflektor_median' field must be a set or sequence and each value of type 'bool'"
        self._reflektor_median = value

    @builtins.property
    def intrusion(self):
        """Message field 'intrusion'."""
        return self._intrusion

    @intrusion.setter
    def intrusion(self, value):
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
                "The 'intrusion' field must be a set or sequence and each value of type 'bool'"
        self._intrusion = value
