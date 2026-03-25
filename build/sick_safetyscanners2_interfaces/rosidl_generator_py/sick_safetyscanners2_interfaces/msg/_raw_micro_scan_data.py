# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/RawMicroScanData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RawMicroScanData(type):
    """Metaclass of message 'RawMicroScanData'."""

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
                'sick_safetyscanners2_interfaces.msg.RawMicroScanData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__raw_micro_scan_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__raw_micro_scan_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__raw_micro_scan_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__raw_micro_scan_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__raw_micro_scan_data

            from sick_safetyscanners2_interfaces.msg import ApplicationData
            if ApplicationData.__class__._TYPE_SUPPORT is None:
                ApplicationData.__class__.__import_type_support__()

            from sick_safetyscanners2_interfaces.msg import DataHeader
            if DataHeader.__class__._TYPE_SUPPORT is None:
                DataHeader.__class__.__import_type_support__()

            from sick_safetyscanners2_interfaces.msg import DerivedValues
            if DerivedValues.__class__._TYPE_SUPPORT is None:
                DerivedValues.__class__.__import_type_support__()

            from sick_safetyscanners2_interfaces.msg import GeneralSystemState
            if GeneralSystemState.__class__._TYPE_SUPPORT is None:
                GeneralSystemState.__class__.__import_type_support__()

            from sick_safetyscanners2_interfaces.msg import IntrusionData
            if IntrusionData.__class__._TYPE_SUPPORT is None:
                IntrusionData.__class__.__import_type_support__()

            from sick_safetyscanners2_interfaces.msg import MeasurementData
            if MeasurementData.__class__._TYPE_SUPPORT is None:
                MeasurementData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RawMicroScanData(metaclass=Metaclass_RawMicroScanData):
    """Message class 'RawMicroScanData'."""

    __slots__ = [
        '_header',
        '_derived_values',
        '_general_system_state',
        '_measurement_data',
        '_intrusion_data',
        '_application_data',
    ]

    _fields_and_field_types = {
        'header': 'sick_safetyscanners2_interfaces/DataHeader',
        'derived_values': 'sick_safetyscanners2_interfaces/DerivedValues',
        'general_system_state': 'sick_safetyscanners2_interfaces/GeneralSystemState',
        'measurement_data': 'sick_safetyscanners2_interfaces/MeasurementData',
        'intrusion_data': 'sick_safetyscanners2_interfaces/IntrusionData',
        'application_data': 'sick_safetyscanners2_interfaces/ApplicationData',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'DataHeader'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'DerivedValues'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'GeneralSystemState'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'MeasurementData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'IntrusionData'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'ApplicationData'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sick_safetyscanners2_interfaces.msg import DataHeader
        self.header = kwargs.get('header', DataHeader())
        from sick_safetyscanners2_interfaces.msg import DerivedValues
        self.derived_values = kwargs.get('derived_values', DerivedValues())
        from sick_safetyscanners2_interfaces.msg import GeneralSystemState
        self.general_system_state = kwargs.get('general_system_state', GeneralSystemState())
        from sick_safetyscanners2_interfaces.msg import MeasurementData
        self.measurement_data = kwargs.get('measurement_data', MeasurementData())
        from sick_safetyscanners2_interfaces.msg import IntrusionData
        self.intrusion_data = kwargs.get('intrusion_data', IntrusionData())
        from sick_safetyscanners2_interfaces.msg import ApplicationData
        self.application_data = kwargs.get('application_data', ApplicationData())

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
        if self.header != other.header:
            return False
        if self.derived_values != other.derived_values:
            return False
        if self.general_system_state != other.general_system_state:
            return False
        if self.measurement_data != other.measurement_data:
            return False
        if self.intrusion_data != other.intrusion_data:
            return False
        if self.application_data != other.application_data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import DataHeader
            assert \
                isinstance(value, DataHeader), \
                "The 'header' field must be a sub message of type 'DataHeader'"
        self._header = value

    @builtins.property
    def derived_values(self):
        """Message field 'derived_values'."""
        return self._derived_values

    @derived_values.setter
    def derived_values(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import DerivedValues
            assert \
                isinstance(value, DerivedValues), \
                "The 'derived_values' field must be a sub message of type 'DerivedValues'"
        self._derived_values = value

    @builtins.property
    def general_system_state(self):
        """Message field 'general_system_state'."""
        return self._general_system_state

    @general_system_state.setter
    def general_system_state(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import GeneralSystemState
            assert \
                isinstance(value, GeneralSystemState), \
                "The 'general_system_state' field must be a sub message of type 'GeneralSystemState'"
        self._general_system_state = value

    @builtins.property
    def measurement_data(self):
        """Message field 'measurement_data'."""
        return self._measurement_data

    @measurement_data.setter
    def measurement_data(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import MeasurementData
            assert \
                isinstance(value, MeasurementData), \
                "The 'measurement_data' field must be a sub message of type 'MeasurementData'"
        self._measurement_data = value

    @builtins.property
    def intrusion_data(self):
        """Message field 'intrusion_data'."""
        return self._intrusion_data

    @intrusion_data.setter
    def intrusion_data(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import IntrusionData
            assert \
                isinstance(value, IntrusionData), \
                "The 'intrusion_data' field must be a sub message of type 'IntrusionData'"
        self._intrusion_data = value

    @builtins.property
    def application_data(self):
        """Message field 'application_data'."""
        return self._application_data

    @application_data.setter
    def application_data(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import ApplicationData
            assert \
                isinstance(value, ApplicationData), \
                "The 'application_data' field must be a sub message of type 'ApplicationData'"
        self._application_data = value
