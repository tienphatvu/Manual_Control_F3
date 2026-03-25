# generated from rosidl_generator_py/resource/_idl.py.em
# with input from sick_safetyscanners2_interfaces:msg/ApplicationData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ApplicationData(type):
    """Metaclass of message 'ApplicationData'."""

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
                'sick_safetyscanners2_interfaces.msg.ApplicationData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__application_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__application_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__application_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__application_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__application_data

            from sick_safetyscanners2_interfaces.msg import ApplicationInputs
            if ApplicationInputs.__class__._TYPE_SUPPORT is None:
                ApplicationInputs.__class__.__import_type_support__()

            from sick_safetyscanners2_interfaces.msg import ApplicationOutputs
            if ApplicationOutputs.__class__._TYPE_SUPPORT is None:
                ApplicationOutputs.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ApplicationData(metaclass=Metaclass_ApplicationData):
    """Message class 'ApplicationData'."""

    __slots__ = [
        '_inputs',
        '_outputs',
    ]

    _fields_and_field_types = {
        'inputs': 'sick_safetyscanners2_interfaces/ApplicationInputs',
        'outputs': 'sick_safetyscanners2_interfaces/ApplicationOutputs',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'ApplicationInputs'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sick_safetyscanners2_interfaces', 'msg'], 'ApplicationOutputs'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sick_safetyscanners2_interfaces.msg import ApplicationInputs
        self.inputs = kwargs.get('inputs', ApplicationInputs())
        from sick_safetyscanners2_interfaces.msg import ApplicationOutputs
        self.outputs = kwargs.get('outputs', ApplicationOutputs())

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
        if self.inputs != other.inputs:
            return False
        if self.outputs != other.outputs:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def inputs(self):
        """Message field 'inputs'."""
        return self._inputs

    @inputs.setter
    def inputs(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import ApplicationInputs
            assert \
                isinstance(value, ApplicationInputs), \
                "The 'inputs' field must be a sub message of type 'ApplicationInputs'"
        self._inputs = value

    @builtins.property
    def outputs(self):
        """Message field 'outputs'."""
        return self._outputs

    @outputs.setter
    def outputs(self, value):
        if __debug__:
            from sick_safetyscanners2_interfaces.msg import ApplicationOutputs
            assert \
                isinstance(value, ApplicationOutputs), \
                "The 'outputs' field must be a sub message of type 'ApplicationOutputs'"
        self._outputs = value
