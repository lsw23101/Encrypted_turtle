# generated from rosidl_generator_py/resource/_idl.py.em
# with input from turtle_demo_controller:msg/EncryptedData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EncryptedData(type):
    """Metaclass of message 'EncryptedData'."""

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
            module = import_type_support('turtle_demo_controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'turtle_demo_controller.msg.EncryptedData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encrypted_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encrypted_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encrypted_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encrypted_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encrypted_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EncryptedData(metaclass=Metaclass_EncryptedData):
    """Message class 'EncryptedData'."""

    __slots__ = [
        '_encrypted_data',
        '_data_type',
    ]

    _fields_and_field_types = {
        'encrypted_data': 'string',
        'data_type': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.encrypted_data = kwargs.get('encrypted_data', str())
        self.data_type = kwargs.get('data_type', str())

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
        if self.encrypted_data != other.encrypted_data:
            return False
        if self.data_type != other.data_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def encrypted_data(self):
        """Message field 'encrypted_data'."""
        return self._encrypted_data

    @encrypted_data.setter
    def encrypted_data(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'encrypted_data' field must be of type 'str'"
        self._encrypted_data = value

    @property
    def data_type(self):
        """Message field 'data_type'."""
        return self._data_type

    @data_type.setter
    def data_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'data_type' field must be of type 'str'"
        self._data_type = value
