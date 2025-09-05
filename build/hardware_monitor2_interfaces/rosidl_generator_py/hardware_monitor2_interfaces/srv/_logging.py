# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hardware_monitor2_interfaces:srv/Logging.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Logging_Request(type):
    """Metaclass of message 'Logging_Request'."""

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
            module = import_type_support('hardware_monitor2_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hardware_monitor2_interfaces.srv.Logging_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__logging__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__logging__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__logging__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__logging__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__logging__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Logging_Request(metaclass=Metaclass_Logging_Request):
    """Message class 'Logging_Request'."""

    __slots__ = [
        '_is_logging',
    ]

    _fields_and_field_types = {
        'is_logging': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_logging = kwargs.get('is_logging', str())

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
        if self.is_logging != other.is_logging:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_logging(self):
        """Message field 'is_logging'."""
        return self._is_logging

    @is_logging.setter
    def is_logging(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'is_logging' field must be of type 'str'"
        self._is_logging = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Logging_Response(type):
    """Metaclass of message 'Logging_Response'."""

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
            module = import_type_support('hardware_monitor2_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hardware_monitor2_interfaces.srv.Logging_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__logging__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__logging__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__logging__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__logging__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__logging__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Logging_Response(metaclass=Metaclass_Logging_Response):
    """Message class 'Logging_Response'."""

    __slots__ = [
        '_logging_status',
    ]

    _fields_and_field_types = {
        'logging_status': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.logging_status = kwargs.get('logging_status', str())

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
        if self.logging_status != other.logging_status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def logging_status(self):
        """Message field 'logging_status'."""
        return self._logging_status

    @logging_status.setter
    def logging_status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'logging_status' field must be of type 'str'"
        self._logging_status = value


class Metaclass_Logging(type):
    """Metaclass of service 'Logging'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hardware_monitor2_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hardware_monitor2_interfaces.srv.Logging')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__logging

            from hardware_monitor2_interfaces.srv import _logging
            if _logging.Metaclass_Logging_Request._TYPE_SUPPORT is None:
                _logging.Metaclass_Logging_Request.__import_type_support__()
            if _logging.Metaclass_Logging_Response._TYPE_SUPPORT is None:
                _logging.Metaclass_Logging_Response.__import_type_support__()


class Logging(metaclass=Metaclass_Logging):
    from hardware_monitor2_interfaces.srv._logging import Logging_Request as Request
    from hardware_monitor2_interfaces.srv._logging import Logging_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
