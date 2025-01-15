# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:srv/LinearWrench.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_LinearWrench_Request(type):
    """Metaclass of message 'LinearWrench_Request'."""

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
            module = import_type_support('controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller.srv.LinearWrench_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__linear_wrench__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__linear_wrench__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__linear_wrench__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__linear_wrench__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__linear_wrench__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LinearWrench_Request(metaclass=Metaclass_LinearWrench_Request):
    """Message class 'LinearWrench_Request'."""

    __slots__ = [
        '_fx1',
        '_fy1',
        '_fz1',
        '_fx2',
        '_fy2',
        '_fz2',
        '_ramp',
        '_duration',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'fx1': 'float',
        'fy1': 'float',
        'fz1': 'float',
        'fx2': 'float',
        'fy2': 'float',
        'fz2': 'float',
        'ramp': 'float',
        'duration': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.fx1 = kwargs.get('fx1', float())
        self.fy1 = kwargs.get('fy1', float())
        self.fz1 = kwargs.get('fz1', float())
        self.fx2 = kwargs.get('fx2', float())
        self.fy2 = kwargs.get('fy2', float())
        self.fz2 = kwargs.get('fz2', float())
        self.ramp = kwargs.get('ramp', float())
        self.duration = kwargs.get('duration', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.fx1 != other.fx1:
            return False
        if self.fy1 != other.fy1:
            return False
        if self.fz1 != other.fz1:
            return False
        if self.fx2 != other.fx2:
            return False
        if self.fy2 != other.fy2:
            return False
        if self.fz2 != other.fz2:
            return False
        if self.ramp != other.ramp:
            return False
        if self.duration != other.duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def fx1(self):
        """Message field 'fx1'."""
        return self._fx1

    @fx1.setter
    def fx1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fx1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fx1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fx1 = value

    @builtins.property
    def fy1(self):
        """Message field 'fy1'."""
        return self._fy1

    @fy1.setter
    def fy1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fy1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fy1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fy1 = value

    @builtins.property
    def fz1(self):
        """Message field 'fz1'."""
        return self._fz1

    @fz1.setter
    def fz1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fz1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fz1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fz1 = value

    @builtins.property
    def fx2(self):
        """Message field 'fx2'."""
        return self._fx2

    @fx2.setter
    def fx2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fx2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fx2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fx2 = value

    @builtins.property
    def fy2(self):
        """Message field 'fy2'."""
        return self._fy2

    @fy2.setter
    def fy2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fy2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fy2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fy2 = value

    @builtins.property
    def fz2(self):
        """Message field 'fz2'."""
        return self._fz2

    @fz2.setter
    def fz2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'fz2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fz2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fz2 = value

    @builtins.property
    def ramp(self):
        """Message field 'ramp'."""
        return self._ramp

    @ramp.setter
    def ramp(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'ramp' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ramp' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ramp = value

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'duration' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'duration' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._duration = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LinearWrench_Response(type):
    """Metaclass of message 'LinearWrench_Response'."""

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
            module = import_type_support('controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller.srv.LinearWrench_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__linear_wrench__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__linear_wrench__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__linear_wrench__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__linear_wrench__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__linear_wrench__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LinearWrench_Response(metaclass=Metaclass_LinearWrench_Response):
    """Message class 'LinearWrench_Response'."""

    __slots__ = [
        '_status',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'status': 'boolean',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.status = kwargs.get('status', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.status != other.status:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'status' field must be of type 'bool'"
        self._status = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_LinearWrench_Event(type):
    """Metaclass of message 'LinearWrench_Event'."""

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
            module = import_type_support('controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller.srv.LinearWrench_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__linear_wrench__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__linear_wrench__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__linear_wrench__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__linear_wrench__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__linear_wrench__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class LinearWrench_Event(metaclass=Metaclass_LinearWrench_Event):
    """Message class 'LinearWrench_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<controller/LinearWrench_Request, 1>',
        'response': 'sequence<controller/LinearWrench_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['controller', 'srv'], 'LinearWrench_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['controller', 'srv'], 'LinearWrench_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
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
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from controller.srv import LinearWrench_Request
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
                 len(value) <= 1 and
                 all(isinstance(v, LinearWrench_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'LinearWrench_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from controller.srv import LinearWrench_Response
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
                 len(value) <= 1 and
                 all(isinstance(v, LinearWrench_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'LinearWrench_Response'"
        self._response = value


class Metaclass_LinearWrench(type):
    """Metaclass of service 'LinearWrench'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('controller')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'controller.srv.LinearWrench')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__linear_wrench

            from controller.srv import _linear_wrench
            if _linear_wrench.Metaclass_LinearWrench_Request._TYPE_SUPPORT is None:
                _linear_wrench.Metaclass_LinearWrench_Request.__import_type_support__()
            if _linear_wrench.Metaclass_LinearWrench_Response._TYPE_SUPPORT is None:
                _linear_wrench.Metaclass_LinearWrench_Response.__import_type_support__()
            if _linear_wrench.Metaclass_LinearWrench_Event._TYPE_SUPPORT is None:
                _linear_wrench.Metaclass_LinearWrench_Event.__import_type_support__()


class LinearWrench(metaclass=Metaclass_LinearWrench):
    from controller.srv._linear_wrench import LinearWrench_Request as Request
    from controller.srv._linear_wrench import LinearWrench_Response as Response
    from controller.srv._linear_wrench import LinearWrench_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
