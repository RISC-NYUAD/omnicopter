# generated from rosidl_generator_py/resource/_idl.py.em
# with input from maneuver:srv/Ellipse5D.idl
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


class Metaclass_Ellipse5D_Request(type):
    """Metaclass of message 'Ellipse5D_Request'."""

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
            module = import_type_support('maneuver')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'maneuver.srv.Ellipse5D_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__ellipse5_d__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__ellipse5_d__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__ellipse5_d__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__ellipse5_d__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__ellipse5_d__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Ellipse5D_Request(metaclass=Metaclass_Ellipse5D_Request):
    """Message class 'Ellipse5D_Request'."""

    __slots__ = [
        '_x_min',
        '_x_max',
        '_y_min',
        '_y_max',
        '_z_min',
        '_z_max',
        '_roll_start',
        '_roll_mid',
        '_roll_end',
        '_pitch_start',
        '_pitch_mid',
        '_pitch_end',
        '_yaw',
        '_duration',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'x_min': 'float',
        'x_max': 'float',
        'y_min': 'float',
        'y_max': 'float',
        'z_min': 'float',
        'z_max': 'float',
        'roll_start': 'float',
        'roll_mid': 'float',
        'roll_end': 'float',
        'pitch_start': 'float',
        'pitch_mid': 'float',
        'pitch_end': 'float',
        'yaw': 'float',
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
        self.x_min = kwargs.get('x_min', float())
        self.x_max = kwargs.get('x_max', float())
        self.y_min = kwargs.get('y_min', float())
        self.y_max = kwargs.get('y_max', float())
        self.z_min = kwargs.get('z_min', float())
        self.z_max = kwargs.get('z_max', float())
        self.roll_start = kwargs.get('roll_start', float())
        self.roll_mid = kwargs.get('roll_mid', float())
        self.roll_end = kwargs.get('roll_end', float())
        self.pitch_start = kwargs.get('pitch_start', float())
        self.pitch_mid = kwargs.get('pitch_mid', float())
        self.pitch_end = kwargs.get('pitch_end', float())
        self.yaw = kwargs.get('yaw', float())
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
        if self.x_min != other.x_min:
            return False
        if self.x_max != other.x_max:
            return False
        if self.y_min != other.y_min:
            return False
        if self.y_max != other.y_max:
            return False
        if self.z_min != other.z_min:
            return False
        if self.z_max != other.z_max:
            return False
        if self.roll_start != other.roll_start:
            return False
        if self.roll_mid != other.roll_mid:
            return False
        if self.roll_end != other.roll_end:
            return False
        if self.pitch_start != other.pitch_start:
            return False
        if self.pitch_mid != other.pitch_mid:
            return False
        if self.pitch_end != other.pitch_end:
            return False
        if self.yaw != other.yaw:
            return False
        if self.duration != other.duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def x_min(self):
        """Message field 'x_min'."""
        return self._x_min

    @x_min.setter
    def x_min(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_min = value

    @builtins.property
    def x_max(self):
        """Message field 'x_max'."""
        return self._x_max

    @x_max.setter
    def x_max(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x_max = value

    @builtins.property
    def y_min(self):
        """Message field 'y_min'."""
        return self._y_min

    @y_min.setter
    def y_min(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_min = value

    @builtins.property
    def y_max(self):
        """Message field 'y_max'."""
        return self._y_max

    @y_max.setter
    def y_max(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y_max = value

    @builtins.property
    def z_min(self):
        """Message field 'z_min'."""
        return self._z_min

    @z_min.setter
    def z_min(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z_min' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_min' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_min = value

    @builtins.property
    def z_max(self):
        """Message field 'z_max'."""
        return self._z_max

    @z_max.setter
    def z_max(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z_max' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z_max' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z_max = value

    @builtins.property
    def roll_start(self):
        """Message field 'roll_start'."""
        return self._roll_start

    @roll_start.setter
    def roll_start(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'roll_start' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_start' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_start = value

    @builtins.property
    def roll_mid(self):
        """Message field 'roll_mid'."""
        return self._roll_mid

    @roll_mid.setter
    def roll_mid(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'roll_mid' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_mid' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_mid = value

    @builtins.property
    def roll_end(self):
        """Message field 'roll_end'."""
        return self._roll_end

    @roll_end.setter
    def roll_end(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'roll_end' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'roll_end' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._roll_end = value

    @builtins.property
    def pitch_start(self):
        """Message field 'pitch_start'."""
        return self._pitch_start

    @pitch_start.setter
    def pitch_start(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch_start' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_start' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_start = value

    @builtins.property
    def pitch_mid(self):
        """Message field 'pitch_mid'."""
        return self._pitch_mid

    @pitch_mid.setter
    def pitch_mid(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch_mid' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_mid' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_mid = value

    @builtins.property
    def pitch_end(self):
        """Message field 'pitch_end'."""
        return self._pitch_end

    @pitch_end.setter
    def pitch_end(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pitch_end' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pitch_end' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pitch_end = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw = value

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


class Metaclass_Ellipse5D_Response(type):
    """Metaclass of message 'Ellipse5D_Response'."""

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
            module = import_type_support('maneuver')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'maneuver.srv.Ellipse5D_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__ellipse5_d__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__ellipse5_d__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__ellipse5_d__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__ellipse5_d__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__ellipse5_d__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Ellipse5D_Response(metaclass=Metaclass_Ellipse5D_Response):
    """Message class 'Ellipse5D_Response'."""

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


class Metaclass_Ellipse5D_Event(type):
    """Metaclass of message 'Ellipse5D_Event'."""

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
            module = import_type_support('maneuver')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'maneuver.srv.Ellipse5D_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__ellipse5_d__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__ellipse5_d__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__ellipse5_d__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__ellipse5_d__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__ellipse5_d__event

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


class Ellipse5D_Event(metaclass=Metaclass_Ellipse5D_Event):
    """Message class 'Ellipse5D_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<maneuver/Ellipse5D_Request, 1>',
        'response': 'sequence<maneuver/Ellipse5D_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['maneuver', 'srv'], 'Ellipse5D_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['maneuver', 'srv'], 'Ellipse5D_Response'), 1),  # noqa: E501
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
            from maneuver.srv import Ellipse5D_Request
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
                 all(isinstance(v, Ellipse5D_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'Ellipse5D_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from maneuver.srv import Ellipse5D_Response
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
                 all(isinstance(v, Ellipse5D_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'Ellipse5D_Response'"
        self._response = value


class Metaclass_Ellipse5D(type):
    """Metaclass of service 'Ellipse5D'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('maneuver')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'maneuver.srv.Ellipse5D')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__ellipse5_d

            from maneuver.srv import _ellipse5_d
            if _ellipse5_d.Metaclass_Ellipse5D_Request._TYPE_SUPPORT is None:
                _ellipse5_d.Metaclass_Ellipse5D_Request.__import_type_support__()
            if _ellipse5_d.Metaclass_Ellipse5D_Response._TYPE_SUPPORT is None:
                _ellipse5_d.Metaclass_Ellipse5D_Response.__import_type_support__()
            if _ellipse5_d.Metaclass_Ellipse5D_Event._TYPE_SUPPORT is None:
                _ellipse5_d.Metaclass_Ellipse5D_Event.__import_type_support__()


class Ellipse5D(metaclass=Metaclass_Ellipse5D):
    from maneuver.srv._ellipse5_d import Ellipse5D_Request as Request
    from maneuver.srv._ellipse5_d import Ellipse5D_Response as Response
    from maneuver.srv._ellipse5_d import Ellipse5D_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
