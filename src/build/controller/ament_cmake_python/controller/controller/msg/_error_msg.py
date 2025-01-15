# generated from rosidl_generator_py/resource/_idl.py.em
# with input from controller:msg/ErrorMsg.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

# Member 'wrench'
# Member 'exwrench'
# Member 'prop'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ErrorMsg(type):
    """Metaclass of message 'ErrorMsg'."""

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
                'controller.msg.ErrorMsg')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__error_msg
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__error_msg
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__error_msg
            cls._TYPE_SUPPORT = module.type_support_msg__msg__error_msg
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__error_msg

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ErrorMsg(metaclass=Metaclass_ErrorMsg):
    """Message class 'ErrorMsg'."""

    __slots__ = [
        '_ex',
        '_ev',
        '_ea',
        '_er',
        '_ew',
        '_iex',
        '_ier',
        '_accd',
        '_wrench',
        '_exwrench',
        '_prop',
        '_weight',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'ex': 'geometry_msgs/Vector3',
        'ev': 'geometry_msgs/Vector3',
        'ea': 'geometry_msgs/Vector3',
        'er': 'geometry_msgs/Vector3',
        'ew': 'geometry_msgs/Vector3',
        'iex': 'geometry_msgs/Vector3',
        'ier': 'geometry_msgs/Vector3',
        'accd': 'geometry_msgs/Vector3',
        'wrench': 'sequence<float>',
        'exwrench': 'sequence<float>',
        'prop': 'sequence<float>',
        'weight': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
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
        from geometry_msgs.msg import Vector3
        self.ex = kwargs.get('ex', Vector3())
        from geometry_msgs.msg import Vector3
        self.ev = kwargs.get('ev', Vector3())
        from geometry_msgs.msg import Vector3
        self.ea = kwargs.get('ea', Vector3())
        from geometry_msgs.msg import Vector3
        self.er = kwargs.get('er', Vector3())
        from geometry_msgs.msg import Vector3
        self.ew = kwargs.get('ew', Vector3())
        from geometry_msgs.msg import Vector3
        self.iex = kwargs.get('iex', Vector3())
        from geometry_msgs.msg import Vector3
        self.ier = kwargs.get('ier', Vector3())
        from geometry_msgs.msg import Vector3
        self.accd = kwargs.get('accd', Vector3())
        self.wrench = array.array('f', kwargs.get('wrench', []))
        self.exwrench = array.array('f', kwargs.get('exwrench', []))
        self.prop = array.array('f', kwargs.get('prop', []))
        self.weight = kwargs.get('weight', float())

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
        if self.ex != other.ex:
            return False
        if self.ev != other.ev:
            return False
        if self.ea != other.ea:
            return False
        if self.er != other.er:
            return False
        if self.ew != other.ew:
            return False
        if self.iex != other.iex:
            return False
        if self.ier != other.ier:
            return False
        if self.accd != other.accd:
            return False
        if self.wrench != other.wrench:
            return False
        if self.exwrench != other.exwrench:
            return False
        if self.prop != other.prop:
            return False
        if self.weight != other.weight:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ex(self):
        """Message field 'ex'."""
        return self._ex

    @ex.setter
    def ex(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'ex' field must be a sub message of type 'Vector3'"
        self._ex = value

    @builtins.property
    def ev(self):
        """Message field 'ev'."""
        return self._ev

    @ev.setter
    def ev(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'ev' field must be a sub message of type 'Vector3'"
        self._ev = value

    @builtins.property
    def ea(self):
        """Message field 'ea'."""
        return self._ea

    @ea.setter
    def ea(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'ea' field must be a sub message of type 'Vector3'"
        self._ea = value

    @builtins.property
    def er(self):
        """Message field 'er'."""
        return self._er

    @er.setter
    def er(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'er' field must be a sub message of type 'Vector3'"
        self._er = value

    @builtins.property
    def ew(self):
        """Message field 'ew'."""
        return self._ew

    @ew.setter
    def ew(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'ew' field must be a sub message of type 'Vector3'"
        self._ew = value

    @builtins.property
    def iex(self):
        """Message field 'iex'."""
        return self._iex

    @iex.setter
    def iex(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'iex' field must be a sub message of type 'Vector3'"
        self._iex = value

    @builtins.property
    def ier(self):
        """Message field 'ier'."""
        return self._ier

    @ier.setter
    def ier(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'ier' field must be a sub message of type 'Vector3'"
        self._ier = value

    @builtins.property
    def accd(self):
        """Message field 'accd'."""
        return self._accd

    @accd.setter
    def accd(self, value):
        if self._check_fields:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'accd' field must be a sub message of type 'Vector3'"
        self._accd = value

    @builtins.property
    def wrench(self):
        """Message field 'wrench'."""
        return self._wrench

    @wrench.setter
    def wrench(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'f', \
                    "The 'wrench' array.array() must have the type code of 'f'"
                self._wrench = value
                return
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'wrench' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._wrench = array.array('f', value)

    @builtins.property
    def exwrench(self):
        """Message field 'exwrench'."""
        return self._exwrench

    @exwrench.setter
    def exwrench(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'f', \
                    "The 'exwrench' array.array() must have the type code of 'f'"
                self._exwrench = value
                return
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'exwrench' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._exwrench = array.array('f', value)

    @builtins.property
    def prop(self):
        """Message field 'prop'."""
        return self._prop

    @prop.setter
    def prop(self, value):
        if self._check_fields:
            if isinstance(value, array.array):
                assert value.typecode == 'f', \
                    "The 'prop' array.array() must have the type code of 'f'"
                self._prop = value
                return
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'prop' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._prop = array.array('f', value)

    @builtins.property
    def weight(self):
        """Message field 'weight'."""
        return self._weight

    @weight.setter
    def weight(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'weight' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'weight' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._weight = value
