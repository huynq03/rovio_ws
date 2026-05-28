# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rovio_interfaces:srv/SrvResetToPose.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SrvResetToPose_Request(type):
    """Metaclass of message 'SrvResetToPose_Request'."""

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
            module = import_type_support('rovio_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rovio_interfaces.srv.SrvResetToPose_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__srv_reset_to_pose__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__srv_reset_to_pose__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__srv_reset_to_pose__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__srv_reset_to_pose__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__srv_reset_to_pose__request

            from geometry_msgs.msg import Pose
            if Pose.__class__._TYPE_SUPPORT is None:
                Pose.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SrvResetToPose_Request(metaclass=Metaclass_SrvResetToPose_Request):
    """Message class 'SrvResetToPose_Request'."""

    __slots__ = [
        '_t_wm',
    ]

    _fields_and_field_types = {
        't_wm': 'geometry_msgs/Pose',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Pose'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from geometry_msgs.msg import Pose
        self.t_wm = kwargs.get('t_wm', Pose())

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
        if self.t_wm != other.t_wm:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def t_wm(self):
        """Message field 't_wm'."""
        return self._t_wm

    @t_wm.setter
    def t_wm(self, value):
        if __debug__:
            from geometry_msgs.msg import Pose
            assert \
                isinstance(value, Pose), \
                "The 't_wm' field must be a sub message of type 'Pose'"
        self._t_wm = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SrvResetToPose_Response(type):
    """Metaclass of message 'SrvResetToPose_Response'."""

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
            module = import_type_support('rovio_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rovio_interfaces.srv.SrvResetToPose_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__srv_reset_to_pose__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__srv_reset_to_pose__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__srv_reset_to_pose__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__srv_reset_to_pose__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__srv_reset_to_pose__response

            from std_msgs.msg import Empty
            if Empty.__class__._TYPE_SUPPORT is None:
                Empty.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SrvResetToPose_Response(metaclass=Metaclass_SrvResetToPose_Response):
    """Message class 'SrvResetToPose_Response'."""

    __slots__ = [
        '_nothing',
    ]

    _fields_and_field_types = {
        'nothing': 'std_msgs/Empty',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Empty'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Empty
        self.nothing = kwargs.get('nothing', Empty())

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
        if self.nothing != other.nothing:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def nothing(self):
        """Message field 'nothing'."""
        return self._nothing

    @nothing.setter
    def nothing(self, value):
        if __debug__:
            from std_msgs.msg import Empty
            assert \
                isinstance(value, Empty), \
                "The 'nothing' field must be a sub message of type 'Empty'"
        self._nothing = value


class Metaclass_SrvResetToPose(type):
    """Metaclass of service 'SrvResetToPose'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('rovio_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'rovio_interfaces.srv.SrvResetToPose')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__srv_reset_to_pose

            from rovio_interfaces.srv import _srv_reset_to_pose
            if _srv_reset_to_pose.Metaclass_SrvResetToPose_Request._TYPE_SUPPORT is None:
                _srv_reset_to_pose.Metaclass_SrvResetToPose_Request.__import_type_support__()
            if _srv_reset_to_pose.Metaclass_SrvResetToPose_Response._TYPE_SUPPORT is None:
                _srv_reset_to_pose.Metaclass_SrvResetToPose_Response.__import_type_support__()


class SrvResetToPose(metaclass=Metaclass_SrvResetToPose):
    from rovio_interfaces.srv._srv_reset_to_pose import SrvResetToPose_Request as Request
    from rovio_interfaces.srv._srv_reset_to_pose import SrvResetToPose_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
