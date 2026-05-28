# generated from rosidl_generator_py/resource/_idl.py.em
# with input from rovio_interfaces:msg/Health.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Health(type):
    """Metaclass of message 'Health'."""

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
                'rovio_interfaces.msg.Health')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__health
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__health
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__health
            cls._TYPE_SUPPORT = module.type_support_msg__msg__health
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__health

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Health(metaclass=Metaclass_Health):
    """Message class 'Health'."""

    __slots__ = [
        '_header',
        '_valid_feature_ratio',
        '_tracked_feature_ratio',
        '_pixel_covariance_ratio',
        '_nis_z_score_rmse',
        '_accel_deviation',
        '_speed_deviation',
        '_depth_feature_cov_median',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'valid_feature_ratio': 'float',
        'tracked_feature_ratio': 'float',
        'pixel_covariance_ratio': 'float',
        'nis_z_score_rmse': 'float',
        'accel_deviation': 'float',
        'speed_deviation': 'float',
        'depth_feature_cov_median': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.valid_feature_ratio = kwargs.get('valid_feature_ratio', float())
        self.tracked_feature_ratio = kwargs.get('tracked_feature_ratio', float())
        self.pixel_covariance_ratio = kwargs.get('pixel_covariance_ratio', float())
        self.nis_z_score_rmse = kwargs.get('nis_z_score_rmse', float())
        self.accel_deviation = kwargs.get('accel_deviation', float())
        self.speed_deviation = kwargs.get('speed_deviation', float())
        self.depth_feature_cov_median = kwargs.get('depth_feature_cov_median', float())

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
        if self.valid_feature_ratio != other.valid_feature_ratio:
            return False
        if self.tracked_feature_ratio != other.tracked_feature_ratio:
            return False
        if self.pixel_covariance_ratio != other.pixel_covariance_ratio:
            return False
        if self.nis_z_score_rmse != other.nis_z_score_rmse:
            return False
        if self.accel_deviation != other.accel_deviation:
            return False
        if self.speed_deviation != other.speed_deviation:
            return False
        if self.depth_feature_cov_median != other.depth_feature_cov_median:
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
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def valid_feature_ratio(self):
        """Message field 'valid_feature_ratio'."""
        return self._valid_feature_ratio

    @valid_feature_ratio.setter
    def valid_feature_ratio(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'valid_feature_ratio' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'valid_feature_ratio' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._valid_feature_ratio = value

    @builtins.property
    def tracked_feature_ratio(self):
        """Message field 'tracked_feature_ratio'."""
        return self._tracked_feature_ratio

    @tracked_feature_ratio.setter
    def tracked_feature_ratio(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tracked_feature_ratio' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'tracked_feature_ratio' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._tracked_feature_ratio = value

    @builtins.property
    def pixel_covariance_ratio(self):
        """Message field 'pixel_covariance_ratio'."""
        return self._pixel_covariance_ratio

    @pixel_covariance_ratio.setter
    def pixel_covariance_ratio(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pixel_covariance_ratio' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pixel_covariance_ratio' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pixel_covariance_ratio = value

    @builtins.property
    def nis_z_score_rmse(self):
        """Message field 'nis_z_score_rmse'."""
        return self._nis_z_score_rmse

    @nis_z_score_rmse.setter
    def nis_z_score_rmse(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'nis_z_score_rmse' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'nis_z_score_rmse' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._nis_z_score_rmse = value

    @builtins.property
    def accel_deviation(self):
        """Message field 'accel_deviation'."""
        return self._accel_deviation

    @accel_deviation.setter
    def accel_deviation(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'accel_deviation' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'accel_deviation' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._accel_deviation = value

    @builtins.property
    def speed_deviation(self):
        """Message field 'speed_deviation'."""
        return self._speed_deviation

    @speed_deviation.setter
    def speed_deviation(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_deviation' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_deviation' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_deviation = value

    @builtins.property
    def depth_feature_cov_median(self):
        """Message field 'depth_feature_cov_median'."""
        return self._depth_feature_cov_median

    @depth_feature_cov_median.setter
    def depth_feature_cov_median(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'depth_feature_cov_median' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'depth_feature_cov_median' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._depth_feature_cov_median = value
