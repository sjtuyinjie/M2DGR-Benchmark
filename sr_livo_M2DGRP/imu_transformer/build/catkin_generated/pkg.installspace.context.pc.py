# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;message_filters;nodelet;sensor_msgs;tf2;tf2_ros;geometry_msgs;topic_tools".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-limu_transformer_nodelet".split(';') if "-limu_transformer_nodelet" != "" else []
PROJECT_NAME = "imu_transformer"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.3.1"
