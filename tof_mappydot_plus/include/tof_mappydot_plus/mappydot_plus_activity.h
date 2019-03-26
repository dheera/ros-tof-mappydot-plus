#ifndef _mappydot_plus_activity_dot_h
#define _mappydot_plus_activity_dot_h

#include <ros/ros.h>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <byteswap.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>

#define MAPPYDOT_PLUS_ADDRESS_START 0x08

#define MAPPYDOT_PLUS_REG_DISTANCE  0x72
#define MAPPYDOT_PLUS_REG_PERFORM_SINGLE_RANGE                        0x53
#define MAPPYDOT_PLUS_REG_READ_ACCURACY                               0x52
#define MAPPYDOT_PLUS_REG_READ_ERROR_CODE                             0x45
#define MAPPYDOT_PLUS_REG_RANGING_MEASUREMENT_MODE                    0x6d
#define MAPPYDOT_PLUS_REG_MEASUREMENT_BUDGET                          0x42
#define MAPPYDOT_PLUS_REG_SET_CONTINUOUS_RANGING_MODE                 0x63
#define MAPPYDOT_PLUS_REG_SET_SINGLE_RANGING_MODE                     0x73
#define MAPPYDOT_PLUS_REG_CHECK_INTERRUPT                             0x49

#define MAPPYDOT_PLUS_REG_FILTERING_ENABLE                            (0x46)
#define MAPPYDOT_PLUS_REG_FILTERING_DISABLE                           (0x66)
#define MAPPYDOT_PLUS_REG_AVERAGING_ENABLE                            (0x56)
#define MAPPYDOT_PLUS_REG_AVERAGING_DISABLE                           (0x76)
#define MAPPYDOT_PLUS_REG_AVERAGING_SAMPLES                           (0x69)
#define MAPPYDOT_PLUS_REG_REGION_OF_INTEREST                          (0x70)
#define MAPPYDOT_PLUS_REG_SIGMA_LIMIT_CHECK_VALUE                     (0x4C)
#define MAPPYDOT_PLUS_REG_SIGNAL_LIMIT_CHECK_VALUE                    (0x47)
#define MAPPYDOT_PLUS_REG_SET_LED_MODE                                (0x6c)
#define MAPPYDOT_PLUS_REG_SET_LED_THRESHOLD_DISTANCE_IN_MM            (0x65)
#define MAPPYDOT_PLUS_REG_SET_GPIO_MODE                               (0x67)
#define MAPPYDOT_PLUS_REG_SET_GPIO_THRESHOLD_DISTANCE_IN_MM           (0x6f)
#define MAPPYDOT_PLUS_REG_CALIBRATE_DISTANCE_OFFSET                   (0x61)
#define MAPPYDOT_PLUS_REG_CALIBRATE_CROSSTALK                         (0x78)
#define MAPPYDOT_PLUS_REG_ENABLE_CROSSTALK_COMPENSATION               (0x4b)
#define MAPPYDOT_PLUS_REG_DISABLE_CROSSTALK_COMPENSATION              (0x6b) 
#define MAPPYDOT_PLUS_REG_CALIBRATE_SPAD                              (0x75)
#define MAPPYDOT_PLUS_REG_INTERSENSOR_CROSSTALK_REDUCTION_ENABLE      (0x54)
#define MAPPYDOT_PLUS_REG_INTERSENSOR_CROSSTALK_REDUCTION_DISABLE     (0x74)
#define MAPPYDOT_PLUS_REG_INTERSENSOR_CROSSTALK_TIMEOUT               (0x71)
#define MAPPYDOT_PLUS_REG_INTERSENSOR_CROSSTALK_MEASUREMENT_DELAY     (0x51)
#define MAPPYDOT_PLUS_REG_INTERSENSOR_SYNC_ENABLE                     (0x59)
#define MAPPYDOT_PLUS_REG_INTERSENSOR_SYNC_DISABLE                    (0x79)

#define MAPPYDOT_PLUS_REG_FIRMWARE_VERSION                            (0x4e)
#define MAPPYDOT_PLUS_REG_NAME_DEVICE                                 (0x6e)
#define MAPPYDOT_PLUS_REG_DEVICE_NAME                                 (0x64)
#define MAPPYDOT_PLUS_REG_READ_CURRENT_SETTINGS                       (0x62)
#define MAPPYDOT_PLUS_REG_RESTORE_FACTORY_DEFAULTS                    (0x7a)
#define MAPPYDOT_PLUS_REG_WRITE_CURRENT_SETTINGS_AS_START_UP_DEFAULT  (0x77)

#define MAPPYDOT_PLUS_REG_RESET_VL53L1X_RANGING                       (0x58)
#define MAPPYDOT_PLUS_REG_VL53L1X_NOT_SHUTDOWN                        (0x48)
#define MAPPYDOT_PLUS_REG_VL53L1X_SHUTDOWN                            (0x68)
#define MAPPYDOT_PLUS_REG_READ_NONFILTERED_VALUE                      (0x6a)
#define MAPPYDOT_PLUS_REG_AMBIENT_RATE_RETURN                         (0x41)
#define MAPPYDOT_PLUS_REG_SIGNAL_RATE_RETURN                          (0x4A)

#define MAPPYDOT_PLUS_RANGE_SHORT                                 (0x73)
#define MAPPYDOT_PLUS_RANGE_MED                                   (0x6d)
#define MAPPYDOT_PLUS_RANGE_LONG                                  (0x6c)

namespace tof_mappydot_plus {

class MappyDotPlusActivity {
  public:
    MappyDotPlusActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

    bool start();
    bool stop();
    bool spinOnce();

  private:
    bool reset();

    // class variables
    uint32_t seq = 0;
    int file;
    sensor_msgs::PointField pointfield_x;
    sensor_msgs::PointField pointfield_y;
    sensor_msgs::PointField pointfield_z;
    sensor_msgs::PointCloud2 msg_points;
    std_msgs::Float32MultiArray msg_ranges;

    // ROS parameters
    std::string param_frame_id;
    std::string param_device;
    std::vector<int> param_address;
    std::vector<double> param_x;
    std::vector<double> param_y;
    std::vector<double> param_z;
    std::vector<double> param_yaw;

    // ROS node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    // ROS publishers
    ros::Publisher pub_points;
    ros::Publisher pub_ranges;

    // ROS subscribers
};

}

#endif // _mappydot_plus_activity_dot_h
