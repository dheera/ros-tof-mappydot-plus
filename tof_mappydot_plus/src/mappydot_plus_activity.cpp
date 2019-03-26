/* mappydot_plus_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a MappyDotPlus Activity class, constructed with node handles
 * and which handles all ROS duties.
 */

#include "tof_mappydot_plus/mappydot_plus_activity.h"

namespace tof_mappydot_plus {

// ******** constructors ******** //

MappyDotPlusActivity::MappyDotPlusActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");
    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, std::vector<int>{ 0x08 });
    nh_priv.param("x", param_x, std::vector<double>{ 0.0 });
    nh_priv.param("y", param_y, std::vector<double>{ 0.0 });
    nh_priv.param("z", param_z, std::vector<double>{ 0.0 });
    nh_priv.param("yaw", param_yaw, std::vector<double>{ 0.0 });
    nh_priv.param("frame_id", param_frame_id, (std::string)"tof");

    if(param_address.size() != param_x.size()) {
        ROS_ERROR("size of x parameter must match size of address parameter");
	ros::shutdown();
    }

    if(param_address.size() != param_y.size()) {
        ROS_ERROR("size of y parameter must match size of address parameter");
	ros::shutdown();
    }

    if(param_address.size() != param_z.size()) {
        ROS_ERROR("size of z parameter must match size of address parameter");
	ros::shutdown();
    }

    if(param_address.size() != param_yaw.size()) {
        ROS_ERROR("size of yaw parameter must match size of address parameter");
	ros::shutdown();
    }
}

// ******** private methods ******** //

bool MappyDotPlusActivity::reset() {
    for(int i=0; i < param_address.size(); i++) {
    	if(ioctl(file, I2C_SLAVE, param_address[i]) < 0) {
            ROS_WARN_THROTTLE(2, "error setting slave address");
            return false;
        }
    	_i2c_smbus_write_byte_data(file, MAPPYDOT_PLUS_REG_RANGING_MEASUREMENT_MODE, MAPPYDOT_PLUS_RANGE_LONG);
    }
    return true;
}

// ******** public methods ******** //

bool MappyDotPlusActivity::start() {
    ROS_INFO("starting");

    if(!pub_ranges) pub_ranges = nh.advertise<std_msgs::Float32MultiArray>("ranges", 1);

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address[0]) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    return true;
}

bool MappyDotPlusActivity::spinOnce() {
    ros::spinOnce();

    ros::Time time = ros::Time::now();

    seq++;

    double range;
    int error;

    for(int i=0; i < param_address.size(); i++) {
    	if(ioctl(file, I2C_SLAVE, param_address[i]) < 0) {
            ROS_WARN_THROTTLE(2, "error setting slave address");
            return false;
        }
	range = (double)(int16_t)__bswap_16(_i2c_smbus_read_word_data(file, MAPPYDOT_PLUS_REG_DISTANCE));
	error = (uint8_t)(_i2c_smbus_read_byte_data(file, MAPPYDOT_PLUS_REG_READ_ERROR_CODE));
	ROS_WARN_STREAM("sensor " << i << " range " << range << " error " << error);
    }

    return true;    
}

bool MappyDotPlusActivity::stop() {
    ROS_INFO("stopping");

    if(pub_ranges) pub_ranges.shutdown();
    if(pub_points) pub_points.shutdown();

    return true;
}

}
