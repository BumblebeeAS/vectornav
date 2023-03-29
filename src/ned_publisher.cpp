#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <boost/bind.hpp>
#include "frames/frames.h"
#include "geometry_msgs/Vector3Stamped.h"

using namespace frame_utils::coordinates;
using namespace frame_utils::geometry;
constexpr auto base_link_to_base_link_ned = make_rotation<FLU, FRD>(0, 0, 180).value();

void transform(
    ros::Publisher& ned_publisher, 
    ros::Publisher& rpy_publisher, 
    const sensor_msgs::Imu::ConstPtr& msg) {
    sensor_msgs::Imu ned_msg; 

    // Linear acceleration: swap axes
    ned_msg.linear_acceleration.x = msg->linear_acceleration.y; // E => N
    ned_msg.linear_acceleration.y = msg->linear_acceleration.x; // N => E
    ned_msg.linear_acceleration.z = -msg->linear_acceleration.z; // U => D

    // Angular velocity: swap axes
    ned_msg.angular_velocity.x = msg->angular_velocity.y; // Roll
    ned_msg.angular_velocity.y = msg->angular_velocity.x; // Pitch
    ned_msg.angular_velocity.z = msg->angular_velocity.z; // Yaw

    // Orientation: quaternion change
    const auto enu_orientation = make_rotation<ENU, FLU>(
        msg->orientation.w,
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z).value();
	const auto ned_orientation = (NEDtoENU(enu_orientation))(base_link_to_base_link_ned);

    ned_msg.orientation.w = ned_orientation.getW(); 
    ned_msg.orientation.x = ned_orientation.getX(); 
    ned_msg.orientation.y = ned_orientation.getY(); 
    ned_msg.orientation.z = ned_orientation.getZ(); 

    ned_publisher.publish(ned_msg);

    // Publish RPY
    geometry_msgs::Vector3Stamped rpy_msg; 

    // Angular velocity: swap axes
    rpy_msg.vector.x = msg->angular_velocity.y; // Roll
    rpy_msg.vector.y = msg->angular_velocity.x; // Pitch
    rpy_msg.vector.z = msg->angular_velocity.z; // Yaw

    rpy_publisher.publish(rpy_msg);
}

void to_rpy(ros::Publisher& publisher, const sensor_msgs::Imu::ConstPtr& msg) {
    geometry_msgs::Vector3Stamped rpy_msg; 

    // Angular velocity: swap axes
    rpy_msg.vector.x = msg->angular_velocity.y; // Roll
    rpy_msg.vector.y = msg->angular_velocity.x; // Pitch
    rpy_msg.vector.z = msg->angular_velocity.z; // Yaw

    publisher.publish(rpy_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vnav_ned");
    ros::NodeHandle n; 

    ros::Publisher ned_pub = n.advertise<sensor_msgs::Imu>("vnav/imu_ned", 10);
    ros::Publisher rpy_pub = n.advertise<geometry_msgs::Vector3Stamped>("vnav/debug/rpy", 10);
    ros::Subscriber enu_sub = n.subscribe<sensor_msgs::Imu>("vnav/imu", 1000, boost::bind(&transform, ned_pub, rpy_pub, _1));
    ros::spin(); 
}