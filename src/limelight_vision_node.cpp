#include "ros/ros.h"
#include "std_msgs/String.h"

#include "limelight_vision_node/Limelight_Info.h"
#include "limelight_vision_node/Limelight_Status.h"
#include "limelight_vision_node/Limelight.h"
#include "limelight_vision_node/Limelight_Control.h"

#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ck_utilities/geometry/geometry.hpp>
#include <ck_utilities/geometry/geometry_ros_helpers.hpp>
#include "ck_utilities/NTHelper.hpp"
#include "ck_utilities/ParameterHelper.hpp"
#include "ck_utilities/CKMath.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <vector>
#include <sstream>

ros::NodeHandle* node;
std::mutex limelightMutex;
std::vector<std::string> limelight_names;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformBroadcaster* tfBroadcaster;
tf2_ros::StaticTransformBroadcaster * tfStaticBroadcaster;
tf2_ros::TransformListener* tfListener;

static double field_length = ck::math::feet_to_meters(54.0) + ck::math::inches_to_meters(3.25);
static double field_width = ck::math::feet_to_meters(26) + ck::math::inches_to_meters(3.5);

void fwd_limelight_static_transform()
{
	geometry::Transform base_link_to_fwd_limelight_base;
	base_link_to_fwd_limelight_base.linear.x(0.5);
	base_link_to_fwd_limelight_base.linear.y(0.0);
	base_link_to_fwd_limelight_base.linear.z(0.5);

	geometry_msgs::TransformStamped baselink_to_fwd_limelight_base_transform;
	baselink_to_fwd_limelight_base_transform.transform = geometry::to_msg(base_link_to_fwd_limelight_base);
	baselink_to_fwd_limelight_base_transform.header.frame_id = "base_link";
	baselink_to_fwd_limelight_base_transform.child_frame_id = "fwd_limelight_base";
	baselink_to_fwd_limelight_base_transform.header.stamp = ros::Time().now();
	tfStaticBroadcaster->sendTransform(baselink_to_fwd_limelight_base_transform);

	geometry::Transform base_link_to_fwd_limelight = base_link_to_fwd_limelight_base;
	base_link_to_fwd_limelight.angular.yaw();
	base_link_to_fwd_limelight.angular.pitch();
	base_link_to_fwd_limelight.angular.roll();

	geometry_msgs::TransformStamped baselink_to_fwd_limelight_transform;
	baselink_to_fwd_limelight_transform.transform = geometry::to_msg(base_link_to_fwd_limelight);
	baselink_to_fwd_limelight_transform.header.frame_id = "base_link";
	baselink_to_fwd_limelight_transform.child_frame_id = "fwd_limelight";
	baselink_to_fwd_limelight_transform.header.stamp = ros::Time().now();
	tfStaticBroadcaster->sendTransform(baselink_to_fwd_limelight_transform);
}

void rev_limelight_static_transform()
{
	geometry::Transform base_link_to_rev_limelight_base;
	base_link_to_rev_limelight_base.linear.x(-0.5);
	base_link_to_rev_limelight_base.linear.y(0.0);
	base_link_to_rev_limelight_base.linear.z(0.5);

	geometry_msgs::TransformStamped baselink_to_rev_limelight_base_transform;
	baselink_to_rev_limelight_base_transform.transform = geometry::to_msg(base_link_to_rev_limelight_base);
	baselink_to_rev_limelight_base_transform.header.frame_id = "base_link";
	baselink_to_rev_limelight_base_transform.child_frame_id = "rev_limelight_base";
	baselink_to_rev_limelight_base_transform.header.stamp = ros::Time().now();
	tfStaticBroadcaster->sendTransform(baselink_to_rev_limelight_base_transform);

	geometry::Transform base_link_to_rev_limelight = base_link_to_rev_limelight_base;
	base_link_to_rev_limelight.angular.yaw();
	base_link_to_rev_limelight.angular.pitch();
	base_link_to_rev_limelight.angular.roll();

	geometry_msgs::TransformStamped baselink_to_rev_limelight_transform;
	baselink_to_rev_limelight_transform.transform = geometry::to_msg(base_link_to_rev_limelight);
	baselink_to_rev_limelight_transform.header.frame_id = "base_link";
	baselink_to_rev_limelight_transform.child_frame_id = "rev_limelight";
	baselink_to_rev_limelight_transform.header.stamp = ros::Time().now();
	tfStaticBroadcaster->sendTransform(baselink_to_rev_limelight_transform);
}

void limelight_field_center_static_transform()
{

	geometry::Transform map_to_limelight_field_center;
	map_to_limelight_field_center.linear.x(field_length / 2.0);
	map_to_limelight_field_center.linear.y(-1.0 * field_width / 2.0);
	map_to_limelight_field_center.angular.yaw(M_PI);

	geometry_msgs::TransformStamped map_to_limelight_map_transform;
	map_to_limelight_map_transform.transform = geometry::to_msg(map_to_limelight_field_center);
	map_to_limelight_map_transform.header.frame_id = "map";
	map_to_limelight_map_transform.child_frame_id = "limelight_map";
	map_to_limelight_map_transform.header.stamp = ros::Time().now();
	tfStaticBroadcaster->sendTransform(map_to_limelight_map_transform);
}

void publish_static_transforms()
{
	limelight_field_center_static_transform();
	fwd_limelight_static_transform();
	rev_limelight_static_transform();
}

inline bool time_not_timed_out(ros::Time& checkedTime, const double& timeout)
{
	return ((ros::Time::now() - checkedTime) < ros::Duration(timeout));
}

void publish_limelight_data()
{
	static ros::Publisher limelight_pub = node->advertise<limelight_vision_node::Limelight_Status>("LimelightStatus", 1);
	static limelight_vision_node::Limelight_Status limelightStatus;

	ros::Rate rate(100);

	while (ros::ok())
	{
		// ROS_INFO("Running");
		std::vector<double> bot_pose;
		ros::Time last_valid;
		ck::nt::get(bot_pose, last_valid, "limelight", "botpose", bot_pose);
		double total = 0;
		for(auto& i : bot_pose)
		{
			total += i;
		}
		static ros::Time last_transmitted = last_valid;
		if(bot_pose.size() == 6 && total != 0 && last_valid > last_transmitted)
		{
			geometry::Pose robot_pose;
			robot_pose.position.x(bot_pose[0]);
			robot_pose.position.y(bot_pose[1]);
			robot_pose.position.z(bot_pose[2]);
			robot_pose.orientation.roll(ck::math::deg2rad(bot_pose[3]));
			robot_pose.orientation.pitch(ck::math::deg2rad(bot_pose[4]));
			robot_pose.orientation.yaw(ck::math::deg2rad(bot_pose[5]));

			std::stringstream s;
			s << robot_pose;
			ROS_INFO("Pose: %s", s.str().c_str());

			nav_msgs::Odometry odom_data;
			odom_data.header.stamp = ros::Time().now();
			odom_data.header.frame_id = "limelight_map";
			odom_data.child_frame_id = "fwd_limelight";
			odom_data.pose.pose = geometry::to_msg(robot_pose);
			odom_data.pose.covariance =
				 { 0.08, 0.0,  0.0,  0.0,  0.0,  0.0,
					0.0, 0.08, 0.0,  0.0,  0.0,  0.0,
					0.0, 0.0,  0.08, 0.0,  0.0,  0.0,
					0.0, 0.0,  0.0,  0.08, 0.0,  0.0,
					0.0, 0.0,  0.0,  0.0,  0.08, 0.0,
					0.0, 0.0,  0.0,  0.0,  0.0,  0.08,};

			static ros::Publisher odom_pub = node->advertise<nav_msgs::Odometry>("LimelightOdometry", 100);
			odom_pub.publish(odom_data);
			last_transmitted = last_valid;
		}
		rate.sleep();
	}
}

void limelightControlCallback(const limelight_vision_node::Limelight_Control& msg)
{
	for (const limelight_vision_node::Limelight& ll : msg.limelights)
	{
		bool setSuccess = true;
		setSuccess &= ck::nt::set(ll.name, "ledMode", ll.ledMode);
		setSuccess &= ck::nt::set(ll.name, "camMode", ll.camMode);
		setSuccess &= ck::nt::set(ll.name, "pipeline", ll.pipeline);
		setSuccess &= ck::nt::set(ll.name, "stream", ll.stream);
		setSuccess &= ck::nt::set(ll.name, "snapshot", ll.snapshot);

		if (!setSuccess)
		{
			ROS_WARN("Failed to set values for limelight: %s", ll.name.c_str());
		}
	}
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "limelight_vision_node");

	ros::NodeHandle n;
	node = &n;
	tfBroadcaster = new tf2_ros::TransformBroadcaster();
	tfStaticBroadcaster = new tf2_ros::StaticTransformBroadcaster();
	tfListener = new tf2_ros::TransformListener(tfBuffer);
	publish_static_transforms();

	// bool required_params_found = true;
	// required_params_found &= n.getParam(CKSP(limelight_names), limelight_names);
	// if (!required_params_found)
	// {
	// 	ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
	// 	return 1;
	// }

	std::thread limelightSendThread(publish_limelight_data);
	// ros::Subscriber limelightControl = node->subscribe("/LimelightControl", 100, limelightControlCallback);

	ros::spin();

	limelightSendThread.join();

	return 0;
}