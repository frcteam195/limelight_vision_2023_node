#include "ros/ros.h"
#include "std_msgs/String.h"

#include "limelight_vision_node/Limelight_Info.h"
#include "limelight_vision_node/Limelight_Status.h"
#include "limelight_vision_node/Limelight.h"
#include "limelight_vision_node/Limelight_Control.h"

#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "ck_utilities/NTHelper.hpp"
#include "ck_utilities/ParameterHelper.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <vector>

ros::NodeHandle* node;
std::mutex limelightMutex;
std::vector<std::string> limelight_names;

tf2_ros::TransformBroadcaster* tfBroadcaster;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

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
	tfListener = new tf2_ros::TransformListener(tfBuffer);

	bool required_params_found = true;
	required_params_found &= n.getParam(CKSP(limelight_names), limelight_names);
	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

	std::thread limelightSendThread(publish_limelight_data);
	ros::Subscriber limelightControl = node->subscribe("/LimelightControl", 100, limelightControlCallback);

	ros::spin();

	limelightSendThread.join();

	return 0;
}