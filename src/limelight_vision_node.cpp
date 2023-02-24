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
#include "ck_utilities/Logger.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

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

void limelight_field_center_static_transform()
{

	geometry::Transform map_to_limelight_field_center;
	map_to_limelight_field_center.linear.x(field_length / 2.0);
	map_to_limelight_field_center.linear.y(-1.0 * field_width / 2.0);
	map_to_limelight_field_center.angular.yaw(M_PI);

	geometry_msgs::TransformStamped map_to_limelight_map_transform;
	map_to_limelight_map_transform.transform = geometry::to_msg(map_to_limelight_field_center);
	map_to_limelight_map_transform.header.frame_id = "odom";
	map_to_limelight_map_transform.child_frame_id = "limelight_map";
	map_to_limelight_map_transform.header.stamp = ros::Time().now();
	tfStaticBroadcaster->sendTransform(map_to_limelight_map_transform);
}

void publish_static_transforms()
{
	limelight_field_center_static_transform();
}

inline bool time_not_timed_out(ros::Time& checkedTime, const double& timeout)
{
	return ((ros::Time::now() - checkedTime) < ros::Duration(timeout));
}

void publish_localization_tf(geometry::Pose bot_pose)
{
    geometry::Transform transform_pose;
    transform_pose.linear = bot_pose.position;
    transform_pose.linear.z(0);
    transform_pose.angular = bot_pose.orientation;
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "limelight_map";
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = "limelight_result";
    transform.transform = geometry::to_msg(transform_pose);

    tfBroadcaster->sendTransform(transform);
}

void publish_localization_data(std::string limelight_name)
{
    std::string bot_json = "";
    ros::Time last_valid_json;
    ck::nt::get(bot_json, last_valid_json, limelight_name, "json", bot_json);

    // we're gonna go crazy here and do back to back outlier rejection across both cameras
    static float last_x = 0;
    static float last_y = 0;
    static float last_yaw = 0;
    static ros::Time last_transmitted = last_valid_json;

    if(last_valid_json > last_transmitted && bot_json.length() > 0)
    {
        last_transmitted = last_valid_json;
        std::map<int, geometry::Pose> poses;
        geometry::Pose bot_pose;

        try
        {
            std::stringstream ss;
            ss << bot_json;
            boost::property_tree::ptree pt;
            boost::property_tree::read_json(ss, pt);

            BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("Results.Fiducial"))
            {
                (void) v;
                std::vector<double> values;
                int marker_id = v.second.get_child("fID").get_value<int>();
                BOOST_FOREACH(boost::property_tree::ptree::value_type &t, v.second.get_child("t6r_fs"))
                {
                    values.push_back(t.second.get_value<double>());
                }

                if (values.size() < 6)
                {
                    ck::log_error << limelight_name << ":Incorrect number of doubles in pose structure" << std::flush;
                    return;
                }

                geometry::Pose pose;
                pose.position.x(values[0]);
                pose.position.y(values[1]);
                pose.position.z(values[2]);
                pose.orientation.roll(values[3]);
                pose.orientation.pitch(values[4]);
                pose.orientation.yaw(values[5]);

                poses[marker_id] = pose;
            }

            std::vector<double> bot_pose_values;
            BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("Results.botpose"))
            {
                bot_pose_values.push_back(v.second.get_value<double>());
            }
            bot_pose.position.x(bot_pose_values[0]);
            bot_pose.position.y(bot_pose_values[1]);
            bot_pose.position.z(bot_pose_values[2]);
            bot_pose.orientation.roll(ck::math::deg2rad(bot_pose_values[3]));
            bot_pose.orientation.pitch(ck::math::deg2rad(bot_pose_values[4]));
            bot_pose.orientation.yaw(ck::math::deg2rad(bot_pose_values[5]));
        }
        catch ( std::exception& ex )
        {
            ck::log_error << limelight_name << ": Exception: " << ex.what();
            ck::log_error << limelight_name << ":Bad JSON received len(" << bot_json.length() << "): " << std::endl << bot_json << std::flush;
            return;
        }
        catch ( ... )
        {
            ck::log_error << limelight_name << ":Bad JSON received len(" << bot_json.length() << "): " << std::endl << bot_json << std::flush;
            return;
        }

        if(poses.size() < 1)
        {
            return;
        }



        // std::vector<geometry::Pose> passing_poses;

        // for(auto &i : poses)
        // {
        //     auto &pose = i.second;

        //     if (pose.position.z() > 0.1)
        //     {
        //         ck::log_debug << "Rejecting pose with z: " << pose.position.z() << std::flush;
        //         continue;
        //     }
        //     if (pose.orientation.pitch() > ck::math::deg2rad(5.0))
        //     {
        //         ck::log_debug << "Rejecting pose with pitch: " << ck::math::rad2deg(pose.orientation.pitch()) << std::flush;
        //         continue;
        //     }
        //     if (pose.orientation.roll() > ck::math::deg2rad(5.0))
        //     {
        //         ck::log_debug << "Rejecting pose with roll: " << ck::math::rad2deg(pose.orientation.roll()) << std::flush;
        //         continue;
        //     }
        //     passing_poses.push_back(pose);
        // }

        // if (passing_poses.size() < 1)
        // {
        //     ck::log_debug << "No passing poses" << std::flush;
        //     return;
        // }

        // geometry::Pose average_pose = passing_poses[0];

        // for (size_t i = 1; i < passing_poses.size(); i++)
        // {
        //     average_pose = average_pose + passing_poses[i];
        // }

        // average_pose = average_pose / passing_poses.size();

        if (std::abs(bot_pose.position.z()) > 0.1)
        {
            ck::log_debug << "Rejecting pose with z: " << bot_pose.position.z() << std::flush;
            return;
        }
        if (std::abs(bot_pose.orientation.pitch()) > ck::math::deg2rad(5.0))
        {
            ck::log_debug << "Rejecting pose with pitch: " << ck::math::rad2deg(bot_pose.orientation.pitch()) << std::flush;
            return;
        }
        if (std::abs(bot_pose.orientation.roll()) > ck::math::deg2rad(5.0))
        {
            ck::log_debug << "Rejecting pose with roll: " << ck::math::rad2deg(bot_pose.orientation.roll()) << std::flush;
            return;
        }

        bool reject = false;
        reject = reject || std::abs(last_x - bot_pose.position.x()) > 0.5;
        reject = reject || std::abs(last_y - bot_pose.position.y()) > 0.5;
        reject = reject || std::abs(ck::math::rad2deg(last_yaw - bot_pose.orientation.yaw())) > 20.0;

        last_x = bot_pose.position.x();
        last_y = bot_pose.position.y();
        last_yaw = bot_pose.orientation.yaw();

        static uint32_t match_count = 0;

        if (reject)
        {
            match_count = 0;
            ck::log_debug << limelight_name << ": Filter reject" << std::flush;
            return;
        }

        match_count ++;

        (void) match_count;
        if (match_count < 2)
        {
            ck::log_debug << limelight_name << ": Filter reject" << std::flush;
            return;
        }

        nav_msgs::Odometry odom_data;
        odom_data.header.stamp = ros::Time().now();
        odom_data.header.frame_id = "limelight_map";
        odom_data.child_frame_id = "limelight_map";
        odom_data.pose.pose = geometry::to_msg(bot_pose);

        geometry::Covariance limelight_covariance;
        limelight_covariance.x_var(1.0);
        limelight_covariance.y_var(1.0);
        odom_data.pose.covariance = geometry::to_msg(limelight_covariance);

        static ros::Publisher odom_pub = node->advertise<nav_msgs::Odometry>("LimelightOdometry", 100);
        odom_pub.publish(odom_data);

        publish_localization_tf(bot_pose);
    }
}

limelight_vision_node::Limelight_Info process_limelight_data(std::string limelight_name)
{
    publish_localization_data(limelight_name);

    limelight_vision_node::Limelight_Info limelightInfo;
    limelightInfo.name = limelight_name;

    ros::Time response_time(0);
    double default_val = 0;
    bool limelight_data_valid = false;
    const double timeout = 0.2;

    double tv = 0;
    ck::nt::get(tv, response_time, limelight_name, "tv", default_val);
    limelightInfo.target_valid = tv > 0 ? true : false;
    limelight_data_valid |= time_not_timed_out(response_time, timeout);

    ck::nt::get(limelightInfo.target_dx_deg, response_time, limelight_name, "tx", default_val);
    limelightInfo.target_dx_deg = -limelightInfo.target_dx_deg;
    limelight_data_valid |= time_not_timed_out(response_time, timeout);

    ck::nt::get(limelightInfo.target_dy_deg, response_time, limelight_name, "ty", default_val);
    limelightInfo.target_dy_deg = -limelightInfo.target_dy_deg;
    limelight_data_valid |= time_not_timed_out(response_time, timeout);

    ck::nt::get(limelightInfo.target_area, response_time, limelight_name, "ta", default_val);
    limelight_data_valid |= time_not_timed_out(response_time, timeout);

    ck::nt::get(limelightInfo.target_skew, response_time, limelight_name, "ts", default_val);
    limelight_data_valid |= time_not_timed_out(response_time, timeout);

    ck::nt::get(limelightInfo.target_latency, response_time, limelight_name, "tl", default_val);
    limelight_data_valid |= time_not_timed_out(response_time, timeout);

    limelightInfo.target_valid &= limelight_data_valid;

    return limelightInfo;
}

void publish_limelight_data()
{
	static ros::Publisher limelight_pub = node->advertise<limelight_vision_node::Limelight_Status>("LimelightStatus", 1);
	limelight_vision_node::Limelight_Status limelightStatus;

	ros::Rate rate(100);

	while (ros::ok())
	{
        limelightStatus.limelights.push_back(process_limelight_data("limelight-front"));
        limelightStatus.limelights.push_back(process_limelight_data("limelight-back"));
        limelight_pub.publish(limelightStatus);
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

	std::thread limelightSendThread(publish_limelight_data);
	ros::Subscriber limelightControl = node->subscribe("/LimelightControl", 100, limelightControlCallback);

	ros::spin();

	limelightSendThread.join();

	return 0;
}