#ifndef ROS_SERVER_H
#define ROS_SERVER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>

class ROS_server
{
	public:
		static bool initialize();
		static void shutDown();

        static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr getPublisher();
        static rclcpp::Node::SharedPtr getNode();

	private:
        ROS_server() {}
		
		static rclcpp::Node::SharedPtr node;

        // Publishers:
        static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_publisher;

};

#endif
