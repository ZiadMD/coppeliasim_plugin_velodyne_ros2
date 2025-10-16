#include "../include/coppeliasim_plugin_velodyne/ros_server_velodyne.h"
#include "../include/v_repLib.h"

rclcpp::Node::SharedPtr ROS_server::node = nullptr;

// Publishers:
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ROS_server::pointCloud_publisher;

bool ROS_server::initialize()
{
    // Get or attach to the existing global ROS 2 context
    auto context = rclcpp::contexts::get_global_default_context();

    if (!context->is_valid()) {
        // Try to attach gracefully — do NOT call rclcpp::init()
        int argc = 0;
        char** argv = nullptr;
        try {
            context->init(argc, argv);
        } catch (const std::exception& e) {
            printf("[ROS_server] Failed to attach to ROS2 context: %s\n", e.what());
            return false;
        }
    }

    try {
        node = std::make_shared<rclcpp::Node>(
            "vrep_velodyne",
            rclcpp::NodeOptions().context(context)
        );

        pointCloud_publisher = node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/velodyne/points2",
            10
        );
    }
    catch (const std::exception& e) {
        printf("[ROS_server] Failed to create node or publisher: %s\n", e.what());
        return false;
    }

    printf("[ROS_server] Initialized and attached to existing ROS 2 context.\n");
    return true;
}

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ROS_server::getPublisher()
{
    return ROS_server::pointCloud_publisher;
}

rclcpp::Node::SharedPtr ROS_server::getNode()
{
    return ROS_server::node;
}

void ROS_server::shutDown()
{
	// Disable the publishers:
    pointCloud_publisher.reset();

	// Shut down:
	// if (rclcpp::ok()) {
    //     rclcpp::shutdown();
    // } Fix the issue #69

    node.reset();
    printf("[ROS_server] Velodyne ROS2 node shutdown complete.\n");
}
