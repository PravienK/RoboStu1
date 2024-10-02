#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <iostream>

class Lidar : public rclcpp::Node
{
public:
    Lidar() : Node("lidar_processor")
    {
        // Create a subscriber to the laser scan topic
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&Lidar::scanCallback, this, std::placeholders::_1));

        // Create a publisher for the filtered laser scan data
        data_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);

    }

private:
    void scanCallback(const std::shared_ptr<sensor_msgs::msg::LaserScan> scan)
    {
        // Create a new LaserScan message for the filtered data
        auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan); //creates a new LaserScan message named new_scan
        std::vector<float> modified_ranges; // hold the filtered range values

        // Set the filtering interval (nth point)
        size_t n = 10; // Change 'n' to any other value to filter at different intervals

        // Filter the laser scan data by taking every nth point
        for (size_t i = 0; i < scan->ranges.size(); i += n)
        {
            modified_ranges.push_back(scan->ranges[i]);
        }

        // Assign the filtered ranges to the new scan message
        new_scan->ranges = modified_ranges;

        // Update the angle_increment accordingly
        new_scan->angle_increment = scan->angle_increment * n;

        // Publish the filtered laser scan data
        data_publisher_->publish(*new_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr data_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Lidar>());
    rclcpp::shutdown();
    return 0;
}
