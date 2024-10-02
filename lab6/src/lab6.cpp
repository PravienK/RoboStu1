#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class ScanToImageNode : public rclcpp::Node
{
public:
    ScanToImageNode() : Node("scan_to_image_node"), angle_difference_(0.0), relative_orientation_(0.0)
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanToImageNode::scanCallback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ScanToImageNode::odomCallback, this, std::placeholders::_1));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "Scan to Image Node started.");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        cv::Mat current_image = laserScanToMat(msg);

        if (first_image_.empty()) //first scan stores as a first image 
        {
            first_image_ = current_image;
            cv::imshow("First Image", first_image_);
            cv::waitKey(1);
            return;
        }

        if (second_image_.empty()) //second scan stores as a second image
        {
            second_image_ = current_image;
            cv::imshow("Second Image", second_image_);
            cv::waitKey(1);
            calculateYawChange(); //calculates the yaw in these images
            return;
        }

        first_image_ = second_image_;
        second_image_ = current_image;
        calculateYawChange();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update the robot's pose using odometry data
        current_pose_ = msg->pose.pose;

        // Extract section of the map around the robot
        cv::Mat map_section = extractMapSection(current_pose_);

        // Extract edges from the map section
        cv::Mat edge_image;
        cv::Canny(map_section, edge_image, 50, 150);

        // Visualize the edges
        cv::imshow("Edge Image", edge_image);
        cv::waitKey(1);
    }

    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr &scan)
    {
        cv::Mat image = cv::Mat::zeros(500, 500, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max)
            {
                continue;
            }
            float angle = scan->angle_min + i * scan->angle_increment;
            int x = static_cast<int>(250 + range * cos(angle) * 100);
            int y = static_cast<int>(250 + range * sin(angle) * 100);
            if (x >= 0 && x < 500 && y >= 0 && y < 500)
            {
                image.at<uchar>(y, x) = 255;
            }
        }
        return image;
    }

    cv::Mat extractMapSection(const geometry_msgs::msg::Pose &pose)
    {
  
        cv::Mat map_section = cv::Mat::zeros(500, 500, CV_8UC1);
        return map_section;
    }

    void calculateYawChange()
    {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_, second_image_, srcPoints, dstPoints);

        if (!srcPoints.empty() && !dstPoints.empty())
        {
            cv::Mat affineMatrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);

            if (!affineMatrix.empty())
            {
                double angle = atan2(affineMatrix.at<double>(1, 0), affineMatrix.at<double>(0, 0)) * 180.0 / CV_PI;
                relative_orientation_ += angle;
                RCLCPP_INFO(this->get_logger(), "Estimated Yaw Change: %f degrees", angle);
                RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f degrees", relative_orientation_);

                // Propagate the robot to a new location
                propagateRobot();
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Affine transformation matrix is empty.");
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No matching points found.");
        }
    }

    void detectAndMatchFeatures(const cv::Mat &img1, const cv::Mat &img2,
                                std::vector<cv::Point2f> &srcPoints, std::vector<cv::Point2f> &dstPoints)
    {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch &a, const cv::DMatch &b)
                  { return a.distance < b.distance; });

        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        matches.resize(numGoodMatches);

        for (const auto &match : matches)
        {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    void propagateRobot()
    {
        // Publish velocity commands to move the robot to a new location
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.1;  // Move forward
        cmd_vel.angular.z = 0.0; // No rotation
        cmd_publisher_->publish(cmd_vel);

        // Sleep for a short duration to allow the robot to move
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Stop the robot
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_publisher_->publish(cmd_vel);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    cv::Mat first_image_, second_image_;
    double angle_difference_;
    double relative_orientation_;
    geometry_msgs::msg::Pose current_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToImageNode>());
    rclcpp::shutdown();
    return 0;
}
