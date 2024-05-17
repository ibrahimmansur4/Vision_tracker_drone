#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class RobotCamera : public rclcpp::Node
{
public:
    RobotCamera() : Node("Camera23")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&RobotCamera::camera_callback, this, std::placeholders::_1));
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        cv::namedWindow("Camera Output");
    }

    ~RobotCamera()
    {
        cv::destroyWindow("Camera Output");
    }

private:
    void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat cap = cv_ptr->image;

        // Detecting the black region
        cv::Scalar lower_black(0, 0, 0);
        cv::Scalar upper_black(0, 0, 44);

        // Converting the HSV of image obtained from the bridge
        cv::Mat hsv;
        cv::cvtColor(cap, hsv, cv::COLOR_BGR2HSV);
        cv::Mat mask;
        cv::inRange(hsv, lower_black, upper_black, mask);
        cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)), cv::Point(-1, -1), 9);
        cv::medianBlur(mask, mask, 7);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)), cv::Point(-1, -1), 1);

        cv::Mat thresh;
        cv::threshold(mask, thresh, 127, 255, cv::THRESH_BINARY);
        std::vector<std::vector<cv::Point>> cnts;
        cv::findContours(thresh, cnts, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        int frame_center_x = cap.cols / 2;
        int frame_center_y = cap.rows / 2;

        if (!cnts.empty())
        {
            std::vector<cv::Point> c = *std::max_element(cnts.begin(), cnts.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b)
            {
                return cv::contourArea(a) < cv::contourArea(b);
            });

            cv::Rect rect = cv::boundingRect(c);
            cv::Point center = (rect.tl() + rect.br()) / 2;
            cv::circle(cap, center, rect.width / 2, cv::Scalar(0, 0, 0), 2);
            cv::putText(cap, "X : " + std::to_string(center.x), cv::Point(center.x + rect.width / 2 + 5, center.y), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            cv::putText(cap, "Y : " + std::to_string(center.y), cv::Point(center.x + rect.width / 2 + 5, center.y + 35), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            cv::line(cap, cv::Point(frame_center_x, frame_center_y), center, cv::Scalar(0, 0, 0), 3);

            // Alignment Tracking
            int error_x = center.x - frame_center_x;

            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.2;
            twist.angular.z = -error_x / 100.0;
            pub_->publish(twist);

            if (rect.width > 150)
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                pub_->publish(twist);
            }
        }
        else
        {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            pub_->publish(twist);
        }

        cv::imshow("Camera Output", cap);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCamera>());
    rclcpp::shutdown();
    return 0;
}