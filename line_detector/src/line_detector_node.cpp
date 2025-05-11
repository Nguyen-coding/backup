#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <cmath>

int a = 0;

class LineDetectorNode : public rclcpp::Node {
public:
    LineDetectorNode() : Node("line_detector_node") {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("lowest_line_height", 10);

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&LineDetectorNode::imageCallback, this, std::placeholders::_1)
        );
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge 예외: %s", e.what());
            return;
        }

        if (frame.empty()) return;

        cv::Mat gray, edge;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 2, 2);
        cv::Canny(gray, edge, 50, 150, 3);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(edge, lines, 1, CV_PI/180, 50, 50, 10);

        double minAngle = -10.0 * (CV_PI / 180.0);
        double maxAngle = 10.0 * (CV_PI / 180.0);

        int lowestLineY = 0;
        std::vector<cv::Vec4i> horizontalLines;

        for (const auto& l : lines) {
            double angle = atan2(l[3] - l[1], l[2] - l[0]);
            if (angle > minAngle && angle < maxAngle) {
                int lineLength = std::hypot(l[2] - l[0], l[3] - l[1]);
                if (lineLength > 100) {
                    horizontalLines.push_back(l);
                    lowestLineY = std::max(lowestLineY, std::max(l[1], l[3]));
                }
            }
        }

        std::vector<cv::Vec4i> mergedLines;
        const int heightThreshold = 10;
        for (const auto& l1 : horizontalLines) {
            bool merged = false;
            for (auto& l2 : mergedLines) {
                if ((std::abs(l1[1] - l2[1]) < heightThreshold && std::abs(l1[3] - l2[3]) < heightThreshold) ||
                    (std::abs(l1[1] - l2[3]) < heightThreshold && std::abs(l1[3] - l2[1]) < heightThreshold)) {
                    l2[0] = std::min(l2[0], l1[0]);
                    l2[1] = (l2[1] + l1[1]) / 2;
                    l2[2] = std::max(l2[2], l1[2]);
                    l2[3] = (l2[3] + l1[3]) / 2;
                    merged = true;
                    break;
                }
            }
            if (!merged) mergedLines.push_back(l1);
        }

        for (const auto& l : mergedLines) {
            cv::line(frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        }

        if (lowestLineY > 0) {
            RCLCPP_INFO(this->get_logger(), "가장 낮은 가로선의 높이: %d", lowestLineY);
            std_msgs::msg::Int32 msg;
            if (lowestLineY > 500) a++;
            msg.data = (a > 0) ? 0 : 1;
            RCLCPP_INFO(this->get_logger(), "메세지 값: %d a 값 %d", msg.data, a);
            publisher_->publish(msg);
        }

        cv::imshow("Frame", frame);
        cv::waitKey(1);
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
