#include <stdlib.h>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/float32.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter : public rclcpp::Node {
private:
    const std::string OPENCV_WINDOW = "Image window";
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr current_angle_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_angle_pub_;
    
    double current_angle_ = 0.0;
    double target_center_ = 320.0;  // Assume 640px width, center at 320
    bool object_detected_ = false;
    rclcpp::Time last_detection_time_;
    double search_angle_direction_ = 1.0;

    void currentAngleCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_angle_ = msg->data;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        target_center_ = cv_ptr->image.cols / 2.0;

        //convert bgr -> hsv
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
        
        cv::Mat red_mask;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask);
        
        std::vector<cv::Point> red_pixels;
        for (int y = 0; y < red_mask.rows; y++) {
            for (int x = 0; x < red_mask.cols; x++) {
                if (red_mask.at<uchar>(y, x) > 0) {  // If pixel is red
                    red_pixels.push_back(cv::Point(x, y));
                }
            }
        }
        
        auto desired_angle_msg = std_msgs::msg::Float32();
        
        if (!red_pixels.empty()) {
            int sum_x = 0;
            for (const auto& pixel : red_pixels) {
                sum_x += pixel.x;
            }
            int average_x = sum_x / red_pixels.size();
            
            object_detected_ = true;
            last_detection_time_ = this->now();
            
            double error = average_x - target_center_;
            double tolerance = 10.0; // pixels
            
            if (std::abs(error) > tolerance) {
                double angle_adjustment = -error * 0.1 * (3.14159 / 180.0); 
                desired_angle_msg.data = current_angle_ + angle_adjustment;
                desired_angle_pub_->publish(desired_angle_msg);
            } else {
                desired_angle_msg.data = current_angle_;
                desired_angle_pub_->publish(desired_angle_msg);
            }
            
        } else {
            object_detected_ = false;
            

            double search_speed = 1; // degrees per frame
            desired_angle_msg.data = current_angle_ + search_angle_direction_ * search_speed * (3.14159 / 180.0);
            desired_angle_pub_->publish(desired_angle_msg);
            
            if (std::abs(current_angle_) > 1.57) 
                search_angle_direction_ *= -1.0;
        
        }

        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        pub_.publish(cv_ptr->toImageMsg());
    }

public:
    ImageConverter() : Node("opencv_image_publisher") {

        // Open demo window that will show output image
        cv::namedWindow(OPENCV_WINDOW);

        last_detection_time_ = this->now();

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        pub_ = image_transport::create_publisher(this, "out_image_base_topic", custom_qos);
        sub_ = image_transport::create_subscription(this, "/robotcam",
                std::bind(&ImageConverter::imageCallback, this, std::placeholders::_1), "raw", custom_qos);
        
        current_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/current_angle", 10, std::bind(&ImageConverter::currentAngleCallback, this, std::placeholders::_1));
        
        desired_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
}