
#ifndef VISION_HPP
#define VISION_HPP


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "image_transport/image_transport.hpp"
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include <cv_bridge/cv_bridge.h>
//#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/aruco.hpp>
#include <aruco.h>
#include <aruco/ippe.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "dots_interfaces/msg/tag.hpp"
#include "dots_interfaces/msg/tag_array.hpp"

class Dots_process_cam : public rclcpp::Node
{
public:
    Dots_process_cam(std::string name);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr        img_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr           img_pub;
    rclcpp::Publisher<dots_interfaces::msg::TagArray>::SharedPtr    tags_pub;

    void send_transform(cv::Mat &rvec, cv::Mat &tvec, int id);
    void img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    const double k1 =  0;
    const double k2 =  0;
    const double k3 =  0;
    const double p1 =  0;
    const double p2 =  0;
    
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 185, 0, 320, 0, 185, 240, 0, 0, 1);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);



    aruco::MarkerDetector                           detector;
    std::map<uint32_t, aruco::MarkerPoseTracker>    mtracker;
    aruco::MarkerMap                                mmap;
    aruco::MarkerMapPoseTracker                     mmtracker;
    aruco::CameraParameters                         camera;

    std::string                             frame_prefix;
    std::string                             cam_name;
    std::string                             cam_cal;
    std::string                             cam_link_frame;
    std::string                             marker_map;

    tf2::BufferCore                         bc;
    std::shared_ptr<tf2_ros::TransformBroadcaster>  br;

    std::map<int, double>                   msizes;
};



#endif