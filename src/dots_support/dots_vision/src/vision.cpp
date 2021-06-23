#include <cstdio>

#include "dots_vision/vision.hpp"

using std::placeholders::_1;


//https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
#include <memory>
template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}



// Data on image topic is processed using the Aruco library and any tags
// identified are output as an annotated image and as ??



Dots_process_cam::Dots_process_cam(std::string name) 
: Node(name)
{

    frame_prefix    = this->declare_parameter("frame_prefix", "");
    cam_name        = this->declare_parameter("cam_name", "cam0");
    cam_link_frame  = frame_prefix + cam_name + "_link_optical";

    cam_cal         = this->declare_parameter("cam_cal", "cam0.yaml");
    marker_map      = this->declare_parameter("marker_map", "");

    RCLCPP_INFO(this->get_logger(), "cam_frame %s", cam_link_frame.c_str());

    img_sub     = this->create_subscription<sensor_msgs::msg::Image>("img_in", rclcpp::SensorDataQoS(), 
                        std::bind(&Dots_process_cam::img_sub_callback, this, _1));
    img_pub     = this->create_publisher<sensor_msgs::msg::Image>("img_out", rclcpp::SensorDataQoS());

    tags_pub    = this->create_publisher<dots_interfaces::msg::TagArray>("tags_out", rclcpp::SystemDefaultsQoS());


    // FIXME read from yaml
    camera.setParams(camera_matrix, dist_coeffs, cv::Size(640, 480));

    detector.setDictionary("ARUCO_MIP_36h12");
    detector.setDetectionMode(aruco::DM_FAST, 0.01);

    if (marker_map.size())
    {
        mmap.readFromFile(marker_map.c_str());
        mmtracker.setParams(camera, mmap);
    }


    // Make the transform broadcaster
    br = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Marker size map

    msizes[0] = 0.24;
    msizes[1] = 0.24;
    msizes[2] = 0.24;
    msizes[3] = 0.24;
    msizes[4] = 0.24;
    msizes[5] = 0.24;
    msizes[6] = 0.24;
    msizes[7] = 0.24;
    msizes[8] = 0.24;
    msizes[9] = 0.24;
    msizes[10] = 0.24;
    msizes[11] = 0.24;
    msizes[100] = 0.04;
    msizes[101] = 0.04;
    msizes[102] = 0.04;
    msizes[103] = 0.04;
    msizes[104] = 0.04;
    msizes[105] = 0.04;
    msizes[200] = 0.0234;
    msizes[201] = 0.0234;

}

void Dots_process_cam::send_transform(cv::Mat &rvec, cv::Mat &tvec, int id)
{
    // Pose is in form of Tvec and Rvec 3 element tuples, representing
    // transform and rotation.
    //
    // We need to turn this into a tf transform, from the camera to the 
    // detected marker..
    //
    // https://stackoverflow.com/questions/46363618/aruco-markers-with-opencv-get-the-3d-corner-coordinates?rq=1
    //

    cv::Mat rot(3, 3, CV_32FC1);
    cv::Rodrigues(rvec, rot);
    tf2::Matrix3x3 rrot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
                        rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
                        rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));
    tf2::Quaternion q;

 

    rrot.getRotation(q);
    if (std::isnan(q.x()) || std::isnan(q.y()) || std::isnan(q.z()) || std::isnan(q.w()))
        return;

    // Get transform from cam to fiducial
    //tf2::BufferCore bc;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id           = cam_link_frame;
    // FIXME compensate for camera system latency here
    t.header.stamp              = this->now();
    t.child_frame_id            = frame_prefix + cam_name + string_format("_fid%03d", id);
    t.transform.translation.x   = tvec.ptr<float>(0)[0];
    t.transform.translation.y   = tvec.ptr<float>(0)[1];
    t.transform.translation.z   = tvec.ptr<float>(0)[2];
    t.transform.rotation.x      = q.x();
    t.transform.rotation.y      = q.y();
    t.transform.rotation.z      = q.z();
    t.transform.rotation.w      = q.w();
    bc.setTransform(t, "default_authority", true);

    // Send the transform
    br->sendTransform(t);
}

void Dots_process_cam::img_sub_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "In image callback");


    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


    dots_interfaces::msg::TagArray tag_list_msg;

    //auto start = std::chrono::steady_clock::now();
    std::vector<aruco::Marker> markers = detector.detect(cv_ptr->image);
    //auto end = std::chrono::steady_clock::now();
    //auto t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    //RCLCPP_INFO(this->get_logger(), "Detected %3d %d", markers.size(), t);


    if (marker_map.size())
    {
        if (mmtracker.isValid())
        {
            if (mmtracker.estimatePose(markers))
            {
                // FIXME temp use this id to indicate target
                dots_interfaces::msg::Tag tag;
                tag.id = 200;
                tag_list_msg.data.push_back(tag);
                auto rvec = mmtracker.getRvec();
                auto tvec = mmtracker.getTvec();

                aruco::CvDrawingUtils::draw3dAxis(cv_ptr->image, camera, rvec, tvec, 0.3);

                // RCLCPP_INFO(this->get_logger(), "tvec %f %f %f", tvec.ptr<float>(0)[0], 
                //     tvec.ptr<float>(0)[1],tvec.ptr<float>(0)[2]);
                send_transform(rvec, tvec, 200);
            }
        }

    }
    else
    {
        // Calculate extrinsics outside detector so we can use different size tags
        for (auto &m : markers)
        {
            double marker_size = 0.05;
            auto it = msizes.find(m.id);
            if (it != msizes.end())
                marker_size = it->second;


            m.calculateExtrinsics(marker_size, camera, false);

            if (m.isPoseValid() && (m.id < 200))
            {

                float hl = m[3].y - m[0].y;
                float hr = m[2].y - m[1].y;
                float alignment = (hl - hr) / (hl + hr);
                // if ((m.id == 100) && (cam_name == "cam0")) 
                // {
                //     RCLCPP_INFO(this->get_logger(), "(% 6f, %06f) (% 6f, %06f) (% 6f, %06f) (% 6f, %06f) % 6f %6d", m[0].x, m[0].y, m[1].x, m[1].y, m[2].x, m[2].y, m[3].x, m[3].y, alignment, t);
                // }
                //aruco::CvDrawingUtils::draw3dAxis(cv_ptr->image, camera, m.Rvec, m.Tvec, 0.3);
                cv::aruco::drawAxis(cv_ptr->image, camera_matrix, dist_coeffs, m.Rvec, m.Tvec, 0.3);
                m.draw(cv_ptr->image);
                send_transform(m.Rvec, m.Tvec, m.id);

                dots_interfaces::msg::Tag tag;
                tag.id = m.id;
                tag.alignment = alignment;
                tag.points[0].x = m[0].x;
                tag.points[0].y = m[0].y;
                tag.points[1].x = m[1].x;
                tag.points[1].y = m[1].y;
                tag.points[2].x = m[2].x;
                tag.points[2].y = m[2].y;
                tag.points[3].x = m[3].x;
                tag.points[3].y = m[3].y;

                tag_list_msg.data.push_back(tag);
            }
        }
    }    


    tags_pub->publish(tag_list_msg);

    sensor_msgs::msg::Image img_msg;
    cv_ptr->toImageMsg(img_msg);
    img_pub->publish(img_msg);

    //auto end = std::chrono::steady_clock::now();
    //auto t = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    //RCLCPP_INFO(this->get_logger(), "Detected %3d %d", tag_list_msg.data.size(), t);
}


int main(int argc, char ** argv)
{

    rclcpp::init(argc, argv);
    auto n = std::make_shared<Dots_process_cam>("dots_process_cam");
    RCLCPP_INFO(n->get_logger(), "Starting node");
    rclcpp::spin(n);
    rclcpp::shutdown();    
}