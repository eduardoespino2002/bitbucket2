/*
 * Copyright (C) 2021 Simon Jones
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <algorithm>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;


bool string_in(const std::string &needle, const std::vector<std::string> &haystack)
{
    return haystack.cend() != std::find(haystack.cbegin(), haystack.cend(), needle);
}


class Tf_relay : public rclcpp::Node
{
public:
    Tf_relay(std::string n_) : Node(n_)
    {
        prefix          = this->declare_parameter("prefix", "");
        frame_list      = this->declare_parameter<std::vector<std::string> >("exclude_frames", {});

        if (frame_list.empty())
        {
            RCLCPP_WARN(this->get_logger(), "'frames' param is empty, rewriting all frames");
        }

        tf_pub          = this->create_publisher<tf2_msgs::msg::TFMessage>("out/tf", 100);
        tf_static_pub   = this->create_publisher<tf2_msgs::msg::TFMessage>("out/tf_static", 100);
        tf_sub          = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "in/tf", 100, std::bind(&Tf_relay::tf_callback, this, _1));
        tf_static_sub   = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "in/tf_static", 100, std::bind(&Tf_relay::tf_static_callback, this, _1));
    }

private:
    std::string                 prefix;
    std::vector<std::string>    frame_list;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr      tf_pub;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr      tf_static_pub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr   tf_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr   tf_static_sub;
    
    void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        tf2_msgs::msg::TFMessage output_msg;
        for (const auto &transform : msg->transforms)
        {
            // Append prefix to frame names
            geometry_msgs::msg::TransformStamped output_tf = transform;
            if (frame_list.empty() || !string_in(transform.header.frame_id, frame_list))
            {
                output_tf.header.frame_id = prefix + transform.header.frame_id;
            }
            else
            {
                output_tf.header.frame_id = transform.header.frame_id;
            }
            if (frame_list.empty() || !string_in(transform.child_frame_id, frame_list))
            {
                output_tf.child_frame_id = prefix + transform.child_frame_id;
            }
            else
            {
                output_tf.child_frame_id = transform.child_frame_id;
            }
            output_msg.transforms.push_back(output_tf);
            //RCLCPP_INFO(this->get_logger(), "%s->%s", transform.header.frame_id.c_str(), output_tf.header.frame_id.c_str());
        }
        tf_pub->publish(output_msg);
    }

    void tf_static_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
    {
        static tf2_msgs::msg::TFMessage static_tf_msg;
        for (const auto &transform : msg->transforms)
        {
            // Append prefix to frame names
            geometry_msgs::msg::TransformStamped output_tf = transform;
            if (frame_list.empty() || !string_in(transform.header.frame_id, frame_list))
            {
                output_tf.header.frame_id = prefix + transform.header.frame_id;
            }
            else
            {
                output_tf.header.frame_id = transform.header.frame_id;
            }
            if (frame_list.empty() || !string_in(transform.child_frame_id, frame_list))
            {
                output_tf.child_frame_id = prefix + transform.child_frame_id;
            }
            else
            {
                output_tf.child_frame_id = transform.child_frame_id;
            }

            // Did the transform already exist in the message?
            auto iter = static_tf_msg.transforms.begin();
            for (; iter != static_tf_msg.transforms.end(); ++iter)
            {
                if (iter->header.frame_id == output_tf.header.frame_id && iter->child_frame_id == output_tf.child_frame_id)
                {
                    // This transform already exists
                    break;
                }
            }
            if (iter == static_tf_msg.transforms.end())
            {
                // Insert new transform since one didn't already exist
                static_tf_msg.transforms.push_back(output_tf);
            }
            else
            {
                // Replace existing transform with same frame names
                iter = static_tf_msg.transforms.insert(iter, output_tf);
                ++iter;
                static_tf_msg.transforms.erase(iter);
            }
        }
        tf_static_pub->publish(static_tf_msg);
    }
};




int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<Tf_relay>("tf2_relay");
    rclcpp::spin(n);
    rclcpp::shutdown();

    return 0;
}