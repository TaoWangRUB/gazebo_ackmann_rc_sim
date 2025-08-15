#include <thread>
#include <memory>
#include <cmath>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/EventManager.hh>
#include <ignition/gazebo/components/Camera.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/Entity.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/Element.hh>

namespace camera_info_plugin
{

class CameraInfoPublisher : public ignition::gazebo::System,
                            public ignition::gazebo::ISystemConfigure,
                            public ignition::gazebo::ISystemPostUpdate
{
public:
    CameraInfoPublisher() = default;
    
    ~CameraInfoPublisher() override
    {
        // Shutdown ROS node gracefully
        if (node_ && rclcpp::ok())
        {
            node_.reset();
        }
        
        // Join the executor thread
        if (exec_thread_.joinable())
        {
            exec_thread_.join();
        }
    }

    void Configure(const ignition::gazebo::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   ignition::gazebo::EntityComponentManager &ecm,
                   ignition::gazebo::EventManager &) override
    {
        camera_entity_ = entity;
        
        // Initialize ROS2 if not already done
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }
        
        // Create node
        node_ = std::make_shared<rclcpp::Node>("camera_info_publisher_" + std::to_string(entity));
        
        // Get topic name from SDF or use default
        std::string topic_name = "/camera_info";
        if (sdf->HasElement("topic"))
        {
            topic_name = sdf->Get<std::string>("topic");
        }
        
        // Get frame_id from SDF or use default
        frame_id_ = "camera_link";
        if (sdf->HasElement("frame_id"))
        {
            frame_id_ = sdf->Get<std::string>("frame_id");
        }
        
        // Get publish rate from SDF or use default (30 Hz)
        publish_rate_ = 30.0;
        if (sdf->HasElement("publish_rate"))
        {
            publish_rate_ = sdf->Get<double>("publish_rate");
        }
        
        // Create publisher
        pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(topic_name, 10);
        
        // Start executor thread
        exec_thread_ = std::thread([this]() {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(node_);
            executor.spin();
        });
        
        // Get camera parameters from SDF or use defaults
        // Note: We'll get parameters from the plugin SDF configuration
        // since the Camera component doesn't expose SDF data directly
        
        // Get camera parameters from plugin SDF configuration
        if (sdf->HasElement("width"))
        {
            width_ = sdf->Get<int>("width");
        }
        if (sdf->HasElement("height"))
        {
            height_ = sdf->Get<int>("height");
        }
        if (sdf->HasElement("horizontal_fov"))
        {
            hfov_ = sdf->Get<double>("horizontal_fov");
        }
        if (sdf->HasElement("near_clip"))
        {
            near_clip_ = sdf->Get<double>("near_clip");
        }
        if (sdf->HasElement("far_clip"))
        {
            far_clip_ = sdf->Get<double>("far_clip");
        }
        
        // Get distortion parameters if available
        if (sdf->HasElement("k1"))
            k1_ = sdf->Get<double>("k1");
        if (sdf->HasElement("k2"))
            k2_ = sdf->Get<double>("k2");
        if (sdf->HasElement("k3"))
            k3_ = sdf->Get<double>("k3");
        if (sdf->HasElement("p1"))
            p1_ = sdf->Get<double>("p1");
        if (sdf->HasElement("p2"))
            p2_ = sdf->Get<double>("p2");
        
        // Check if we have a camera component (for validation)
        auto cam_comp = ecm.Component<ignition::gazebo::components::Camera>(camera_entity_);
        if (cam_comp)
        {
            RCLCPP_INFO(node_->get_logger(), 
                "Camera sensor found for entity %lu", camera_entity_);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), 
                "Camera component not found for entity %lu, using configured defaults", camera_entity_);
        }
        
        RCLCPP_INFO(node_->get_logger(), 
            "Camera configured: %dx%d, HFOV: %.4f rad, Topic: %s", 
            width_, height_, hfov_, topic_name.c_str());
        
        // Calculate camera intrinsics
        calculate_camera_info();
        
        // Initialize timing
        last_publish_time_ = std::chrono::steady_clock::now();
    }

    void PostUpdate(const ignition::gazebo::UpdateInfo &info,
                    const ignition::gazebo::EntityComponentManager &) override
    {
        // Check if we should publish based on the configured rate
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_publish_time_).count();
        double publish_period_ms = 1000.0 / publish_rate_;
        
        if (elapsed >= publish_period_ms && pub_ && node_ && rclcpp::ok())
        {
            publish_camera_info(info);
            last_publish_time_ = now;
        }
    }

private:
    void calculate_camera_info()
    {
        // Calculate focal length in pixels
        // fx = fy = (image_width / 2) / tan(hfov / 2)
        double fx = (static_cast<double>(width_) / 2.0) / std::tan(hfov_ / 2.0);
        double fy = fx; // Assuming square pixels
        
        // Principal point (optical center)
        double cx = static_cast<double>(width_) / 2.0;
        double cy = static_cast<double>(height_) / 2.0;
        
        // Fill camera intrinsics matrix K (3x3)
        camera_info_.k.fill(0.0);
        camera_info_.k[0] = fx;  // fx
        camera_info_.k[2] = cx;  // cx
        camera_info_.k[4] = fy;  // fy
        camera_info_.k[5] = cy;  // cy
        camera_info_.k[8] = 1.0; // 1
        
        // Fill rectification matrix R (3x3) - identity for monocular camera
        camera_info_.r.fill(0.0);
        camera_info_.r[0] = 1.0;
        camera_info_.r[4] = 1.0;
        camera_info_.r[8] = 1.0;
        
        // Fill projection matrix P (3x4)
        camera_info_.p.fill(0.0);
        camera_info_.p[0] = fx;   // fx
        camera_info_.p[2] = cx;   // cx
        camera_info_.p[5] = fy;   // fy
        camera_info_.p[6] = cy;   // cy
        camera_info_.p[10] = 1.0; // 1
        
        // Fill distortion coefficients
        camera_info_.d.clear();
        camera_info_.d.push_back(k1_);
        camera_info_.d.push_back(k2_);
        camera_info_.d.push_back(p1_);
        camera_info_.d.push_back(p2_);
        camera_info_.d.push_back(k3_);
        
        // Set other fields
        camera_info_.width = width_;
        camera_info_.height = height_;
        camera_info_.distortion_model = "plumb_bob";
        camera_info_.header.frame_id = frame_id_;
        
        RCLCPP_INFO(node_->get_logger(),
            "Camera intrinsics calculated - fx: %.2f, fy: %.2f, cx: %.2f, cy: %.2f",
            fx, fy, cx, cy);
    }
    
    void publish_camera_info(const ignition::gazebo::UpdateInfo &info)
    {
        // Update timestamp
        camera_info_.header.stamp.sec = static_cast<int32_t>(info.simTime.count() / 1000000000);
        camera_info_.header.stamp.nanosec = static_cast<uint32_t>(info.simTime.count() % 1000000000);
        
        // Publish the message
        pub_->publish(camera_info_);
    }

    // Entity and ROS components
    ignition::gazebo::Entity camera_entity_{ignition::gazebo::kNullEntity};
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
    std::thread exec_thread_;
    
    // Camera parameters
    int width_{640};
    int height_{480};
    double hfov_{1.047}; // 60 degrees in radians
    double near_clip_{0.1};
    double far_clip_{100.0};
    
    // Distortion parameters
    double k1_{0.0}, k2_{0.0}, k3_{0.0};
    double p1_{0.0}, p2_{0.0};
    
    // Configuration
    std::string frame_id_{"camera_link"};
    double publish_rate_{30.0};
    
    // Cached camera info message
    sensor_msgs::msg::CameraInfo camera_info_;
    
    // Timing
    std::chrono::steady_clock::time_point last_publish_time_;
};

} // namespace camera_info_plugin

// Register the plugin
IGNITION_ADD_PLUGIN(
    camera_info_plugin::CameraInfoPublisher,
    ignition::gazebo::System,
    camera_info_plugin::CameraInfoPublisher::ISystemConfigure,
    camera_info_plugin::CameraInfoPublisher::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(camera_info_plugin::CameraInfoPublisher, "camera_info_publisher")