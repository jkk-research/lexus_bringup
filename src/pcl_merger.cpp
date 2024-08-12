#include "lexus_bringup/pcl_merger.hpp"

using namespace std::chrono_literals;


namespace merger
{
    class OusterPCLMerger : public rclcpp::Node
    {
    public:
        OusterPCLMerger(const rclcpp::NodeOptions& options)
        : rclcpp::Node("ouster_pcl_merger", options),
          // QoS
          qos(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile)
        {
            // config parameters
            this->declare_parameter(
                "topics", std::vector<std::string>()
            );
            this->declare_parameter(
                "frames", std::vector<std::string>()
            );
            this->declare_parameter(
                "children_frames", std::vector<std::string>()
            );
            this->declare_parameter(
                "target_frame", ""
            );
            this->declare_parameter(
                "pub_topic_name", ""
            );

            topic_list = this->get_parameter("topics").as_string_array();
            frame_list = this->get_parameter("frames").as_string_array();
            children_frame_list = this->get_parameter("children_frames").as_string_array();
            target_frame = this->get_parameter("target_frame").as_string();
            pub_topic_name = this->get_parameter("pub_topic_name").as_string();

            // debug
            logParameterList("Topics", topic_list);

            // init other variables and objects
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
            // tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

            initPointCloudPtrList();

            // timer = this->create_wall_timer(50ms, std::bind(&OusterPCLMerger::broadcastTimerCallback, this));
            timer_pub = this->create_wall_timer(50ms, std::bind(&OusterPCLMerger::mergedPCLPubCallback, this));


            // subscriber and publisher declarations
            initSubscribers();

            this->merged_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                pub_topic_name, 1
            );
        }
    private:
        std::vector<std::string> topic_list;
        std::vector<std::string> frame_list;
        std::vector<std::string> children_frame_list;
        std::string target_frame;
        std::string pub_topic_name;

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        rclcpp::QoS qos;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;
        // std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

        geometry_msgs::msg::TransformStamped t;
        geometry_msgs::msg::TransformStamped transform;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pcl_ptr_list;
        pcl::PointCloud<pcl::PointXYZI> merged_pcl;
        sensor_msgs::msg::PointCloud2 merged_pcl_msg;

        // rclcpp::TimerBase::SharedPtr timer;
        rclcpp::TimerBase::SharedPtr timer_pub;

        std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_list;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pcl_pub;

        // callback and other func signatures (and def. temporarily)
        void callbackCommon(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received PointCloud2 message from frame: %s", msg->header.frame_id.c_str());
            RCLCPP_DEBUG(this->get_logger(), "Message width: %d, height: %d, data size: %lu", msg->width, msg->height, msg->data.size());

            if (msg->data.empty()) {
                RCLCPP_WARN(this->get_logger(), "Received empty point cloud message.");
                return;
            }

            for (unsigned int i = 0; i < pcl_ptr_list.size(); i++) {
                pcl::fromROSMsg(*msg, *pcl_ptr_list[i]);

                if (pcl_ptr_list[i]->empty()) {
                    RCLCPP_WARN(this->get_logger(), "Converted PCL is empty for index: %d", i);
                    continue;
                }

                try 
                {
                    // TODO: skip transform for base frame
                    transform = tf_buffer->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
                    RCLCPP_DEBUG(this->get_logger(), "Transform found from %s to %s", msg->header.frame_id.c_str(), target_frame.c_str());
                    pcl_ros::transformPointCloud(*pcl_ptr_list[i], *pcl_ptr_list[i], transform);
                    pcl_ptr_list[i]->header.frame_id = target_frame;
                } 
                catch (const tf2::TransformException& ex) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Transform Exception: %s", ex.what());
                    return;
                }
            }
        }

        void initPointCloudPtrList() {
            for (unsigned int i = 0; i < topic_list.size(); i++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl_ptr_list.push_back(cloud);
            }
        }


        void initSubscribers() {
            auto bound_callback_func = std::bind(
                &OusterPCLMerger::callbackCommon, this, std::placeholders::_1
            );

            for (unsigned int i = 0; i < topic_list.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_list[i].c_str());
                sub_list.push_back(
                    this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        topic_list[i],
                        qos,
                        bound_callback_func
                    )
                );
            }
        }

        // void broadcastTimerCallback() {
        //     t.header.stamp = this->get_clock()->now();
        //     t.header.frame_id = "world";
        //     t.child_frame_id = "lexus3/os_left_b";
        //     t.transform.translation.x = 0.0;
        //     t.transform.translation.y = 0.0;
        //     t.transform.translation.z = 0.0;
        //     t.transform.rotation.x = 0.0;
        //     t.transform.rotation.y = 0.0;
        //     t.transform.rotation.z = 0.0;
        //     t.transform.rotation.w = 1.0;
        //     tf_broadcaster->sendTransform(t);

        //     t.header.stamp = this->get_clock()->now();
        //     t.header.frame_id = "world";
        //     t.child_frame_id = "lexus3/os_right_b";
        //     t.transform.translation.x = 0.1;
        //     t.transform.translation.y = 0.1;
        //     t.transform.translation.z = 0.1;
        //     t.transform.rotation.x = 0.0;
        //     t.transform.rotation.y = 0.0;
        //     t.transform.rotation.z = 0.0;
        //     t.transform.rotation.w = 1.0;

        //     tf_broadcaster->sendTransform(t);          
        // };

        void mergedPCLPubCallback() {
            int count = 0;

            for(unsigned int i = 0; i < pcl_ptr_list.size(); i++){
                // TODO: take base point cloud as starting point
                merged_pcl += *pcl_ptr_list[i];
                pcl_ptr_list[i]->clear();
            }

            // Check if the merged point cloud is empty
            // if (merged_pcl.empty()) {
            //     RCLCPP_WARN(this->get_logger(), "Merged point cloud is empty, nothing to publish.");
            //     return;
            // }

            merged_pcl.header.frame_id = target_frame; // "world"; // TODO: change to target_frame
            pcl::toROSMsg(merged_pcl, merged_pcl_msg);

            std::weak_ptr<
                std::remove_pointer<decltype(merged_pcl_pub.get())>::type
            > captured_pub = merged_pcl_pub;
            auto pub_ptr = captured_pub.lock();
            if (!pub_ptr) {
                return;
            }

            sensor_msgs::msg::PointCloud2::UniquePtr msg_ptr(
                new sensor_msgs::msg::PointCloud2(merged_pcl_msg)
            );
            
            // msg_ptr->data = {count++};
            msg_ptr->data = std::vector<uint8_t>(1, static_cast<uint8_t>(count++));

            // RCLCPP_DEBUG(
            //     this->get_logger(),
            //     "Published message with value: %d and address: 0x%" PRIXPTR,
            //     msg_ptr->data[0], reinterpret_cast<std::uintptr_t>(msg_ptr.get())
            // );

            pub_ptr->publish(std::move(msg_ptr));
        };

        /* Debug function to log parameters */
        void logParameterList(const std::string& parameter_name, const std::vector<std::string>& list)
        {
            std::string list_str = "[";
            for (const auto& item : list) {
                list_str += item + ", ";
            }
            if (!list.empty()) {
                // remove the trailing comma and space
                list_str.pop_back();
                list_str.pop_back();
            }
            list_str += "]";
            RCLCPP_INFO(this->get_logger(), "%s: %s", parameter_name.c_str(), list_str.c_str());
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(merger::OusterPCLMerger)