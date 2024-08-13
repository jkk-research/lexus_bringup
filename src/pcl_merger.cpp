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
                "child_frames", std::vector<std::string>()
            );
            this->declare_parameter(
                "target_frame", ""
            );
            this->declare_parameter(
                "pub_topic_name", ""
            );

            topic_list = this->get_parameter("topics").as_string_array();
            frame_list = this->get_parameter("frames").as_string_array();
            child_frame_list = this->get_parameter("child_frames").as_string_array();
            target_frame = this->get_parameter("target_frame").as_string();
            pub_topic_name = this->get_parameter("pub_topic_name").as_string();

            // checking parameters (topics) - only active with DEBUG verbosity
            RCLCPP_DEBUG(this->get_logger(), "Logging parameters:");
            RCLCPP_DEBUG(this->get_logger(), "%s", logParameterList("Topics", topic_list).c_str());

            // init other variables and objects
            tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

            initPointCloudPtrList();

            // subscriber and publisher declarations
            timer_pub = this->create_wall_timer(50ms, std::bind(&OusterPCLMerger::mergedPCLPubCallback, this));
            initSubscribers();
            this->merged_pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic_name, 1);
        }
    private:
        std::vector<std::string> topic_list;
        std::vector<std::string> frame_list;
        std::vector<std::string> child_frame_list;
        std::string target_frame;
        std::string pub_topic_name;

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        rclcpp::QoS qos;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer;

        geometry_msgs::msg::TransformStamped transform;
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pcl_ptr_list;
        pcl::PointCloud<pcl::PointXYZI> merged_pcl;
        sensor_msgs::msg::PointCloud2 merged_pcl_msg;

        rclcpp::TimerBase::SharedPtr timer_pub;
        std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_list;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_pcl_pub;

        // callback and other func signatures (and def. temporarily)
        void callbackCommon(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg, int index) {
            pcl::fromROSMsg(*msg, *pcl_ptr_list[index]);
            try
            {
                transform = tf_buffer->lookupTransform(target_frame, msg->header.frame_id, tf2::TimePointZero);
                pcl_ros::transformPointCloud(*pcl_ptr_list[index], *pcl_ptr_list[index], transform);
                pcl_ptr_list[index]->header.frame_id = target_frame;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Transform Exception: %s", ex.what());
                return;
            }
        }

        void initPointCloudPtrList() {
            for (unsigned int i = 0; i < topic_list.size(); i++) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
                pcl_ptr_list.push_back(cloud);
            }
        }

        void initSubscribers()
        {
            for (unsigned int i = 0; i < topic_list.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "Subscribing to topic: %s", topic_list[i].c_str());
                sub_list.push_back(
                    this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        topic_list[i],
                        qos,
                        [this, i](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
                            this->callbackCommon(msg, i);
                        }
                    )
                );
            }
        }
        
        void mergedPCLPubCallback() {
            for(unsigned int i = 0; i < pcl_ptr_list.size(); i++) {
                merged_pcl += *pcl_ptr_list[i];  // TODO: take base point cloud as starting point
                pcl_ptr_list[i]->clear();
            }

            RCLCPP_INFO(
                this->get_logger(),
                "After merging - Merged PointCloud: width = %d, height = %d, size = %lu",
                merged_pcl.width,
                merged_pcl.height,
                merged_pcl.points.size()
            );

            merged_pcl.header.frame_id = target_frame;
            pcl::toROSMsg(merged_pcl, merged_pcl_msg);

            // using a weak ptr first to safely gain acess of the publisher
            std::weak_ptr<
                std::remove_pointer<
                    decltype(merged_pcl_pub.get())
                >::type
            > pub_weak_ptr = merged_pcl_pub;

            // obtain a shared pointer to the publisher if it's still valid
            auto pub_ptr = pub_weak_ptr.lock();
            if (!pub_ptr) {
                RCLCPP_WARN(this->get_logger(), "Publisher no longer exists.");
                return;
            }

            sensor_msgs::msg::PointCloud2::UniquePtr msg_ptr(
                new sensor_msgs::msg::PointCloud2(merged_pcl_msg)
            );

            RCLCPP_DEBUG(
                this->get_logger(),
                "Published message with address: 0x%" PRIXPTR,
                reinterpret_cast<std::uintptr_t>(msg_ptr.get())
            );

            pub_ptr->publish(std::move(msg_ptr));
            // clear the pcl object
            merged_pcl.clear();
        };

        /* Debug function to print out incoming YAML parameters (set at least INFO verbosity in launch file). */
        std::string logParameterList(const std::string& parameter_name, const std::vector<std::string>& list)
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
            return parameter_name + ": " + list_str;
        }
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(merger::OusterPCLMerger)