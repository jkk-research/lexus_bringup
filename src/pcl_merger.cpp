#include "lexus_bringup/pcl_merger.hpp"


namespace merger 
{
    class OusterPCLMerger : public rclcpp::Node
    {
    public:
        OusterPCLMerger(const rclcpp::NodeOptions& options)
        : rclcpp::Node("ouster_pcl_merger", options)
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

            topic_list = this->get_parameter("topics").as_string_array();
            frame_list = this->get_parameter("frames").as_string_array();
            children_frame_list = this->get_parameter("children_frames").as_string_array();

            // DEBUG
            log_parameter_list("Topics", topic_list);

            // subscriber and publisher declarations

            // init other variables and objects
        }
    private:
        std::vector<std::string> topic_list;
        std::vector<std::string> frame_list;
        std::vector<std::string> children_frame_list;

        // callback and other func signatures (and def. temporarily)

        /* Debug function to log parameters */
        void log_parameter_list(const std::string& parameter_name, const std::vector<std::string>& list)
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