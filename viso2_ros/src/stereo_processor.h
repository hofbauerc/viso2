#ifndef STEREO_PROCESSOR_H_
#define STEREO_PROCESSOR_H_

#include <ros/ros.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.hpp>

namespace viso2_ros
{

    using namespace std::chrono_literals;

/**
 * This is an abstract base class for stereo image processing nodes.
 * It handles synchronization of input topics (approximate or exact)
 * and checks for sync errors.
 * To use this class, subclass it and implement the imageCallback() method.
 */
    class StereoProcessor : public rclcpp::Node
    {

    private:

        // subscriber
        std::shared_ptr<image_transport::SubscriberFilter> left_sub_, right_sub_;
        std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>> left_info_sub_, right_info_sub_;
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> ExactPolicy;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo, sensor_msgs::msg::CameraInfo> ApproximatePolicy;
        typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
        boost::shared_ptr<ExactSync> exact_sync_;
        boost::shared_ptr<ApproximateSync> approximate_sync_;
        int queue_size_;

        // for sync checking
        rclcpp::TimerBase::SharedPtr check_synced_timer_;
        int left_received_, right_received_, left_info_received_, right_info_received_, all_received_;

        // for sync checking
        static void increment(int *value)
        {
            ++(*value);
        }

        void dataCb(const sensor_msgs::msg::Image::ConstSharedPtr &l_image_msg,
                    const sensor_msgs::msg::Image::ConstSharedPtr &r_image_msg,
                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &l_info_msg,
                    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &r_info_msg)
        {

            // For sync error checking
            ++all_received_;

            // call implementation
            imageCallback(l_image_msg, r_image_msg, l_info_msg, r_info_msg);
        }

        void checkInputsSynchronized()
        {
            int threshold = 3 * all_received_;
            if (left_received_ >= threshold || right_received_ >= threshold ||
                left_info_received_ >= threshold || right_info_received_ >= threshold)
            {
                RCLCPP_WARN(
                        this->get_logger(),
                        "[stereo_processor] Low number of synchronized left/right/left_info/right_info tuples received.\n"
                        "Left images received:       %d (topic '%s')\n"
                        "Right images received:      %d (topic '%s')\n"
                        "Left camera info received:  %d (topic '%s')\n"
                        "Right camera info received: %d (topic '%s')\n"
                        "Synchronized tuples: %d\n"
                        "Possible issues:\n"
                        "\t* stereo_image_proc is not running.\n"
                        "\t  Does `rosnode info %s` show any connections?\n"
                        "\t* The cameras are not synchronized.\n"
                        "\t  Try restarting the node with parameter _approximate_sync:=True\n"
                        "\t* The network is too slow. One or more images are dropped from each tuple.\n"
                        "\t  Try restarting the node, increasing parameter 'queue_size' (currently %d)",
                        left_received_, left_sub_->getTopic().c_str(),
                        right_received_, right_sub_->getTopic().c_str(),
                        left_info_received_, left_info_sub_->getTopic().c_str(),
                        right_info_received_, right_info_sub_->getTopic().c_str(),
                        all_received_, this->get_name(), queue_size_);
            }
        }


    protected:

        /**
         * Constructor, subscribes to input topics using image transport and registers
         * callbacks.
         * \param transport The image transport to use
         */
        StereoProcessor(const std::string &node_name, rclcpp::NodeOptions &options) :
                rclcpp::Node(node_name, options),
                left_received_(0), right_received_(0), left_info_received_(0), right_info_received_(0), all_received_(0)
        {
            image_transport::TransportHints hints(this, "raw");
            left_sub_ = std::make_shared<image_transport::SubscriberFilter>(
                    this, "left/image_rect", hints.getTransport());
            right_sub_ = std::make_shared<image_transport::SubscriberFilter>(
                    this, "right/image_rect", hints.getTransport());
            left_info_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
                    this, "left/image_rect/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());
            right_info_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::CameraInfo>>(
                    this, "right/image_rect/camera_info", rclcpp::SensorDataQoS().get_rmw_qos_profile());

            // Complain every 15s if the topics appear unsynchronized
            left_sub_->registerCallback(boost::bind(StereoProcessor::increment, &left_received_));
            right_sub_->registerCallback(boost::bind(StereoProcessor::increment, &right_received_));
            left_info_sub_->registerCallback(boost::bind(StereoProcessor::increment, &left_info_received_));
            right_info_sub_->registerCallback(boost::bind(StereoProcessor::increment, &right_info_received_));
            check_synced_timer_ =
                    this->create_wall_timer(5s, std::bind(&StereoProcessor::checkInputsSynchronized, this));

            // Synchronize input topics. Optionally do approximate synchronization.
            queue_size_ = this->declare_parameter("queue_size", 5);
            bool approx = this->declare_parameter("approximate_sync", true);
            if (approx)
            {
                approximate_sync_.reset(new ApproximateSync(ApproximatePolicy(queue_size_),
                                                            *left_sub_, *right_sub_, *left_info_sub_,
                                                            *right_info_sub_));
                approximate_sync_->registerCallback(&StereoProcessor::dataCb, this);
            }
            else
            {
                exact_sync_.reset(new ExactSync(ExactPolicy(queue_size_),
                                                *left_sub_, *right_sub_, *left_info_sub_, *right_info_sub_));
                exact_sync_->registerCallback(&StereoProcessor::dataCb, this);
            }
        }

        /**
         * Implement this method in sub-classes
         */
        virtual void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &l_image_msg,
                                   const sensor_msgs::msg::Image::ConstSharedPtr &r_image_msg,
                                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr &l_info_msg,
                                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr &r_info_msg) = 0;

    };

} // end of namespace

#endif

