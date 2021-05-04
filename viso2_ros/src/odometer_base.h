
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#define DBG_EXPORT_TRAJECTORY
#ifdef DBG_EXPORT_TRAJECTORY
#include <fstream>
#endif


namespace viso2_ros
{

/** Different modes for setting the covariance matrix of the motion */
    typedef enum _CovarianceMode
    {
        CovModeStandard = 0, ///< Setting covariances based on default values
        CovModeInlierBased = 1, ///< Setting covariance based on the number of inliers found
        CovModeSvd = 2 ///< SVD based covariance estimation
    } CovarianceMode;

/**
 * Base class for odometers, handles tf's, odometry and pose
 * publishing. This can be used as base for any incremental pose estimating
 * sensor. Sensors that measure velocities cannot be used.
 */
    class OdometerBase
    {

    private:

        rclcpp::Node *node_;

        // publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

        // tf related
        std::string sensor_frame_id_;
        std::string odom_frame_id_;
        std::string base_link_frame_id_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        bool publish_tf_;
        bool invert_tf_;

        // the current integrated camera pose
        tf2::Transform integrated_pose_;
        // timestamp of the last update
        rclcpp::Time last_update_time_;
        // the latest motion of base to sensor <- added in order to avoid timing problems with the transform
        tf2::Stamped<tf2::Transform> base_to_sensor_;
        std::shared_ptr<tf2_ros::Buffer> buffer_;

        // indicates whether the transform from base to sensor has been set at least once
        bool base_to_sensor_set_;
        // enforces waiting for base <- sensor before publishing results
        bool wait_for_base_to_sensor_;
        // waits for correct velocities before publishing
        bool wait_for_velocities_;

        // initial pose of the base
        bool initial_base_pose_is_id_;
        bool initial_base_pose_set_;
        tf2::Stamped<tf2::Transform> initial_base_pose_;

        // covariances
        std::array<double, 36> pose_covariance_;
        std::array<double, 36> twist_covariance_;


    protected:

#ifdef DBG_EXPORT_TRAJECTORY
        std::ofstream posestream_; ///< Stream for logging trajectory to CSV file
#endif

    public:
        CovarianceMode cov_mode; ///< Mode for setting the covariances of the result
        // covariances
        int nof_inliers_min; ///< Minimum number of inliers required
        int nof_inliers_ok; ///< Intermediate number of inliers
        int nof_inliers_good; ///< Number of inliers of high quality results

        double cov_pos_min; ///< Position covariance at minimum number of inliers required
        double cov_pos_ok; ///< Position covariance at intermediate number of inliers
        double cov_pos_good; ///< Position covariance at number of inliers of high quality results

        double cov_ori_min; ///< Orientation covariance at minimum number of inliers required
        double cov_ori_ok; ///< Orientation covariance at intermediate number of inliers
        double cov_ori_good; ///< Orientation covariance at number of inliers of high quality results

        OdometerBase(rclcpp::Node *node) :
                node_(node),
                last_update_time_(0, 0)
        {

            buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_node_clock_interface()->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
            tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);
            // Read local parameters
            odom_frame_id_ = node->declare_parameter("odom_frame_id", std::string("odom"));
            base_link_frame_id_ = node->declare_parameter("base_link_frame_id", std::string("base_link"));
            sensor_frame_id_ = node->declare_parameter("sensor_frame_id", std::string("camera"));
            publish_tf_ = node->declare_parameter("publish_tf", true);
            invert_tf_ = node->declare_parameter("invert_tf", false);
            wait_for_base_to_sensor_ = node->declare_parameter("wait_for_base_to_sensor", false);
            wait_for_velocities_ = node->declare_parameter("wait_for_velocities", false);
            initial_base_pose_is_id_ = node->declare_parameter("initialize_pose_as_id", true);

            RCLCPP_INFO_STREAM(node->get_logger(), "Basic Odometer Settings:" << std::endl <<

                                                                              "  odom_frame_id      = "
                                                                              << odom_frame_id_ << std::endl <<
                                                                              "  base_link_frame_id = "
                                                                              << base_link_frame_id_ << std::endl <<
                                                                              "  publish_tf         = "
                                                                              << (publish_tf_ ? "true" : "false")
                                                                              << std::endl <<
                                                                              "  invert_tf          = "
                                                                              << (invert_tf_ ? "true" : "false"));

            // TODO
            // covariance mode parameters
            // Read parameters for setting covariance from the number of inliers
            bool cov_from_inliers = true;
            cov_from_inliers &= node_->has_parameter("nof_inliers_min");
            cov_from_inliers &= node_->has_parameter("nof_inliers_ok");
            cov_from_inliers &= node_->has_parameter("nof_inliers_good");

            cov_from_inliers &= node_->has_parameter("cov_pos_min");
            cov_from_inliers &= node_->has_parameter("cov_pos_ok");
            cov_from_inliers &= node_->has_parameter("cov_pos_good");

            cov_from_inliers &= node_->has_parameter("cov_ori_min");
            cov_from_inliers &= node_->has_parameter("cov_ori_ok");
            cov_from_inliers &= node_->has_parameter("cov_ori_good");

            cov_mode = cov_from_inliers ? CovModeInlierBased : CovModeStandard;
            // Activate setting covariance from svd
            if (node_->has_parameter("cov_from_svd"))
                if (node_->get_parameter("cov_from_svd").as_int() == 2)
                    cov_mode = CovModeSvd;

            if (cov_mode == CovModeInlierBased)
            {
                nof_inliers_min = node_->get_parameter("nof_inliers_min").as_int();
                nof_inliers_ok = node_->get_parameter("nof_inliers_ok").as_int();
                nof_inliers_good = node_->get_parameter("nof_inliers_good").as_int();

                cov_pos_min = node_->get_parameter("cov_pos_min").as_double();
                cov_pos_ok = node_->get_parameter("cov_pos_ok").as_double();
                cov_pos_good = node_->get_parameter("cov_pos_good").as_double();

                cov_ori_min = node_->get_parameter("cov_ori_min").as_double();
                cov_ori_ok = node_->get_parameter("cov_ori_ok").as_double();
                cov_ori_good = node_->get_parameter("cov_ori_good").as_double();
            }

            // advertise
            odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
            pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 2);

            // TODO
            reset_service_ = node_->create_service<std_srvs::srv::Empty>("reset_pose",
                                                                         std::bind(&OdometerBase::resetPose, this,
                                                                                   std::placeholders::_1,
                                                                                   std::placeholders::_2));

            integrated_pose_.setIdentity();
            base_to_sensor_.setIdentity();
            initial_base_pose_.setIdentity();
            base_to_sensor_set_ = false;
            initial_base_pose_set_ = false;

            pose_covariance_.fill(0.0);
            twist_covariance_.fill(0.0);
        }

        void setSensorFrameId(const std::string &frame_id)
        {
            sensor_frame_id_ = frame_id;
        }

        std::string getSensorFrameId() const
        {
            return sensor_frame_id_;
        }

        void setPoseCovariance(const std::array<double, 36> &pose_covariance)
        {
            pose_covariance_ = pose_covariance;
        }

        void setTwistCovariance(const std::array<double, 36> &twist_covariance)
        {
            twist_covariance_ = twist_covariance;
        }

        void integrateAndPublish(const tf2::Transform &delta_transform, const rclcpp::Time &timestamp)
        {
            if (sensor_frame_id_.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "[odometer] update called with unknown sensor frame id!");
                return;
            }
            if (timestamp.seconds() < last_update_time_.seconds())
            {
                RCLCPP_WARN(node_->get_logger(),
                            "[odometer] saw negative time change in incoming sensor data, resetting pose.");
                integrated_pose_.setIdentity();
                buffer_->clear();
            }

            // integrate the pose
            integrated_pose_ *= delta_transform;

            // Try to get the transform from sensor to base
            std::string error_msg;
            if (buffer_->canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, rclcpp::Duration(0),
                                      &error_msg))
            {
                geometry_msgs::msg::TransformStamped base_to_sensor_stf =
                        buffer_->lookupTransform(base_link_frame_id_, sensor_frame_id_, timestamp);
                tf2::convert(base_to_sensor_stf, base_to_sensor_);
                base_to_sensor_set_ = true;
            }
            else
            {
                if (!base_to_sensor_set_)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Transform error: %s", error_msg.c_str());
                }
            }

            // initialize the pose of base_link in odom or leave it set to id
            if (!initial_base_pose_is_id_)
            {
                // Try to initialize trajectory with current odom <- base_link or odom <- base_link_init
                if (!initial_base_pose_set_)
                {
                    std::string error_msg;

                    if (buffer_->canTransform(odom_frame_id_, base_link_frame_id_, timestamp, rclcpp::Duration(0),
                                              &error_msg))
                    {
                        geometry_msgs::msg::TransformStamped initialial_base_pose_stf =
                                buffer_->lookupTransform(odom_frame_id_, base_link_frame_id_, timestamp);
                        tf2::convert(initialial_base_pose_stf, initial_base_pose_);

                        // Set the actual integrated pose to the identity, so the result is initialzed with the trafo looked up
                        integrated_pose_.setIdentity();

                        initial_base_pose_set_ = true;
                        RCLCPP_INFO(node_->get_logger(), "Trafo %s to %s AVAILABLE -> INITIALIZED stereo odometer",
                                    base_link_frame_id_.c_str(), odom_frame_id_.c_str());
                    }
                    else
                    {
                        std::string base_link_init_frame_id = base_link_frame_id_ + "_init";

                        if (buffer_->canTransform(odom_frame_id_, base_link_init_frame_id, timestamp,
                                                  rclcpp::Duration(0), &error_msg))
                        {
                            geometry_msgs::msg::TransformStamped initial_base_pose_stf =
                                    buffer_->lookupTransform(odom_frame_id_, base_link_init_frame_id, timestamp);
                            tf2::convert(initial_base_pose_stf, initial_base_pose_);

                            // Set the actual integrated pose to the identity, so the result is initialzed with the trafo looked up
                            integrated_pose_.setIdentity();

                            initial_base_pose_set_ = true;
                            RCLCPP_INFO(node_->get_logger(), "Trafo %s to %s AVAILABLE -> INITIALIZED stereo odometer",
                                        base_link_init_frame_id.c_str(), odom_frame_id_.c_str());
                        }
                        else
                        {
                            RCLCPP_WARN(node_->get_logger(),
                                        "Trafo %s (or %s) to %s NOT available -> Cannot initialize stereo odometer",
                                        base_link_frame_id_.c_str(), base_link_init_frame_id.c_str(),
                                        odom_frame_id_.c_str());
                            return;
                        }
                    }
                }
            }

            // transform integrated pose to base frame
            tf2::Transform base_transform =
                    initial_base_pose_ * base_to_sensor_ * integrated_pose_ * base_to_sensor_.inverse();

            // Also transform the covariances
            transformCovariance(base_to_sensor_, pose_covariance_);
            transformCovariance(base_to_sensor_, twist_covariance_);

            RCLCPP_DEBUG(node_->get_logger(), "cov in body: %.3f %.3f %.3f %.3f %.3f %.3f",
                         pose_covariance_[0], pose_covariance_[7], pose_covariance_[14],
                         pose_covariance_[21], pose_covariance_[28], pose_covariance_[35]);

            nav_msgs::msg::Odometry odometry_msg;
            odometry_msg.header.stamp = timestamp;
            odometry_msg.header.frame_id = odom_frame_id_;
            odometry_msg.child_frame_id = base_link_frame_id_;
            odometry_msg.pose.pose.position.x = base_transform.getOrigin().x();
            odometry_msg.pose.pose.position.y = base_transform.getOrigin().y();
            odometry_msg.pose.pose.position.z = base_transform.getOrigin().z();
            odometry_msg.pose.pose.orientation.w = base_transform.getRotation().w();
            odometry_msg.pose.pose.orientation.x = base_transform.getRotation().x();
            odometry_msg.pose.pose.orientation.y = base_transform.getRotation().y();
            odometry_msg.pose.pose.orientation.z = base_transform.getRotation().z();

            // calculate twist (not possible for first run as no delta_t can be computed)
            tf2::Transform delta_base_transform = base_to_sensor_ * delta_transform * base_to_sensor_.inverse();
            if (last_update_time_.seconds() > rclcpp::Time(0, 0).seconds())
            {
                double delta_t = (timestamp - last_update_time_).seconds();
                if (delta_t != 0.0)
                {
                    odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
                    odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
                    odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
                    tf2::Quaternion delta_rot = delta_base_transform.getRotation();
                    tf2Scalar angle = delta_rot.getAngle();
                    tf2::Vector3 axis = delta_rot.getAxis();
                    tf2::Vector3 angular_twist = axis * angle / delta_t;
                    odometry_msg.twist.twist.angular.x = angular_twist.x();
                    odometry_msg.twist.twist.angular.y = angular_twist.y();
                    odometry_msg.twist.twist.angular.z = angular_twist.z();
                }
            }

            // Check if base <- sensor and or velocities are mandatory for publishing and only publish if available
            bool publish_result =
                    (!wait_for_base_to_sensor_ || base_to_sensor_set_) &&
                    (!wait_for_velocities_ || last_update_time_ != rclcpp::Time(0, 0));

            // TODO:
            for (int i = 0; i < 36; ++i)
            {
                odometry_msg.pose.covariance[i] = pose_covariance_[i];
                odometry_msg.twist.covariance[i] = twist_covariance_[i];
            }

            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = odometry_msg.header.stamp;
            pose_msg.header.frame_id = odometry_msg.header.frame_id;
            pose_msg.pose = odometry_msg.pose.pose;

            if (publish_result)
            {
                odom_pub_->publish(odometry_msg);
                pose_pub_->publish(pose_msg);
            }

            if (publish_tf_ && publish_result)
            {
                if (invert_tf_)
                {
                    geometry_msgs::msg::Transform tf;
                    tf2::convert(base_transform.inverse(), tf);
                    geometry_msgs::msg::TransformStamped tfs;
                    tfs.transform = tf;
                    tfs.header.stamp = timestamp;
                    tfs.header.frame_id = base_link_frame_id_;
                    tfs.child_frame_id = odom_frame_id_;
                    tf_broadcaster_->sendTransform(tfs);
                }
                else
                {
                    geometry_msgs::msg::Transform tf;
                    tf2::convert(base_transform, tf);
                    geometry_msgs::msg::TransformStamped tfs;
                    tfs.transform = tf;
                    tfs.header.stamp = timestamp;
                    tfs.header.frame_id = odom_frame_id_;
                    tfs.child_frame_id = base_link_frame_id_;
                    tf_broadcaster_->sendTransform(tfs);
                }
            }

            last_update_time_ = timestamp;
        }


        bool resetPose(std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
        {
            integrated_pose_.setIdentity();
            return true;
        }

        /**
         * \brief Transforms the covariance matrix of either the pose or the twist.
         * \param[in] tf Linear transformation that should be applied
         * \param[in,out] cov Covariance matrix that is transformed in place
         */
        void transformCovariance(
                const tf2::Transform &tf,
                std::array<double, 36> &cov)
        {
            tf2::Matrix3x3 covT(cov[0], cov[1], cov[2],
                                cov[6], cov[7], cov[8],
                                cov[12], cov[13], cov[14]);

            tf2::Matrix3x3 covR(cov[21], cov[22], cov[23],
                                cov[27], cov[28], cov[29],
                                cov[33], cov[34], cov[35]);

            covT = tf.getBasis() * covT * tf.getBasis().transpose();
            covR = tf.getBasis() * covR * tf.getBasis().transpose();

            for (int r = 0; r < 3; ++r)
            {
                cov[r * 6] = covT.getRow(r).x();
                cov[r * 6 + 1] = covT.getRow(r).y();
                cov[r * 6 + 2] = covT.getRow(r).z();

                cov[(r + 3) * 6 + 3] = covR.getRow(r).x();
                cov[(r + 3) * 6 + 4] = covR.getRow(r).y();
                cov[(r + 3) * 6 + 5] = covR.getRow(r).z();
            }
        }
    };

} // end of namespace

#endif

