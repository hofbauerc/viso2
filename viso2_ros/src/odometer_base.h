
#ifndef ODOMETER_BASE_H_
#define ODOMETER_BASE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

  // publisher
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;

  ros::ServiceServer reset_service_;

  // tf related
  std::string sensor_frame_id_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  bool publish_tf_;
  bool invert_tf_;

  // the current integrated camera pose
  tf::Transform integrated_pose_;
  // timestamp of the last update
  ros::Time last_update_time_;
  // the latest motion of base to sensor <- added in order to avoid timing problems with the transform
  tf::StampedTransform base_to_sensor_;
  // indicates whether the transform from base to sensor has been set at least once
  bool base_to_sensor_set_;
  // enforces waiting for base <- sensor before publishing results
  bool wait_for_base_to_sensor_;
  // waits for correct velocities before publishing
  bool wait_for_velocities_;

  // initial pose of the base
  bool initial_base_pose_is_id_;
  bool initial_base_pose_set_;
  tf::Transform initial_base_pose_;

  // covariances
  boost::array<double, 36> pose_covariance_;
  boost::array<double, 36> twist_covariance_;


protected:

#ifdef DBG_EXPORT_TRAJECTORY
  std::ofstream posestream_; ///< Stream for logging trajectory to CSV file
#endif

  // covariances
  CovarianceMode cov_mode_; ///< Mode for setting the covariances of the result
  int nof_inliers_min_; ///< Minimum number of inliers required
  int nof_inliers_ok_; ///< Intermediate number of inliers
  int nof_inliers_good_; ///< Number of inliers of high quality results

  double cov_pos_min_; ///< Position covariance at minimum number of inliers required
  double cov_pos_ok_; ///< Position covariance at intermediate number of inliers
  double cov_pos_good_; ///< Position covariance at number of inliers of high quality results

  double cov_ori_min_; ///< Orientation covariance at minimum number of inliers required
  double cov_ori_ok_; ///< Orientation covariance at intermediate number of inliers
  double cov_ori_good_; ///< Orientation covariance at number of inliers of high quality results


public:

  OdometerBase()
  :
	  tf_listener_(ros::Duration(5.0))
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");

    local_nh.param("odom_frame_id", odom_frame_id_, std::string("/odom"));
    local_nh.param("base_link_frame_id", base_link_frame_id_, std::string("/base_link"));
    local_nh.param("sensor_frame_id", sensor_frame_id_, std::string("/camera"));
    local_nh.param("publish_tf", publish_tf_, true);
    local_nh.param("invert_tf", invert_tf_, false);
    local_nh.param("wait_for_base_to_sensor", wait_for_base_to_sensor_, false);
    local_nh.param("wait_for_velocities", wait_for_velocities_, false);
    local_nh.param("initialize_pose_as_id", initial_base_pose_is_id_, true);

    ROS_INFO_STREAM("Basic Odometer Settings:" << std::endl <<
                    "  odom_frame_id      = " << odom_frame_id_ << std::endl <<
                    "  base_link_frame_id = " << base_link_frame_id_ << std::endl <<
                    "  publish_tf         = " << (publish_tf_?"true":"false") << std::endl <<
                    "  invert_tf          = " << (invert_tf_?"true":"false"));

    // covariance mode parameters
    // Read parameters for setting covariance from the number of inliers
    bool cov_from_inliers = true;
    cov_from_inliers &= local_nh.getParam("nof_inliers_min", nof_inliers_min_);
    cov_from_inliers &= local_nh.getParam("nof_inliers_ok", nof_inliers_ok_);
    cov_from_inliers &= local_nh.getParam("nof_inliers_good", nof_inliers_good_);

    cov_from_inliers &= local_nh.getParam("cov_pos_min",  cov_pos_min_);
    cov_from_inliers &= local_nh.getParam("cov_pos_ok",  cov_pos_ok_);
    cov_from_inliers &= local_nh.getParam("cov_pos_good",  cov_pos_good_);

    cov_from_inliers &= local_nh.getParam("cov_ori_min",  cov_ori_min_);
    cov_from_inliers &= local_nh.getParam("cov_ori_ok",  cov_ori_ok_);
    cov_from_inliers &= local_nh.getParam("cov_ori_good",  cov_ori_good_);

    cov_mode_ = cov_from_inliers ? CovModeInlierBased : CovModeStandard;
    // Activate setting covariance from svd
    int tmp_cov_mode = 0;
    if (local_nh.getParam("cov_from_svd", tmp_cov_mode))
      if (tmp_cov_mode == 2)
        cov_mode_ = CovModeSvd;

    // advertise
    odom_pub_ = local_nh.advertise<nav_msgs::Odometry>("odometry", 2);
    pose_pub_ = local_nh.advertise<geometry_msgs::PoseStamped>("pose", 2);

    reset_service_ = local_nh.advertiseService("reset_pose", &OdometerBase::resetPose, this);

    integrated_pose_.setIdentity();
    base_to_sensor_.setIdentity();
    initial_base_pose_.setIdentity();
    base_to_sensor_set_ = false;
    initial_base_pose_set_ = false;

    pose_covariance_.assign(0.0);
    twist_covariance_.assign(0.0);
  }

protected:

  void setSensorFrameId(const std::string& frame_id)
  {
    sensor_frame_id_ = frame_id;
  }

  std::string getSensorFrameId() const
  {
    return sensor_frame_id_;
  }

  void setPoseCovariance(const boost::array<double, 36>& pose_covariance)
  {
    pose_covariance_ = pose_covariance;
  }

  void setTwistCovariance(const boost::array<double, 36>& twist_covariance)
  {
    twist_covariance_ = twist_covariance;
  }

  void integrateAndPublish(const tf::Transform& delta_transform, const ros::Time& timestamp)
  {
    if (sensor_frame_id_.empty())
    {
      ROS_ERROR("[odometer] update called with unknown sensor frame id!");
      return;
    }
    if (timestamp < last_update_time_)
    {
      ROS_WARN("[odometer] saw negative time change in incoming sensor data, resetting pose.");
      integrated_pose_.setIdentity();
      tf_listener_.clear();
    }

    // integrate the pose
    integrated_pose_ *= delta_transform;

    // Try to get the transform from sensor to base
    std::string error_msg;
    if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
    {
      tf_listener_.lookupTransform(
          base_link_frame_id_,
          sensor_frame_id_,
          timestamp, base_to_sensor_);

      base_to_sensor_set_ = true;
    }
    else
    {
      if (!base_to_sensor_set_)
      {
        ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                                "last one will be used!",
                          base_link_frame_id_.c_str(),
                          sensor_frame_id_.c_str());
        ROS_DEBUG("Transform error: %s", error_msg.c_str());
      }
    }

    // initialize the pose of base_link in odom or leave it set to id
    if (!initial_base_pose_is_id_)
    {
      // Try to initialize trajectory with current odom <- base_link or odom <- base_link_init
      if (!initial_base_pose_set_)
      {
        std::string error_msg;

        if (tf_listener_.canTransform(odom_frame_id_, base_link_frame_id_, timestamp, &error_msg))
        {
          tf::StampedTransform initialial_base_pose_stf;
          tf_listener_.lookupTransform(odom_frame_id_, base_link_frame_id_, timestamp, initialial_base_pose_stf);
          initial_base_pose_ = tf::Transform(initialial_base_pose_stf.getRotation(), initialial_base_pose_stf.getOrigin());

          // Set the actual integrated pose to the identity, so the result is initialzed with the trafo looked up
          integrated_pose_.setIdentity();

          initial_base_pose_set_ = true;
          ROS_INFO("Trafo %s to %s AVAILABLE -> INITIALIZED stereo odometer", base_link_frame_id_.c_str(), odom_frame_id_.c_str());
        }
        else
        {
          std::string base_link_init_frame_id = base_link_frame_id_ + "_init";

          if (tf_listener_.canTransform(odom_frame_id_, base_link_init_frame_id, timestamp, &error_msg))
          {
            tf::StampedTransform initialial_base_pose_stf;
            tf_listener_.lookupTransform(odom_frame_id_, base_link_init_frame_id, timestamp, initialial_base_pose_stf);
            initial_base_pose_ = tf::Transform(initialial_base_pose_stf.getRotation(), initialial_base_pose_stf.getOrigin());

            // Set the actual integrated pose to the identity, so the result is initialzed with the trafo looked up
            integrated_pose_.setIdentity();

            initial_base_pose_set_ = true;
            ROS_INFO("Trafo %s to %s AVAILABLE -> INITIALIZED stereo odometer", base_link_init_frame_id.c_str(), odom_frame_id_.c_str());
          }
          else
          {
            ROS_WARN("Trafo %s (or %s) to %s NOT available -> Cannot initialize stereo odometer", base_link_frame_id_.c_str(), base_link_init_frame_id.c_str(), odom_frame_id_.c_str());
            return;
          }
        }
      }
    }

    // transform integrated pose to base frame
    tf::Transform base_transform = initial_base_pose_ * base_to_sensor_ * integrated_pose_ * base_to_sensor_.inverse();

    // Also transform the covariances
    transformCovariance(base_to_sensor_, pose_covariance_);
    transformCovariance(base_to_sensor_, twist_covariance_);

    ROS_DEBUG("cov in body: %.3f %.3f %.3f %.3f %.3f %.3f",
              pose_covariance_[ 0], pose_covariance_[ 7], pose_covariance_[14],
              pose_covariance_[21], pose_covariance_[28], pose_covariance_[35]);

    nav_msgs::Odometry odometry_msg;
    odometry_msg.header.stamp = timestamp;
    odometry_msg.header.frame_id = odom_frame_id_;
    odometry_msg.child_frame_id = base_link_frame_id_;
    tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);

    // calculate twist (not possible for first run as no delta_t can be computed)
    tf::Transform delta_base_transform = base_to_sensor_ * delta_transform * base_to_sensor_.inverse();
    if (!last_update_time_.isZero())
    {
      double delta_t = (timestamp - last_update_time_).toSec();
      if (delta_t)
      {
        odometry_msg.twist.twist.linear.x = delta_base_transform.getOrigin().getX() / delta_t;
        odometry_msg.twist.twist.linear.y = delta_base_transform.getOrigin().getY() / delta_t;
        odometry_msg.twist.twist.linear.z = delta_base_transform.getOrigin().getZ() / delta_t;
        tf::Quaternion delta_rot = delta_base_transform.getRotation();
        tfScalar angle = delta_rot.getAngle();
        tf::Vector3 axis = delta_rot.getAxis();
        tf::Vector3 angular_twist = axis * angle / delta_t;
        odometry_msg.twist.twist.angular.x = angular_twist.x();
        odometry_msg.twist.twist.angular.y = angular_twist.y();
        odometry_msg.twist.twist.angular.z = angular_twist.z();
      }
    }

    // Check if base <- sensor and or velocities are mandatory for publishing and only publish if available
    bool publish_result =
            (!wait_for_base_to_sensor_ || base_to_sensor_set_) &&
            (!wait_for_velocities_ || !last_update_time_.isZero());

    odometry_msg.pose.covariance = pose_covariance_;
    odometry_msg.twist.covariance = twist_covariance_;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = odometry_msg.header.stamp;
    pose_msg.header.frame_id = odometry_msg.header.frame_id;
    pose_msg.pose = odometry_msg.pose.pose;

    if (publish_result)
    {
        odom_pub_.publish(odometry_msg);
        pose_pub_.publish(pose_msg);
    }

#ifdef DBG_EXPORT_TRAJECTORY
    // Base pose
    if (posestream_.is_open())
    {
    	tf::Quaternion integrated_campose_q(integrated_pose_.getRotation());

    	posestream_
    		<< odometry_msg.header.stamp.sec << ", "
    		<< odometry_msg.header.stamp.nsec << ", "
    		<< odometry_msg.pose.pose.position.x << ", "
    		<< odometry_msg.pose.pose.position.y << ", "
    		<< odometry_msg.pose.pose.position.z << ", "
    		<< odometry_msg.pose.pose.orientation.x << ", "
    		<< odometry_msg.pose.pose.orientation.y << ", "
    		<< odometry_msg.pose.pose.orientation.z << ", "
    		<< odometry_msg.pose.pose.orientation.w << ", "
    		<< odometry_msg.twist.twist.linear.x << ", "
    		<< odometry_msg.twist.twist.linear.y << ", "
    		<< odometry_msg.twist.twist.linear.z << ", "
    		<< odometry_msg.twist.twist.angular.x << ", "
    		<< odometry_msg.twist.twist.angular.y << ", "
    		<< odometry_msg.twist.twist.angular.z << ", "
    		<< integrated_pose_.getOrigin().x() << ", "
			  << integrated_pose_.getOrigin().y() << ", "
			  << integrated_pose_.getOrigin().z() << ", "
    		<< integrated_campose_q.x() << ", "
    		<< integrated_campose_q.y() << ", "
    		<< integrated_campose_q.z() << ", "
    		<< integrated_campose_q.w() << ", ";

    	posestream_.flush();
    }
    else
    {
    	ROS_WARN("posestream not open");
    }
#endif

    if (publish_tf_ && publish_result)
    {
      if (invert_tf_)
      {
        tf_broadcaster_.sendTransform(
          tf::StampedTransform(base_transform.inverse(), timestamp,
                               base_link_frame_id_, odom_frame_id_));
      }
      else
      {
        tf_broadcaster_.sendTransform(
          tf::StampedTransform(base_transform, timestamp,
                               odom_frame_id_, base_link_frame_id_));
      }
    }

    last_update_time_ = timestamp;
  }


  bool resetPose(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
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
    const tf::Transform       &tf,
    boost::array<double, 36>  &cov)
  {
    tf::Matrix3x3 covT(cov[ 0], cov[ 1], cov[ 2],
                       cov[ 6], cov[ 7], cov[ 8],
                       cov[12], cov[13], cov[14]);

    tf::Matrix3x3 covR(cov[21], cov[22], cov[23],
                       cov[27], cov[28], cov[29],
                       cov[33], cov[34], cov[35]);

    covT = tf.getBasis() * covT * tf.getBasis().transpose();
    covR = tf.getBasis() * covR * tf.getBasis().transpose();

    for (int r = 0; r < 3; ++r)
    {
      cov[r*6  ] = covT.getRow(r).x();
      cov[r*6+1] = covT.getRow(r).y();
      cov[r*6+2] = covT.getRow(r).z();

      cov[(r+3)*6+3] = covR.getRow(r).x();
      cov[(r+3)*6+4] = covR.getRow(r).y();
      cov[(r+3)*6+5] = covR.getRow(r).z();
    }
  }
};

} // end of namespace

#endif

