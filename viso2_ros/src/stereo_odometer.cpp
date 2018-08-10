#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <libviso2/viso_stereo.h>

#include <viso2_ros/VisoInfo.h>

#include "stereo_processor.h"
#include "odometer_base.h"
#include "odometry_params.h"

// to remove after debugging
#include <opencv2/highgui/highgui.hpp>

/** Create a feature flow visualization window */
//#define DBG_CREATE_VISUALIZATION_IMAGES
#ifdef DBG_CREATE_VISUALIZATION_IMAGES
/** Provide a "path/to/images" for storing flow visualization images there */
#define DBG_CREATE_VISUALIZATION_IMAGES_PATH "/path/to/images"
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/opencv.hpp>
#include <strstream>
#endif


namespace viso2_ros
{

// some arbitrary values (0.1m^2 linear cov. 10deg^2. angular cov.)
static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
		0, 0.1, 0, 0, 0, 0,
		0, 0, 0.1, 0, 0, 0,
		0, 0, 0, 0.17, 0, 0,
		0, 0, 0, 0, 0.17, 0,
		0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.05, 0, 0, 0, 0, 0,
		0, 0.05, 0, 0, 0, 0,
		0, 0, 0.05, 0, 0, 0,
		0, 0, 0, 0.09, 0, 0,
		0, 0, 0, 0, 0.09, 0,
		0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 99999, 0, 0, 0, 0, 0,
		0, 99999, 0, 0, 0, 0,
		0, 0, 99999, 0, 0, 0,
		0, 0, 0, 99999, 0, 0,
		0, 0, 0, 0, 99999, 0,
		0, 0, 0, 0, 0, 99999 } };


class StereoOdometer : public StereoProcessor, public OdometerBase
{

private:

  boost::shared_ptr<VisualOdometryStereo> visual_odometer_;
  VisualOdometryStereo::parameters visual_odometer_params_;

  ros::Publisher point_cloud_pub_;
  ros::Publisher info_pub_;
  ros::Publisher diagnostic_pub_;

  bool got_lost_;

  // change reference frame method. 0, 1 or 2. 0 means allways change. 1 and 2 explained below
  int ref_frame_change_method_;
  bool change_reference_frame_;
  double ref_frame_motion_threshold_; // method 1. Change the reference frame if last motion is small
  int ref_frame_inlier_threshold_; // method 2. Change the reference frame if the number of inliers is low
  Matrix reference_motion_;

public:

  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

  StereoOdometer(const std::string& transport) :
    StereoProcessor(transport), OdometerBase(),
    got_lost_(false), change_reference_frame_(false)
  {
    // Read local parameters
    ros::NodeHandle local_nh("~");
    odometry_params::loadParams(local_nh, visual_odometer_params_);

    local_nh.param("ref_frame_change_method", ref_frame_change_method_, 0);
    local_nh.param("ref_frame_motion_threshold", ref_frame_motion_threshold_, 5.0);
    local_nh.param("ref_frame_inlier_threshold", ref_frame_inlier_threshold_, 150);

    point_cloud_pub_ = local_nh.advertise<PointCloud>("point_cloud", 1);
    info_pub_ = local_nh.advertise<VisoInfo>("info", 1);
    diagnostic_pub_ = local_nh.advertise<diagnostic_msgs::DiagnosticStatus>("diagnostic", 1);

    reference_motion_ = Matrix::eye(4);

#ifdef DBG_EXPORT_TRAJECTORY
    std::string 	fnTraj;
		if (local_nh.getParam("file_trajectory",     fnTraj))
		{
			// Open the file for binary writing
			posestream_.open(fnTraj.c_str(), std::ios::out | std::ios::trunc);

			if (posestream_.is_open())
			{
		    	posestream_
		    		<< "stamp_sec, "
		    		<< "stamp_nsec, "
		    		<< "position_x, "
		    		<< "position_y, "
		    		<< "position_z, "
		    		<< "orientation_x, "
		    		<< "orientation_y, "
		    		<< "orientation_z, "
		    		<< "orientation_w, "
		    		<< "twist_linear_x, "
		    		<< "twist_linear_y, "
		    		<< "twist_linear_z, "
		    		<< "twist_angular_x, "
		    		<< "twist_angular_y, "
		    		<< "twist_angular_z, "
		    		<< "cam_position_x, "
					  << "cam_position_y, "
					  << "cam_position_z, "
		    		<< "cam_orientation_x, "
		    		<< "cam_orientation_y, "
		    		<< "cam_orientation_z, "
		    		<< "cam_orientation_w, "
					  << "nof_inliers" << std::endl;

		    	posestream_.flush();
			}
			else
			{
				ROS_ERROR("Cannot open file %s", fnTraj.c_str());
				throw(-3);
			}
		}
		else
		{
			ROS_WARN("No trajctory filename provided!");
		}
#endif
  }

protected:

  void initOdometer(
    const sensor_msgs::CameraInfoConstPtr& l_info_msg,
    const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    int queue_size;
    bool approximate_sync;
    ros::NodeHandle local_nh("~");
    local_nh.param("queue_size", queue_size, 5);
    local_nh.param("approximate_sync", approximate_sync, false);

    // read calibration info from camera info message
    // to fill remaining parameters
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(*l_info_msg, *r_info_msg);
    visual_odometer_params_.base = model.baseline();
    visual_odometer_params_.calib.cu = model.left().cx();
    visual_odometer_params_.calib.cv = model.left().cy();
    visual_odometer_params_.calib.f = model.left().fx();

    visual_odometer_.reset(new VisualOdometryStereo(visual_odometer_params_));
    if (l_info_msg->header.frame_id != "") setSensorFrameId(l_info_msg->header.frame_id);
    ROS_INFO_STREAM("Initialized libviso2 stereo odometry "
                    "with the following parameters:" << std::endl <<
                    visual_odometer_params_ <<
                    "  queue_size = " << queue_size << std::endl <<
                    "  approximate_sync = " << approximate_sync << std::endl <<
                    "  ref_frame_change_method = " << ref_frame_change_method_ << std::endl <<
                    "  ref_frame_motion_threshold = " << ref_frame_motion_threshold_ << std::endl <<
                    "  ref_frame_inlier_threshold = " << ref_frame_inlier_threshold_);
  }

  void imageCallback(
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::ImageConstPtr& r_image_msg,
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg)
  {
    ros::WallTime start_time = ros::WallTime::now();
		ros::Time start_time_ros(start_time.sec, start_time.nsec);
		ros::Duration delay = ros::Time::now() - l_image_msg->header.stamp;
		ROS_INFO("Input delay: %.3f", delay.toSec());
    bool first_run = false;
    // create odometer if not exists
    if (!visual_odometer_)
    {
      first_run = true;
      initOdometer(l_info_msg, r_info_msg);
    }

    // convert images if necessary
    uint8_t *l_image_data, *r_image_data;
    int l_step, r_step;
    cv_bridge::CvImageConstPtr l_cv_ptr, r_cv_ptr;
    l_cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8);
    l_image_data = l_cv_ptr->image.data;
    l_step = l_cv_ptr->image.step[0];
    r_cv_ptr = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8);
    r_image_data = r_cv_ptr->image.data;
    r_step = r_cv_ptr->image.step[0];

    ROS_ASSERT(l_step == r_step);
    ROS_ASSERT(l_image_msg->width == r_image_msg->width);
    ROS_ASSERT(l_image_msg->height == r_image_msg->height);

    int32_t dims[] = {l_image_msg->width, l_image_msg->height, l_step};
    // on first run or when odometer got lost, only feed the odometer with
    // images without retrieving data
    if (first_run || got_lost_)
    {
      visual_odometer_->process(l_image_data, r_image_data, dims);
      got_lost_ = false;
      // on first run publish zero once
      if (first_run)
      {
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);
#ifdef DBG_EXPORT_TRAJECTORY
        // Number of inliers
        if (posestream_.is_open())
        {
          posestream_ << visual_odometer_->getNumberOfInliers() << std::endl;
          posestream_.flush();
        }
        else
        {
          ROS_WARN("posestream not open");
        }
#endif
      }
    }
    else
    {
      bool success = visual_odometer_->process(
          l_image_data, r_image_data, dims, change_reference_frame_);
      if (success)
      {
        Matrix motion = Matrix::inv(visual_odometer_->getMotion());
        ROS_DEBUG("Found %i matches with %i inliers.",
                  visual_odometer_->getNumberOfMatches(),
                  visual_odometer_->getNumberOfInliers());
        ROS_DEBUG_STREAM("libviso2 returned the following motion:\n" << motion);
        Matrix camera_motion;
        // if image was replaced due to small motion we have to subtract the
        // last motion to get the increment
        if (change_reference_frame_)
        {
          camera_motion = Matrix::inv(reference_motion_) * motion;
        }
        else
        {
          // image was not replaced, report full motion from odometer
          camera_motion = motion;
        }
        reference_motion_ = motion; // store last motion as reference

        std::vector<Matcher::p_match> matches = visual_odometer_->getMatches();
        std::vector<int> inlier_indices = visual_odometer_->getInlierIndices();

        // Set up delta
        tf::Matrix3x3 rot_mat(
          camera_motion.val[0][0], camera_motion.val[0][1], camera_motion.val[0][2],
          camera_motion.val[1][0], camera_motion.val[1][1], camera_motion.val[1][2],
          camera_motion.val[2][0], camera_motion.val[2][1], camera_motion.val[2][2]);
        tf::Vector3 t(camera_motion.val[0][3], camera_motion.val[1][3], camera_motion.val[2][3]);
        tf::Transform delta_transform(rot_mat, t);

        // Publish diagnostics
        diagnostic_msgs::DiagnosticStatus ds;
        ds.hardware_id = ros::this_node::getName();
        ds.level = diagnostic_msgs::DiagnosticStatus::OK;
        ds.name = ros::this_node::getName();
        ds.message = "";
        ds.values.resize(1);
        ds.values[0].key = "d";
        ds.values[0].value = "0.0";
        diagnostic_pub_.publish(ds);

        // Set the covariance
        switch (cov_mode_)
        {
          case CovModeStandard:
          {
            setPoseCovariance(STANDARD_POSE_COVARIANCE);
            setTwistCovariance(STANDARD_TWIST_COVARIANCE);
            break;
          }
          case CovModeInlierBased:
          {
            // Set covariance based on number of inliers
            double covPos = 999999.0, covOri = 999999.0;

            int nof_inliers = visual_odometer_->getNumberOfInliers();

            double w1 = 0.0, w2 = 0.0;

            if (nof_inliers > nof_inliers_min_)
            {
              if (nof_inliers > nof_inliers_ok_)
              {
                // Matching is good
                nof_inliers = nof_inliers > nof_inliers_good_ ? nof_inliers_good_ : nof_inliers;
                w2 = (double)(nof_inliers - nof_inliers_ok_) / (double)(nof_inliers_good_ - nof_inliers_ok_);
                w1 = 1.0 - w2;
                covPos = w1 * cov_pos_ok_ + w2 * cov_pos_good_;
                covOri = w1 * cov_ori_ok_ + w2 * cov_ori_good_;
              }
              else
              {
                // Matching is useable
                w2 = (double)(nof_inliers - nof_inliers_min_) / (double)(nof_inliers_ok_ - nof_inliers_min_);
                w1 = 1.0 - w2;
                covPos = w1 * cov_pos_min_ + w2 * cov_pos_ok_;
                covOri = w1 * cov_ori_min_ + w2 * cov_ori_ok_;
              }
            }

            ROS_INFO("nof inliers: %d  pos: %.3f  ori: %.3f", nof_inliers, covPos, covOri);

            boost::array<double, 36> pose_covariance =
              { { covPos, 0, 0, 0, 0, 0,
                  0, covPos, 0, 0, 0, 0,
                  0, 0, covPos, 0, 0, 0,
                  0, 0, 0, covOri, 0, 0,
                  0, 0, 0, 0, covOri, 0,
                  0, 0, 0, 0, 0, covOri } };

            setPoseCovariance(pose_covariance);
            setTwistCovariance(pose_covariance);
            break;
          }
          case CovModeSvd:
          {
            // Set the covariance from the estimation
            boost::array<double, 36> pose_covariance =
              { { visual_odometer_->getCovariance()[3], 0, 0, 0, 0, 0,
                  0, visual_odometer_->getCovariance()[4], 0, 0, 0, 0,
                  0, 0, visual_odometer_->getCovariance()[5], 0, 0, 0,
                  0, 0, 0, visual_odometer_->getCovariance()[0], 0, 0,
                  0, 0, 0, 0, visual_odometer_->getCovariance()[1], 0,
                  0, 0, 0, 0, 0, visual_odometer_->getCovariance()[2] } };

            setPoseCovariance(pose_covariance);
            setTwistCovariance(pose_covariance);

            ROS_DEBUG("cov in camera: %.3f %.3f %.3f %.3f %.3f %.3f",
                      pose_covariance[ 0], pose_covariance[ 7], pose_covariance[14],
                      pose_covariance[21], pose_covariance[28], pose_covariance[35]);

            break;
          }
          default:
            break;
        }

        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        if (point_cloud_pub_.getNumSubscribers() > 0)
        {
					computeAndPublishPointCloud(l_info_msg, l_image_msg, r_info_msg, matches, inlier_indices);
        }

#ifdef DBG_CREATE_VISUALIZATION_IMAGES
        cv_bridge::CvImagePtr pImgL = cv_bridge::toCvCopy(l_image_msg, sensor_msgs::image_encodings::RGB8);

        cv::Point2f pt1;
        cv::Point2f pt2;
        cv::Point2f ptd;

        float ptDist = 0.0f;

        Matcher::p_match 	goodMatch;

        cv::Mat pxVal(1, 1, CV_8UC1, cv::Scalar(0));
        cv::Mat pxRGB(1, 1, CV_8UC3, cv::Scalar(0, 0, 0));

        if (success)
        {
          // Plot all matches
          for (u_int32_t i = 0; i < matches.size(); ++i)
          {
            goodMatch = matches[i];

            pt1.x = goodMatch.u1p;  pt1.y = goodMatch.v1p;
            pt2.x = goodMatch.u1c;  pt2.y = goodMatch.v1c;

            line(pImgL->image, pt1, pt2, cv::Scalar(255, 0, 0), 2);
            line(pImgL->image, pt1, pt1, cv::Scalar(255, 0, 0), 5);
          }

          // Plot inliers
          for (u_int32_t i = 0; i < inlier_indices.size(); ++i)
          {
            goodMatch = matches[inlier_indices[i]];

            pt1.x = goodMatch.u1p; 	pt1.y = goodMatch.v1p;
            pt2.x = goodMatch.u1c; 	pt2.y = goodMatch.v1c;

            // Colormap
            ptd = pt2 - pt1;
            ptDist = sqrtf(ptd.x * ptd.x + ptd.y * ptd.y);
            ptDist = rintf(ptDist * (84.0f / 35.0f));
            pxVal.at<uchar>(0,0) = 84 - (ptDist <= 84 ? ptDist : 84);
//						ROS_DEBUG("%.1f, %d", ptDist, pxVal.at<uchar>(0,0));
            cv::applyColorMap(pxVal, pxRGB, cv::COLORMAP_HSV);
//						ROS_DEBUG("%d, %d, %d", pxRGB.at<cv::Vec3b>(0, 0)[0], pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]);

            line(pImgL->image, pt1, pt2, cv::Scalar(pxRGB.at<cv::Vec3b>(0, 0)[0], pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]), 2);
            line(pImgL->image, pt1, pt1, cv::Scalar(pxRGB.at<cv::Vec3b>(0, 0)[0], pxRGB.at<cv::Vec3b>(0, 0)[1], pxRGB.at<cv::Vec3b>(0, 0)[2]), 5);
          }
        }

        imshow("matches", pImgL->image);
        cv::waitKey(1);

  #ifdef DBG_CREATE_VISUALIZATION_IMAGES_PATH
        // Write the image to disk
        std::stringstream fnStream("");
        fnStream << std::string(DBG_CREATE_VISUALIZATION_IMAGES_PATH) << std::string("/img_");
        fnStream << std::setw(8) << std::setfill ('0') << l_image_msg->header.seq << ".png";
        std::vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);
        cv::imwrite(fnStream.str(), pImgL->image, compression_params);
    #endif
#endif
      }
      else
      {
        setPoseCovariance(BAD_COVARIANCE);
        setTwistCovariance(BAD_COVARIANCE);
        tf::Transform delta_transform;
        delta_transform.setIdentity();
        integrateAndPublish(delta_transform, l_image_msg->header.stamp);

        ROS_DEBUG("Call to VisualOdometryStereo::process() failed.");
				ROS_WARN_THROTTLE(1.0, "Visual Odometer got lost!");
        got_lost_ = true;
      }

      if(success)
      {

        // Proceed depending on the reference frame change method
        switch ( ref_frame_change_method_ )
        {
          case 1:
          {
            // calculate current feature flow
            double feature_flow = computeFeatureFlow(visual_odometer_->getMatches());
            change_reference_frame_ = (feature_flow < ref_frame_motion_threshold_);
            ROS_DEBUG_STREAM("Feature flow is " << feature_flow
                << ", marking last motion as "
                << (change_reference_frame_ ? "small." : "normal."));
            break;
          }
          case 2:
          {
            change_reference_frame_ = (visual_odometer_->getNumberOfInliers() > ref_frame_inlier_threshold_);
            break;
          }
          default:
            change_reference_frame_ = false;
        }

      }
      else
        change_reference_frame_ = false;

      if(!change_reference_frame_)
        ROS_DEBUG_STREAM("Changing reference frame");

#ifdef DBG_EXPORT_TRAJECTORY
      // Number of inliers
			if (posestream_.is_open())
			{
				posestream_ << visual_odometer_->getNumberOfInliers() << std::endl;
				posestream_.flush();
			}
			else
			{
				ROS_WARN("posestream not open");
			}
#endif

      // Delay of the output
      delay = ros::Time::now() - l_image_msg->header.stamp;
      ROS_INFO("Output delay: %.3f", delay.toSec());

      // create and publish viso2 info msg
      VisoInfo info_msg;
      info_msg.header.stamp = l_image_msg->header.stamp;
      info_msg.got_lost = !success;
      info_msg.change_reference_frame = !change_reference_frame_;
      info_msg.num_matches = visual_odometer_->getNumberOfMatches();
      info_msg.num_inliers = visual_odometer_->getNumberOfInliers();
      ros::WallDuration time_elapsed = ros::WallTime::now() - start_time;
      info_msg.runtime = time_elapsed.toSec();
      info_pub_.publish(info_msg);
    }
  }

  double computeFeatureFlow(
      const std::vector<Matcher::p_match>& matches)
  {
    double total_flow = 0.0;
    for (size_t i = 0; i < matches.size(); ++i)
    {
      double x_diff = matches[i].u1c - matches[i].u1p;
      double y_diff = matches[i].v1c - matches[i].v1p;
      total_flow += sqrt(x_diff * x_diff + y_diff * y_diff);
    }
    return total_flow / matches.size();
  }

  void computeAndPublishPointCloud(
      const sensor_msgs::CameraInfoConstPtr& l_info_msg,
      const sensor_msgs::ImageConstPtr& l_image_msg,
      const sensor_msgs::CameraInfoConstPtr& r_info_msg,
      const std::vector<Matcher::p_match>& matches,
      const std::vector<int32_t>& inlier_indices)
  {
    try
    {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::RGB8);
      // read calibration info from camera info message
      image_geometry::StereoCameraModel model;
      model.fromCameraInfo(*l_info_msg, *r_info_msg);
      PointCloud::Ptr point_cloud(new PointCloud());
      point_cloud->header.frame_id = getSensorFrameId();
      point_cloud->header.stamp = pcl_conversions::toPCL(l_info_msg->header).stamp;
      point_cloud->width = 1;
      point_cloud->height = inlier_indices.size();
      point_cloud->points.resize(inlier_indices.size());

      for (size_t i = 0; i < inlier_indices.size(); ++i)
      {
        const Matcher::p_match& match = matches[inlier_indices[i]];
        cv::Point2d left_uv;
        left_uv.x = match.u1c;
        left_uv.y = match.v1c;
        cv::Point3d point;
        double disparity = match.u1c - match.u2c;
        model.projectDisparityTo3d(left_uv, disparity, point);
        point_cloud->points[i].x = point.x;
        point_cloud->points[i].y = point.y;
        point_cloud->points[i].z = point.z;
        cv::Vec3b color = cv_ptr->image.at<cv::Vec3b>(left_uv.y,left_uv.x);
        point_cloud->points[i].r = color[0];
        point_cloud->points[i].g = color[1];
        point_cloud->points[i].b = color[2];
      }
      ROS_DEBUG("Publishing point cloud with %zu points.", point_cloud->size());
      point_cloud_pub_.publish(point_cloud);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

} // end of namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_odometer");
  if (ros::names::remap("stereo") == "stereo") {
    ROS_WARN("'stereo' has not been remapped! Example command-line usage:\n"
             "\t$ rosrun viso2_ros stereo_odometer stereo:=narrow_stereo image:=image_rect");
  }
  if (ros::names::remap("image").find("rect") == std::string::npos) {
    ROS_WARN("stereo_odometer needs rectified input images. The used image "
             "topic is '%s'. Are you sure the images are rectified?",
             ros::names::remap("image").c_str());
  }

  std::string transport = argc > 1 ? argv[1] : "raw";
  viso2_ros::StereoOdometer odometer(transport);

  ros::spin();
  return 0;
}

